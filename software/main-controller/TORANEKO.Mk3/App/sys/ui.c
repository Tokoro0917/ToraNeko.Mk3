/*
 * ui.c
 * 機体の操作：LED の表示と合図、タイヤ・スイッチ・前センサの読み取り（MODES.md の 1〜3）
 *
 * ■ LED の並び（上から見た図。上が機体の前）
 *
 *             LED3  LED4  LED5          ← モード（4・2・1）
 *      SW
 *      LED1  LED2        LED6  LED7     ← No.（8・4・2・1）
 *
 * ■ 入力の読み方
 *   - タイヤ：エンコーダの「回った量」の差を足していき、UI_TIRE_COUNT_PER_STEP ごとに 1 段とする
 *   - スイッチ：押した瞬間に 1 回だけ「押された」と答える。2 回続けて同じ値を読んだときだけ変わったとみなし、
 *               チャタリング（押した瞬間に入ったり切れたりすること）を取り除く
 *   - 前センサの手：HAND_STABLE_COUNT 回続けて同じ値を読んだときだけ、かざした／離したとみなす
 */
#include "sys/ui.h"

#include "ctrl/wall_ctrl.h"
#include "drv/encoder.h"
#include "drv/led.h"
#include "drv/switch.h"
#include "drv/timebase.h"

#define BLINK_MS           150   /* 全部を点滅させるときの、点けている時間・消している時間 [ms] */
#define WAIT_BLINK_MS      250   /* スタート待ちで、モードと No. を点滅させる間隔 [ms] */
#define SIGNAL_OK_MS       1000  /* 「うまくいった」で全部を点けておく時間 [ms] */
#define SIGNAL_ERROR_MS    80    /* 「できなかった」で外側の 4 個を速く点滅させる間隔 [ms] */
#define SIGNAL_ERROR_TIMES 5     /* 「できなかった」の点滅の回数 */
#define START_DELAY_MS     1000  /* 手を離してから走り出すまで [ms] */
#define HAND_STABLE_COUNT  3     /* 手をかざした／離したとみなすまでに、同じ値が続く回数（UI_POLL_MS ごと） */
#define FAILSAFE_BLINK_MS  500   /* フェイルセーフで止まったときに、全部を点滅させる間隔 [ms] */
#define LOADING_STEP_MS    150   /* 「ロード中」で、LED を 1 個ずつ点けていく間隔 [ms]（7 個で約 1 秒） */

/* タイヤ：前に読んだときの回った量と、まだ 1 段にならない回転量 [カウント] */
static int32_t s_left_last_count = 0;
static int32_t s_left_rest = 0;
static int32_t s_right_last_count = 0;
static int32_t s_right_rest = 0;

/* スイッチ：前に読んだ値と、チャタリングを取り除いたあとの状態（true なら押されている） */
static bool s_switch_last_read = false;
static bool s_switch_state = false;

/* 起動時に 1 回呼ぶ。LED を消し、今のタイヤとスイッチの状態を覚える */
void ui_init(void)
{
  led_all_off();
  ui_forget_tire_rotation();
  s_switch_last_read = switch_is_pressed();
  s_switch_state = s_switch_last_read;
}

/* ---------------------------------------------------------------------------
 * LED の表示
 * ------------------------------------------------------------------------- */

/* 2 進数の 1 桁を LED に出す。残りの値 rest がその桁の重み weight 以上なら LED を点けて、
 * 重みを引いた残りを返す。小さければ LED を消して、rest をそのまま返す（上の桁から順に呼ぶ） */
static int show_digit(uint8_t led_num, int rest, int weight)
{
  if (rest >= weight) {
    led_set(led_num, true);
    return rest - weight;
  }
  led_set(led_num, false);
  return rest;
}

/* モード（0〜7）を真ん中の 3 個に、No.（0〜15）を外側の 4 個に、2 進数で出す */
void ui_show_mode_number(int mode, int number)
{
  int rest = mode;
  rest = show_digit(3, rest, 4);
  rest = show_digit(4, rest, 2);
  show_digit(5, rest, 1);

  rest = number;
  rest = show_digit(1, rest, 8);
  rest = show_digit(2, rest, 4);
  rest = show_digit(6, rest, 2);
  show_digit(7, rest, 1);
}

/* 0〜31 の値を、LED3〜LED7 に 2 進数で出す（LED3 = 16、LED4 = 8、LED5 = 4、LED6 = 2、LED7 = 1）。
 * ゴールの位置を入れるときに使う。LED1・LED2 は消す */
void ui_show_value(int value)
{
  led_set(1, false);
  led_set(2, false);
  int rest = value;
  rest = show_digit(3, rest, 16);
  rest = show_digit(4, rest, 8);
  rest = show_digit(5, rest, 4);
  rest = show_digit(6, rest, 2);
  show_digit(7, rest, 1);
}

/* 7 個全部を点ける */
void ui_all_on(void)
{
  for (uint8_t num = 1; num <= LED_COUNT; num++) {
    led_set(num, true);
  }
}

/* 7 個全部を消す */
void ui_all_off(void)
{
  led_all_off();
}

/* 外側の 4 個（LED1・LED2・LED6・LED7）を点ける／消す */
static void set_outer_leds(bool on)
{
  led_set(1, on);
  led_set(2, on);
  led_set(6, on);
  led_set(7, on);
}

/* ---------------------------------------------------------------------------
 * 合図
 * ------------------------------------------------------------------------- */

/* 7 個全部を times 回点滅させる（決定したとき 1 回、ゴールの y の入力に移るとき 2 回） */
void ui_blink_all(int times)
{
  for (int i = 0; i < times; i++) {
    ui_all_on();
    ui_wait_ms(BLINK_MS);
    ui_all_off();
    ui_wait_ms(BLINK_MS);
  }
}

/* 「うまくいった」：7 個全部を 1 秒点ける */
void ui_signal_ok(void)
{
  ui_all_on();
  ui_wait_ms(SIGNAL_OK_MS);
  ui_all_off();
}

/* 「できなかった」：外側の 4 個を速く 5 回点滅させる */
void ui_signal_error(void)
{
  ui_all_off();
  for (int i = 0; i < SIGNAL_ERROR_TIMES; i++) {
    set_outer_leds(true);
    ui_wait_ms(SIGNAL_ERROR_MS);
    set_outer_leds(false);
    ui_wait_ms(SIGNAL_ERROR_MS);
  }
}

/* 時刻 until_ms [ms] になるまで待つ。待っているあいだに押されたスイッチは読み捨てる（あとで「押された」と答えないようにする）。
 * 待つ回数ではなく時刻で決めるのは、実機の 1 回の待ち（HAL_Delay）が少し長くなることがあり、回数で数えるとずれがたまるため */
static void wait_until_ignoring_switch(uint32_t until_ms)
{
  while (timebase_get_ms() < until_ms) {
    ui_switch_pushed();  /* 読むだけで、答えは使わない */
    ui_wait_ms(UI_POLL_MS);
  }
}

/* 「ロード中」：LED1 から LED7 へ 1 個ずつ順に点けていき（左下 → 上 → 右下）、全部点いたら消す（約 1 秒）。
 * このあいだはスイッチを押しても何も起きない（続けて押したときに、次の決定にならないようにする） */
void ui_show_loading(void)
{
  ui_all_off();
  uint32_t next_ms = timebase_get_ms();
  for (uint8_t num = 1; num <= LED_COUNT; num++) {
    led_set(num, true);
    next_ms = next_ms + LOADING_STEP_MS;
    wait_until_ignoring_switch(next_ms);
  }
  ui_all_off();
}

/* ---------------------------------------------------------------------------
 * 入力
 * ------------------------------------------------------------------------- */

/* これまでにタイヤを回した量を忘れる。モードを選び始めるときに呼ぶ
 * （走っているあいだにタイヤが回った量で、モードや No. が変わらないようにするため） */
void ui_forget_tire_rotation(void)
{
  s_left_last_count = encoder_get_left_total();
  s_left_rest = 0;
  s_right_last_count = encoder_get_right_total();
  s_right_rest = 0;
}

/* 回転量 rest [カウント] が何段ぶんか。前に回したら正、後ろに回したら負。
 * 1 段に足りない分は数えない（C の割り算は、0 の方へ切り捨てる。例：-350 / 300 = -1） */
static int steps_in(int32_t rest)
{
  return rest / UI_TIRE_COUNT_PER_STEP;
}

/* 前に読んでから、左タイヤが何段回ったか（前に回したら正、後ろに回したら負） */
int ui_read_left_steps(void)
{
  int32_t count = encoder_get_left_total();
  s_left_rest = s_left_rest + (count - s_left_last_count);
  s_left_last_count = count;

  int steps = steps_in(s_left_rest);
  s_left_rest = s_left_rest - steps * UI_TIRE_COUNT_PER_STEP;
  return steps;
}

/* 前に読んでから、右タイヤが何段回ったか（前に回したら正、後ろに回したら負） */
int ui_read_right_steps(void)
{
  int32_t count = encoder_get_right_total();
  s_right_rest = s_right_rest + (count - s_right_last_count);
  s_right_last_count = count;

  int steps = steps_in(s_right_rest);
  s_right_rest = s_right_rest - steps * UI_TIRE_COUNT_PER_STEP;
  return steps;
}

/* スイッチが押された瞬間なら true（押しっぱなしでも、1 回押すごとに 1 回だけ true）。
 * UI_POLL_MS ごとに呼ぶ。2 回続けて同じ値を読んだときだけ、状態が変わったとみなす（チャタリングを取り除く） */
bool ui_switch_pushed(void)
{
  bool now_read = switch_is_pressed();
  bool pushed = false;

  if (now_read == s_switch_last_read && now_read != s_switch_state) {
    s_switch_state = now_read;
    if (s_switch_state) {
      pushed = true;  /* 離れた状態から、押された状態に変わった */
    }
  }
  s_switch_last_read = now_read;
  return pushed;
}

/* wait_ms [ms] 待つ */
void ui_wait_ms(uint32_t wait_ms)
{
  timebase_wait_ms(wait_ms);
}

/* ---------------------------------------------------------------------------
 * 待つ
 * ------------------------------------------------------------------------- */

/* 前センサに手がかざされる（want_hand が true）、または手が離れる（false）まで待つ。
 * HAND_STABLE_COUNT 回続けて同じ値を読んだら、そうなったとみなす */
static void wait_hand(bool want_hand)
{
  int stable_count = 0;
  while (stable_count < HAND_STABLE_COUNT) {
    if (wall_ctrl_is_hand_front() == want_hand) {
      stable_count = stable_count + 1;
    } else {
      stable_count = 0;
    }
    ui_wait_ms(UI_POLL_MS);
  }
}

/* スタート待ち（MODES.md の 2 の 3〜5）。走り出してよくなったら true、選び直すなら false。
 *   1. 選んだモードと No. を点滅させて、前センサに手がかざされるのを待つ。スイッチが押されたら false（選び直し）
 *   2. 手がかざされたら 7 個全部を点けて、手が離れるのを待つ
 *   3. 手が離れたら全部消して、1 秒待ってから true（このあいだにジャイロの 0 点を測る。ステップ4で足す） */
bool ui_wait_start(int mode, int number)
{
  bool lit = true;
  uint32_t next_toggle_ms = timebase_get_ms() + WAIT_BLINK_MS;
  int hand_count = 0;

  /* 1. 手がかざされるのを待つ */
  while (hand_count < HAND_STABLE_COUNT) {
    if (ui_switch_pushed()) {
      ui_all_off();
      return false;
    }
    if (wall_ctrl_is_hand_front()) {
      hand_count = hand_count + 1;
    } else {
      hand_count = 0;
    }

    /* WAIT_BLINK_MS ごとに、点ける・消すを入れ替える */
    if (timebase_get_ms() >= next_toggle_ms) {
      lit = !lit;
      next_toggle_ms = next_toggle_ms + WAIT_BLINK_MS;
    }
    if (lit) {
      ui_show_mode_number(mode, number);
    } else {
      ui_all_off();
    }
    ui_wait_ms(UI_POLL_MS);
  }

  /* 2. 全部点けて、手が離れるのを待つ */
  ui_all_on();
  wait_hand(false);

  /* 3. 全部消して、1 秒待つ */
  ui_all_off();
  ui_wait_ms(START_DELAY_MS);
  return true;
}

/* フェイルセーフで止まったあと（MODES.md の 3）。
 *   1. 7 個全部を FAILSAFE_BLINK_MS ごとに点滅させて、スイッチが押されるのを待つ
 *   2. 押されたら「ロード中」を出す。このあいだはスイッチを押しても何も起きない
 *   3. 全部消して戻る */
void ui_wait_failsafe_reset(void)
{
  bool lit = true;
  uint32_t next_toggle_ms = timebase_get_ms() + FAILSAFE_BLINK_MS;

  /* 1. スイッチが押されるのを待つ */
  while (!ui_switch_pushed()) {
    /* FAILSAFE_BLINK_MS ごとに、点ける・消すを入れ替える */
    if (timebase_get_ms() >= next_toggle_ms) {
      lit = !lit;
      next_toggle_ms = next_toggle_ms + FAILSAFE_BLINK_MS;
    }
    if (lit) {
      ui_all_on();
    } else {
      ui_all_off();
    }
    ui_wait_ms(UI_POLL_MS);
  }

  /* 2・3. 「ロード中」を出して、全部消す */
  ui_show_loading();
}
