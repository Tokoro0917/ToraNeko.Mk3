/*
 * test_mode.c
 * モード選択と操作（sys/mode.c、sys/ui.c、sys/goal.c）の PC 上でのテスト。実機がなくても動く
 *
 * ビルドと実行（このフォルダで）:  make
 *
 * ■ しくみ
 * mode.c・ui.c が使う関数（時計、LED、タイヤ、スイッチ、前センサの手、Flash、探索）を、ここで仮に作る。
 *   - 時計：timebase_wait_ms() が呼ばれると、その分だけ仮の時計を進める
 *   - 入力：「何 ms に、タイヤを何段回す・スイッチを押す・手をかざす」という予定を先に作っておき、
 *           仮の時計がその時刻を過ぎたら入力を変える
 *   - LED：点けた・消した時刻を全部覚えておき、あとで「この時刻にどの LED が点いていたか」を調べる
 *   - 探索：explore_run() は走らずに、呼ばれた時刻と中身を覚えて、決めておいた結果を返す
 *
 * ■ テストの内容
 *   1. LED の表し方（モードと No.、ゴールの値）
 *   2. 入力の読み方（タイヤの段、スイッチのチャタリング）
 *   3. ゴール（電源を入れたときに Flash から読む、使える位置か）
 *   4. モードを選んで、スタートして、探索を呼ぶまで（LED の表示と時刻も）
 *   5. No.0 では決定しない、スタート待ちでの選び直し、走ったあとも同じモード・No.、フェイルセーフ、ゴールへ行けなかった
 *   6. ログ出力、ゴールの位置の入力（使えない位置のときは x に戻る）、迷路呼び出し
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ctrl/wall_ctrl.h"
#include "drv/encoder.h"
#include "drv/led.h"
#include "drv/switch.h"
#include "drv/timebase.h"
#include "sys/explore.h"
#include "sys/failsafe.h"
#include "sys/flash.h"
#include "sys/goal.h"
#include "sys/mode.h"
#include "sys/output.h"
#include "sys/ui.h"

/* ---------------------------------------------------------------------------
 * テスト結果の数え方
 * ------------------------------------------------------------------------- */

static int s_fail_count = 0;

/* ok が false なら失敗として数え、message を表示する */
static void check(bool ok, const char *message)
{
  if (!ok) {
    s_fail_count = s_fail_count + 1;
    printf("  失敗: %s\n", message);
  }
}

/* ---------------------------------------------------------------------------
 * 仮の時計と、入力の予定
 * ------------------------------------------------------------------------- */

static uint32_t s_now_ms = 0;          /* 仮の時計 [ms] */
static uint32_t s_time_limit_ms = 0;   /* これを過ぎたら、終わらないとみなしてテストをやめる */

/* 入力の種類 */
typedef enum {
  INPUT_LEFT_COUNT,   /* 左タイヤを value カウント回す */
  INPUT_RIGHT_COUNT,  /* 右タイヤを value カウント回す */
  INPUT_SWITCH,       /* スイッチを押す（value = 1）・離す（0） */
  INPUT_HAND,         /* 前センサに手をかざす（value = 1）・離す（0） */
} input_kind_t;

/* 入力の予定 1 つ */
typedef struct {
  uint32_t time_ms;
  input_kind_t kind;
  int value;
} input_event_t;

#define EVENT_MAX 200
static input_event_t s_events[EVENT_MAX];  /* 時刻の早い順に並べておく */
static int s_event_count = 0;
static int s_event_next = 0;               /* 次に使う予定 */

/* 今の入力 */
static int32_t s_left_count = 0;
static int32_t s_right_count = 0;
static bool s_switch = false;
static bool s_hand = false;

/* 予定を 1 つ足す。時刻の早い順になるように、後ろから入れる場所を探して入れる */
static void add_event(uint32_t time_ms, input_kind_t kind, int value)
{
  if (s_event_count >= EVENT_MAX) {
    printf("  予定が多すぎる\n");
    exit(1);
  }
  int i = s_event_count;
  while (i > 0 && s_events[i - 1].time_ms > time_ms) {
    s_events[i] = s_events[i - 1];
    i = i - 1;
  }
  s_events[i].time_ms = time_ms;
  s_events[i].kind = kind;
  s_events[i].value = value;
  s_event_count = s_event_count + 1;
}

/* 予定を作る道具 */
static void rotate_left_at(uint32_t time_ms, int steps)
{
  add_event(time_ms, INPUT_LEFT_COUNT, steps * UI_TIRE_COUNT_PER_STEP);
}

static void rotate_right_at(uint32_t time_ms, int steps)
{
  add_event(time_ms, INPUT_RIGHT_COUNT, steps * UI_TIRE_COUNT_PER_STEP);
}

static void press_switch_at(uint32_t time_ms)
{
  add_event(time_ms, INPUT_SWITCH, 1);
  add_event(time_ms + 100, INPUT_SWITCH, 0);
}

static void hand_at(uint32_t time_ms, uint32_t duration_ms)
{
  add_event(time_ms, INPUT_HAND, 1);
  add_event(time_ms + duration_ms, INPUT_HAND, 0);
}

/* 仮の時計が過ぎた予定を、入力に反映する */
static void apply_events(void)
{
  while (s_event_next < s_event_count && s_events[s_event_next].time_ms <= s_now_ms) {
    input_event_t event = s_events[s_event_next];
    if (event.kind == INPUT_LEFT_COUNT) {
      s_left_count = s_left_count + event.value;
    } else if (event.kind == INPUT_RIGHT_COUNT) {
      s_right_count = s_right_count + event.value;
    } else if (event.kind == INPUT_SWITCH) {
      s_switch = (event.value == 1);
    } else {
      s_hand = (event.value == 1);
    }
    s_event_next = s_event_next + 1;
  }
}

/* ---------------------------------------------------------------------------
 * 仮の LED（点けた・消した時刻を覚えておく）
 * ------------------------------------------------------------------------- */

typedef struct {
  uint32_t time_ms;
  uint8_t num;
  bool on;
} led_change_t;

#define LED_CHANGE_MAX 20000
static led_change_t s_led_changes[LED_CHANGE_MAX];
static int s_led_change_count = 0;
static bool s_led_on[LED_COUNT + 1];  /* [1]〜[7] を使う */

/* time_ms の時点で、LED1〜LED7 が点いていたかを "1"（点）と "0"（消）の 7 文字で返す（左が LED1） */
static const char *leds_at(uint32_t time_ms)
{
  static char pattern[LED_COUNT + 1];
  for (int i = 0; i < LED_COUNT; i++) {
    pattern[i] = '0';
  }
  pattern[LED_COUNT] = '\0';
  for (int i = 0; i < s_led_change_count; i++) {
    if (s_led_changes[i].time_ms > time_ms) {
      break;
    }
    if (s_led_changes[i].on) {
      pattern[s_led_changes[i].num - 1] = '1';
    } else {
      pattern[s_led_changes[i].num - 1] = '0';
    }
  }
  return pattern;
}

/* time_ms の時点の LED が expected（"0110110" など）と同じか調べる */
static void check_leds_at(uint32_t time_ms, const char *expected, const char *message)
{
  const char *actual = leds_at(time_ms);
  if (strcmp(actual, expected) != 0) {
    printf("  %u ms の LED: %s（正しくは %s）\n", time_ms, actual, expected);
  }
  check(strcmp(actual, expected) == 0, message);
}

/* ---------------------------------------------------------------------------
 * 仮の Flash と探索
 * ------------------------------------------------------------------------- */

static bool s_flash_has_goal = false;    /* Flash にゴールが保存されているか */
static int s_flash_goal_x = 0;
static int s_flash_goal_y = 0;
static int s_flash_goal_save_count = 0;
static bool s_flash_has_maze = false;    /* Flash に迷路が保存されているか */
static int s_flash_maze_load_count = 0;

static int s_failsafe_clear_count = 0;

static int s_output_count = 0;    /* 出力が呼ばれた回数 */
static int s_output_goal_x = 0;   /* 出力に渡されたゴール */
static int s_output_goal_y = 0;

static explore_result_t s_explore_result = EXPLORE_RESULT_DONE;  /* 探索が返す結果 */
static int s_explore_count = 0;           /* 探索が呼ばれた回数 */
static explore_type_t s_explore_type = EXPLORE_ROUND_TRIP;
static int s_explore_goal[4];             /* 渡されたゴール（x、y、幅、高さ） */
static uint32_t s_explore_time_ms = 0;    /* 呼ばれた時刻 */

/* テストを始める前に、仮の時計・入力・LED・Flash・探索を最初の状態にする */
static void reset_world(void)
{
  s_now_ms = 0;
  s_time_limit_ms = 100000;
  s_event_count = 0;
  s_event_next = 0;
  s_left_count = 0;
  s_right_count = 0;
  s_switch = false;
  s_hand = false;
  s_led_change_count = 0;
  for (int i = 0; i <= LED_COUNT; i++) {
    s_led_on[i] = false;
  }
  s_flash_has_goal = false;
  s_flash_goal_save_count = 0;
  s_flash_has_maze = false;
  s_flash_maze_load_count = 0;
  s_failsafe_clear_count = 0;
  s_output_count = 0;
  s_explore_result = EXPLORE_RESULT_DONE;
  s_explore_count = 0;
  s_explore_time_ms = 0;
}

/* ---------------------------------------------------------------------------
 * mode.c・ui.c・goal.c が使う関数（ここで仮に作る）
 * ------------------------------------------------------------------------- */

uint32_t timebase_get_ms(void)
{
  return s_now_ms;
}

void timebase_wait_ms(uint32_t wait_ms)
{
  s_now_ms = s_now_ms + wait_ms;
  apply_events();
  if (s_now_ms > s_time_limit_ms) {
    printf("  失敗: %u ms たっても終わらない（入力の予定が足りないか、待ち続けている）\n", s_now_ms);
    exit(1);
  }
}

void led_set(uint8_t num, bool on)
{
  if (num < 1 || num > LED_COUNT) {
    return;
  }
  if (s_led_on[num] == on) {
    return;  /* 変わらないときは覚えない */
  }
  s_led_on[num] = on;
  if (s_led_change_count < LED_CHANGE_MAX) {
    s_led_changes[s_led_change_count].time_ms = s_now_ms;
    s_led_changes[s_led_change_count].num = num;
    s_led_changes[s_led_change_count].on = on;
    s_led_change_count = s_led_change_count + 1;
  }
}

void led_all_off(void)
{
  for (uint8_t num = 1; num <= LED_COUNT; num++) {
    led_set(num, false);
  }
}

int32_t encoder_get_left_total(void)
{
  return s_left_count;
}

int32_t encoder_get_right_total(void)
{
  return s_right_count;
}

bool switch_is_pressed(void)
{
  return s_switch;
}

bool wall_ctrl_is_hand_front(void)
{
  return s_hand;
}

void failsafe_clear(void)
{
  s_failsafe_clear_count = s_failsafe_clear_count + 1;
}

bool flash_load_maze(void)
{
  s_flash_maze_load_count = s_flash_maze_load_count + 1;
  return s_flash_has_maze;
}

void flash_save_goal(int goal_x, int goal_y)
{
  s_flash_has_goal = true;
  s_flash_goal_x = goal_x;
  s_flash_goal_y = goal_y;
  s_flash_goal_save_count = s_flash_goal_save_count + 1;
}

bool flash_load_goal(int *goal_x, int *goal_y)
{
  if (!s_flash_has_goal) {
    return false;
  }
  *goal_x = s_flash_goal_x;
  *goal_y = s_flash_goal_y;
  return true;
}

/* 出力：出さずに、呼ばれたことを覚える */
void output_all(int goal_x, int goal_y)
{
  s_output_count = s_output_count + 1;
  s_output_goal_x = goal_x;
  s_output_goal_y = goal_y;
}

/* 探索：走らずに、呼ばれたことを覚える。走っているあいだはタイヤが回るので、回った量も足す */
explore_result_t explore_run(explore_type_t type, int goal_x, int goal_y, int goal_width, int goal_height)
{
  s_explore_count = s_explore_count + 1;
  s_explore_type = type;
  s_explore_goal[0] = goal_x;
  s_explore_goal[1] = goal_y;
  s_explore_goal[2] = goal_width;
  s_explore_goal[3] = goal_height;
  s_explore_time_ms = s_now_ms;
  s_left_count = s_left_count + 100000;
  s_right_count = s_right_count + 77777;
  return s_explore_result;
}

/* ---------------------------------------------------------------------------
 * 1. LED の表し方
 * ------------------------------------------------------------------------- */

static void test_led_display(void)
{
  printf("[1] LED の表し方\n");
  reset_world();

  /* モード5 = LED3 + LED5（4 + 1）、No.6 = LED2 + LED6（4 + 2） */
  ui_show_mode_number(5, 6);
  check_leds_at(0, "0110110", "モード5・No.6 の表示");
  s_now_ms = 10;
  ui_show_mode_number(7, 15);
  check_leds_at(10, "1111111", "モード7・No.15 は全部点く");
  s_now_ms = 20;
  ui_show_mode_number(0, 0);
  check_leds_at(20, "0000000", "モード0・No.0 は全部消える");
  s_now_ms = 30;
  ui_show_mode_number(2, 9);
  check_leds_at(30, "1001001", "モード2（LED4）・No.9（LED1 + LED7）");

  /* ゴールの値：LED3 = 16、LED4 = 8、LED5 = 4、LED6 = 2、LED7 = 1 */
  s_now_ms = 40;
  ui_show_value(20);
  check_leds_at(40, "0010100", "値 20 = LED3 + LED5");
  s_now_ms = 50;
  ui_show_value(30);
  check_leds_at(50, "0011110", "値 30 = LED3〜LED6");
}

/* ---------------------------------------------------------------------------
 * 2. 入力の読み方
 * ------------------------------------------------------------------------- */

static void test_inputs(void)
{
  printf("[2] 入力の読み方\n");

  /* タイヤ：1 段に足りない回転は覚えておき、足すと 1 段になる。後ろに回すと負 */
  reset_world();
  ui_init();
  add_event(100, INPUT_RIGHT_COUNT, 150);
  add_event(200, INPUT_RIGHT_COUNT, 150);
  add_event(300, INPUT_RIGHT_COUNT, -350);
  ui_wait_ms(150);
  check(ui_read_right_steps() == 0, "半段だけ回しても、まだ 0 段");
  ui_wait_ms(100);
  check(ui_read_right_steps() == 1, "もう半段回すと 1 段");
  ui_wait_ms(100);
  check(ui_read_right_steps() == -1, "後ろに 1 段と少し回すと -1 段");
  check(ui_read_left_steps() == 0, "左タイヤは回していない");

  /* 回した量を忘れると、そのあと読んでも 0 段 */
  add_event(500, INPUT_LEFT_COUNT, 5000);
  ui_wait_ms(200);
  ui_forget_tire_rotation();
  check(ui_read_left_steps() == 0, "回した量を忘れたあとは 0 段");

  /* スイッチ：チャタリングがあっても 1 回押せば 1 回。押しっぱなしでも 1 回 */
  reset_world();
  ui_init();
  add_event(500, INPUT_SWITCH, 1);   /* 押した瞬間に、入ったり切れたりする */
  add_event(510, INPUT_SWITCH, 0);
  add_event(520, INPUT_SWITCH, 1);
  add_event(1500, INPUT_SWITCH, 0);  /* 1 秒押しっぱなしで離す */
  add_event(2000, INPUT_SWITCH, 1);  /* もう 1 回押す */
  add_event(2100, INPUT_SWITCH, 0);
  int push_count = 0;
  for (int i = 0; i < 300; i++) {
    if (ui_switch_pushed()) {
      push_count = push_count + 1;
    }
    ui_wait_ms(UI_POLL_MS);
  }
  check(push_count == 2, "2 回押したら、2 回だけ「押された」になる");
}

/* ---------------------------------------------------------------------------
 * 3. ゴール
 * ------------------------------------------------------------------------- */

static void test_goal(void)
{
  printf("[3] ゴール\n");
  reset_world();
  goal_init();
  check(goal_get_x() == 15 && goal_get_y() == 15, "Flash にゴールがなければ、中央の 2×2（15, 15）");

  s_flash_has_goal = true;
  s_flash_goal_x = 3;
  s_flash_goal_y = 4;
  goal_init();
  check(goal_get_x() == 3 && goal_get_y() == 4, "Flash のゴールを読む");

  s_flash_goal_x = 0;
  s_flash_goal_y = 0;
  goal_init();
  check(goal_get_x() == 15 && goal_get_y() == 15, "Flash のゴールがおかしい（スタートを含む）ときは中央");

  check(goal_is_valid(30, 30), "(30, 30) は使える（2×2 が迷路の右上の角に入る）");
  check(!goal_is_valid(31, 5), "(31, 5) は迷路からはみ出す");
  check(!goal_is_valid(0, 0), "(0, 0) はスタートの区画を含む");
  check(goal_is_valid(0, 1), "(0, 1) はスタートの区画を含まない");
  check(!goal_set(0, 0), "使えない位置は設定できない");
  check(s_flash_goal_save_count == 0, "使えない位置は Flash に保存しない");
}

/* ---------------------------------------------------------------------------
 * 4・5. モードを選んで、走らせる
 * ------------------------------------------------------------------------- */

/* モード0 No.1 を選んで、スタートして、探索を呼ぶまで。LED の表示と時刻も調べる。
 * 走ったあとにもう一度スタートすると、同じモード・No. のまま（走っているあいだにタイヤが回っても変わらない） */
static void test_select_and_start(void)
{
  printf("[4] モードを選んで、スタートして、探索を呼ぶまで\n");
  reset_world();
  s_flash_has_goal = true;
  s_flash_goal_x = 10;
  s_flash_goal_y = 12;
  mode_init();

  rotate_right_at(100, 1);   /* No.1 */
  press_switch_at(500);      /* 決定（510 ms に「押された」になる） */
  hand_at(1500, 600);        /* 手をかざして、2100 ms に離す */
  mode_run_once();

  check(s_explore_count == 1, "探索が 1 回呼ばれる");
  check(s_explore_type == EXPLORE_ROUND_TRIP, "No.1 は往復");
  check(s_explore_goal[0] == 10 && s_explore_goal[1] == 12, "Flash から読んだゴールを渡す");
  check(s_explore_goal[2] == 2 && s_explore_goal[3] == 2, "ゴールの大きさは 2×2");
  check(s_explore_time_ms >= 2100 + 1000 && s_explore_time_ms <= 2100 + 1100, "手を離してから 1 秒後に走り出す");
  printf("  手を離した 2100 ms → 走り出した %u ms\n", s_explore_time_ms);

  check_leds_at(300, "0000001", "選んでいるあいだは、モード0・No.1 を点けたまま");
  check_leds_at(550, "1111111", "決定すると、全部が 1 回点滅する（点いているところ）");
  check_leds_at(700, "0000000", "決定すると、全部が 1 回点滅する（消えたところ）");
  check_leds_at(900, "0000001", "スタート待ちは、モードと No. を点滅させる（点いているところ）");
  check_leds_at(1150, "0000000", "スタート待ちは、モードと No. を点滅させる（消えたところ）");
  check_leds_at(1800, "1111111", "手をかざしているあいだは、全部点く");
  check_leds_at(2500, "0000000", "手を離すと、全部消える");

  /* 2 回目：選び直さずにスイッチを押せば、同じモード・No. で走る */
  uint32_t second_start = s_explore_time_ms + 2000;
  press_switch_at(second_start);
  hand_at(second_start + 1000, 500);
  mode_run_once();
  check(mode_get_mode() == 0 && mode_get_number() == 1, "走ったあとも、モード0・No.1 のまま");
  check(s_explore_count == 2 && s_explore_type == EXPLORE_ROUND_TRIP, "2 回目も往復で走る");
}

/* No.0 では決定しない。そのあと No.3 にすると、未知壁探索で走る */
static void test_number_zero(void)
{
  printf("[5] No.0 ではスイッチを押しても決定しない\n");
  reset_world();
  mode_init();

  press_switch_at(200);        /* No.0 で押す：何も起きない */
  rotate_right_at(1000, 3);    /* No.3 */
  press_switch_at(1500);
  hand_at(2500, 500);
  mode_run_once();

  check(s_explore_count == 1 && s_explore_type == EXPLORE_UNKNOWN_WALLS, "No.3 は未知壁探索");
  check(s_explore_time_ms > 3000, "No.0 で押したときは始まらない");
  check_leds_at(300, "0000000", "No.0 で押しても、点滅しない（選んでいるまま）");
}

/* モードは 0 の前が 7、No. は 0 の前が 15（一周する）。することのない番号では決定しない */
static void test_wrap(void)
{
  printf("[5] モードと No. は一周する\n");
  reset_world();
  mode_init();

  rotate_left_at(100, -1);    /* モード7 */
  rotate_right_at(100, 3);    /* No.3 */
  press_switch_at(500);       /* モード7・No.3 は何もない：決定しない */
  rotate_right_at(1000, -4);  /* No.3 - 4 = No.15（迷路呼び出し） */
  press_switch_at(1500);
  hand_at(2500, 500);
  mode_run_once();

  check_leds_at(300, "0011111", "モード7・No.3 は LED3〜7 が点く");
  check_leds_at(1200, "1111111", "モード7・No.15 は全部点く");
  check(mode_get_mode() == 7 && mode_get_number() == 15, "モード7・No.15 になる");
  check(s_flash_maze_load_count == 1, "迷路呼び出しが 1 回だけ呼ばれる（No.3 では決定しない）");
}

/* スタート待ちでスイッチを押すと、選び直しに戻る（走らない） */
static void test_reselect(void)
{
  printf("[5] スタート待ちでスイッチを押すと、選び直し\n");
  reset_world();
  mode_init();

  rotate_right_at(100, 2);    /* No.2（片道） */
  press_switch_at(500);       /* 決定 */
  press_switch_at(1500);      /* スタート待ちで押す：選び直し */
  mode_run_once();
  check(s_explore_count == 0, "スタート待ちでスイッチを押すと走らない");
  check_leds_at(s_now_ms, "0000000", "選び直しに戻るとき、LED を消す");

  /* 選び直しで、そのまま決定してスタートすると、片道で走る */
  press_switch_at(s_now_ms + 500);
  hand_at(s_now_ms + 1500, 500);
  mode_run_once();
  check(s_explore_count == 1 && s_explore_type == EXPLORE_ONE_WAY, "No.2 は片道");
}

/* 探索がフェイルセーフで止まったら、全部を点滅させてスイッチを待ち、押されたら「ロード中」を出して、フェイルセーフを解除する。
 * 「ロード中」のあいだに押したスイッチは、次の決定にならない */
static void test_failsafe(void)
{
  printf("[5] 探索がフェイルセーフで止まったとき\n");
  reset_world();
  mode_init();
  s_explore_result = EXPLORE_RESULT_FAILSAFE;

  rotate_right_at(100, 1);
  press_switch_at(500);
  hand_at(1500, 500);
  hand_at(4500, 500);    /* フェイルセーフのあいだに手をかざして離しても、解除しない */
  press_switch_at(6000); /* スイッチで解除する。6000〜6030 ms に「ロード中」が始まる */
  press_switch_at(6400); /* 「ロード中」のあいだにもう一度押す：何も起きない */
  mode_run_once();

  uint32_t stop_ms = s_explore_time_ms;
  check(s_failsafe_clear_count == 1, "フェイルセーフを 1 回解除する");
  check(s_now_ms >= 6000, "手をかざしても解除せず、スイッチが押されるまで待つ");
  check_leds_at(stop_ms + 100, "1111111", "フェイルセーフのあいだは、全部を点滅させる（点いたところ）");
  check_leds_at(stop_ms + 600, "0000000", "フェイルセーフのあいだは、全部を点滅させる（消えたところ）");
  check_leds_at(stop_ms + 1100, "1111111", "0.5 秒ごとに点滅する");

  /* 「ロード中」：始まりが 6000〜6030 ms のどこでも、6090 + 150 × k ms には k + 1 個点いている */
  check_leds_at(6090, "1000000", "ロード中：LED1 から点ける");
  check_leds_at(6240, "1100000", "ロード中：次に LED2");
  check_leds_at(6540, "1111000", "ロード中：1 個ずつ増える");
  check_leds_at(6990, "1111111", "ロード中：最後に 7 個全部");
  check_leds_at(s_now_ms, "0000000", "ロード中が終わったら、全部消す");
  check(s_now_ms >= 7050 && s_now_ms <= 7100, "ロード中は約 1 秒（7 個 × 0.15 秒）");

  /* 「ロード中」のあいだに押したスイッチ（6400 ms）で決定していなければ、7600 ms のスイッチで決定して走る。
   * もし 6400 ms で決定していたら、7600 ms のスイッチはスタート待ちでの選び直しになり、走らない */
  press_switch_at(7600);
  hand_at(8600, 500);
  s_explore_result = EXPLORE_RESULT_DONE;
  mode_run_once();
  check(s_explore_count == 2, "ロード中に押したスイッチは、次の決定にならない");
}

/* 探索がゴールへ行けなかったら、外側の 4 個を速く点滅させる */
static void test_no_route(void)
{
  printf("[5] 探索がゴールへ行けなかったとき\n");
  reset_world();
  mode_init();
  s_explore_result = EXPLORE_RESULT_NO_ROUTE;

  rotate_right_at(100, 1);
  press_switch_at(500);
  hand_at(1500, 500);
  mode_run_once();

  check_leds_at(s_explore_time_ms + 40, "1100011", "外側の 4 個が点く");
  check_leds_at(s_explore_time_ms + 120, "0000000", "速く点滅する（消えたところ）");
  check(s_failsafe_clear_count == 0, "フェイルセーフの解除はしない");
}

/* ---------------------------------------------------------------------------
 * 6. ログ出力、ゴールの位置の入力、迷路呼び出し
 * ------------------------------------------------------------------------- */

/* モード7 No.2 を選んで、x = 20、y = 3 を入れる */
static void test_goal_input(void)
{
  printf("[6] ゴールの位置を入れる\n");
  reset_world();
  mode_init();   /* Flash にゴールがないので、最初は (15, 15) */

  rotate_left_at(100, -1);    /* モード7 */
  rotate_right_at(100, 2);    /* No.2 */
  press_switch_at(1000);      /* 決定 */
  hand_at(2000, 500);         /* スタート：3530 ms ごろから x の入力 */
  rotate_right_at(5000, 5);   /* x = 15 + 5 = 20 */
  press_switch_at(7000);      /* x を決める：y の入力へ */
  rotate_right_at(9000, -12); /* y = 15 - 12 = 3 */
  press_switch_at(11000);     /* y を決める：保存 */
  mode_run_once();

  check_leds_at(6000, "0010100", "x = 20 を LED3 + LED5 で出す");
  check_leds_at(10000, "0000011", "y = 3 を LED6 + LED7 で出す");
  check_leds_at(11500, "1111111", "保存できたら、全部を 1 秒点ける");
  check(goal_get_x() == 20 && goal_get_y() == 3, "ゴールが (20, 3) になる");
  check(s_flash_goal_save_count == 1 && s_flash_goal_x == 20 && s_flash_goal_y == 3, "Flash に (20, 3) を 1 回保存する");
}

/* (0, 0) を入れると、保存せずに x の入力に戻る。入れ直すと保存する */
static void test_goal_input_invalid(void)
{
  printf("[6] ゴールの位置に (0, 0) を入れたとき\n");
  reset_world();
  mode_init();

  rotate_left_at(100, -1);
  rotate_right_at(100, 2);
  press_switch_at(2000);
  hand_at(4000, 500);          /* x の入力は 5830 ms ごろから */
  rotate_right_at(8000, -15);  /* x = 0 */
  press_switch_at(10000);
  rotate_right_at(12000, -15); /* y = 0 */
  press_switch_at(14000);      /* (0, 0)：保存せず、x の入力に戻る */
  rotate_right_at(18000, 1);   /* x = 15 + 1 = 16（最初の値は、今のゴールの 15） */
  press_switch_at(20000);
  press_switch_at(24000);      /* y = 15 のまま */
  mode_run_once();

  check_leds_at(14050, "1100011", "(0, 0) では、外側の 4 個を速く点滅させる");
  check(s_flash_goal_save_count == 1, "保存は 1 回だけ（入れ直したとき）");
  check(goal_get_x() == 16 && goal_get_y() == 15, "ゴールが (16, 15) になる");
}

/* ログ出力：モード7 No.1 を選んでスタートすると、今のゴールを渡して出力を呼ぶ */
static void test_output(void)
{
  printf("[6] ログ出力\n");
  reset_world();
  s_flash_has_goal = true;
  s_flash_goal_x = 8;
  s_flash_goal_y = 9;
  mode_init();
  rotate_left_at(100, -1);   /* モード7 */
  rotate_right_at(100, 1);   /* No.1 */
  press_switch_at(500);
  hand_at(1500, 500);
  mode_run_once();
  check(s_output_count == 1, "出力を 1 回呼ぶ");
  check(s_output_goal_x == 8 && s_output_goal_y == 9, "今のゴールを渡す（ビューアがゴールを描くのに使う）");
  check(s_explore_count == 0, "走らない");
}

/* 迷路呼び出し（モード7 No.15）：保存した迷路があれば全部を 1 秒点け、なければ外側の 4 個を速く点滅させる */
static void test_load_maze(void)
{
  printf("[6] 迷路呼び出し\n");
  reset_world();
  mode_init();
  s_flash_has_maze = true;
  rotate_left_at(100, -1);    /* モード7 */
  rotate_right_at(100, -1);   /* No.15 */
  press_switch_at(500);
  hand_at(1500, 500);
  mode_run_once();
  check(s_flash_maze_load_count == 1, "迷路呼び出しで、Flash の迷路を 1 回読む");
  check_leds_at(s_now_ms - 500, "1111111", "呼び出せたら、全部を点ける");

  reset_world();
  mode_init();
  s_flash_has_maze = false;
  rotate_left_at(100, -1);    /* モード7 */
  rotate_right_at(100, -1);   /* No.15 */
  press_switch_at(500);
  hand_at(1500, 500);
  mode_run_once();
  check(s_flash_maze_load_count == 1, "保存した迷路がなくても、Flash を読みに行く");
  check_leds_at(s_now_ms - 160 + 40, "1100011", "保存した迷路がなければ、外側の 4 個を速く点滅させる");
}

/* ---------------------------------------------------------------------------
 * main
 * ------------------------------------------------------------------------- */

int main(void)
{
  test_led_display();
  test_inputs();
  test_goal();
  test_select_and_start();
  test_number_zero();
  test_wrap();
  test_reselect();
  test_failsafe();
  test_no_route();
  test_output();
  test_goal_input();
  test_goal_input_invalid();
  test_load_maze();

  if (s_fail_count == 0) {
    printf("すべて成功\n");
    return 0;
  }
  printf("失敗: %d 件\n", s_fail_count);
  return 1;
}
