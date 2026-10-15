/*
 * mode.c
 * モードを選んで、走らせる（MODES.md の 2・4）
 *
 * ■ 流れ（mode_run_once() の 1 回ぶん）
 *   1. 左タイヤでモード、右タイヤで No. を選ぶ。スイッチで決定する
 *   2. スタート待ち（前センサに手をかざして離すと、1 秒後に始まる。スイッチで 1 に戻る）
 *   3. 選んだモードを走らせる。終わったら、同じモード・No. のまま 1 に戻る
 *
 * ■ 今あるモード（ほかのモードは、中身を作ったときにここに足す）
 *   モード0 No.1〜3  探索（往復・片道・未知壁探索）
 *   モード7 No.1     ログ出力（迷路と走行ログを PC へ出す）
 *   モード7 No.2     ゴールの位置を入れる
 *   モード7 No.15    迷路呼び出し（Flash に保存した迷路を RAM に戻す）
 * No.0 と、まだ何もない番号では、スイッチを押しても決定しない（何も起きない）。
 */
#include "sys/mode.h"

#include "maze/maze_map.h"
#include "sys/explore.h"
#include "sys/failsafe.h"
#include "sys/flash.h"
#include "sys/goal.h"
#include "sys/output.h"
#include "sys/ui.h"

/* 選んでいるモードと No.（走り終わっても、そのまま残す） */
static int s_mode = 0;
static int s_number = 0;

/* 起動時に 1 回呼ぶ。迷路は空にし、ゴールは Flash から読む。モード0 の No.0 から始める */
void mode_init(void)
{
  ui_init();
  maze_map_init();
  goal_init();
  s_mode = 0;
  s_number = 0;
}

/* 今選んでいるモード */
int mode_get_mode(void)
{
  return s_mode;
}

/* 今選んでいる No. */
int mode_get_number(void)
{
  return s_number;
}

/* モード mode の No. number に、することがあるか（決定できるか） */
bool mode_has_action(int mode, int number)
{
  if (mode == 0) {
    if (number >= 1 && number <= 3) {
      return true;  /* 探索 */
    }
    return false;
  }
  if (mode == 7) {
    if (number == 1 || number == 2) {
      return true;  /* ログ出力、ゴールの位置 */
    }
    if (number == 15) {
      return true;  /* 迷路呼び出し */
    }
    return false;
  }
  return false;  /* ほかのモードは、まだ作っていない */
}

/* value に steps を足して、0〜(count - 1) の中で一周させる（count - 1 の次は 0、0 の前は count - 1） */
static int add_and_wrap(int value, int steps, int count)
{
  value = value + steps;
  while (value >= count) {
    value = value - count;
  }
  while (value < 0) {
    value = value + count;
  }
  return value;
}

/* ---------------------------------------------------------------------------
 * モードを選ぶ
 * ------------------------------------------------------------------------- */

/* 左タイヤでモード、右タイヤで No. を選び、スイッチで決定するまで待つ。選んでいるものは LED に出す。
 * することのない番号（No.0 など）では、スイッチを押しても決定しない */
static void select_mode(void)
{
  ui_forget_tire_rotation();
  while (true) {
    s_mode = add_and_wrap(s_mode, ui_read_left_steps(), MODE_COUNT);
    s_number = add_and_wrap(s_number, ui_read_right_steps(), NUMBER_COUNT);
    ui_show_mode_number(s_mode, s_number);

    if (ui_switch_pushed() && mode_has_action(s_mode, s_number)) {
      ui_blink_all(1);  /* 決定の合図 */
      return;
    }
    ui_wait_ms(UI_POLL_MS);
  }
}

/* ---------------------------------------------------------------------------
 * モード0：探索
 * ------------------------------------------------------------------------- */

/* 探索走行をする（No.1 往復、No.2 片道、No.3 未知壁探索） */
static void run_explore(int number)
{
  explore_type_t type = EXPLORE_ROUND_TRIP;
  if (number == 2) {
    type = EXPLORE_ONE_WAY;
  } else if (number == 3) {
    type = EXPLORE_UNKNOWN_WALLS;
  }

  explore_result_t result = explore_run(type, goal_get_x(), goal_get_y(), GOAL_WIDTH, GOAL_HEIGHT);

  if (result == EXPLORE_RESULT_FAILSAFE) {
    /* 7 個全部を点滅させて、スイッチが押されるまで待ち（押したら「ロード中」を出す）、フェイルセーフを解除する */
    ui_wait_failsafe_reset();
    failsafe_clear();
  } else if (result == EXPLORE_RESULT_NO_ROUTE) {
    ui_signal_error();  /* ゴールへ行けなかった */
  }
}

/* ---------------------------------------------------------------------------
 * モード7：設定
 * ------------------------------------------------------------------------- */

/* ゴールの x か y を 1 つ入れる（0〜value_max）。右タイヤで 1 ずつ変え、スイッチで決める。
 * 値は LED3〜LED7 に 2 進数で出す。value は最初の値 */
static int input_goal_value(int value, int value_max)
{
  ui_forget_tire_rotation();
  while (true) {
    value = add_and_wrap(value, ui_read_right_steps(), value_max + 1);
    ui_show_value(value);

    if (ui_switch_pushed()) {
      return value;
    }
    ui_wait_ms(UI_POLL_MS);
  }
}

/* ゴールの位置（左下の区画の x・y）を入れて、Flash に保存する（MODES.md のモード7 No.2）。
 * 使えない位置（スタートの区画を含む）なら、保存せずに x の入力に戻る */
static void input_goal(void)
{
  while (true) {
    ui_blink_all(1);  /* x の入力 */
    int goal_x = input_goal_value(goal_get_x(), GOAL_X_MAX);
    ui_blink_all(2);  /* y の入力 */
    int goal_y = input_goal_value(goal_get_y(), GOAL_Y_MAX);

    if (goal_set(goal_x, goal_y)) {
      ui_signal_ok();
      return;
    }
    ui_signal_error();
  }
}

/* Flash に保存した迷路を RAM に戻す（MODES.md のモード7 No.15 迷路呼び出し） */
static void load_maze(void)
{
  if (flash_load_maze()) {
    ui_signal_ok();
  } else {
    ui_signal_error();  /* 保存した迷路がない */
  }
}

/* 出力と設定（No.1 ログ出力、No.2 ゴールの位置、No.15 迷路呼び出し） */
static void run_setting(int number)
{
  if (number == 1) {
    output_all(goal_get_x(), goal_get_y());
  } else if (number == 2) {
    input_goal();
  } else if (number == 15) {
    load_maze();
  }
}

/* ---------------------------------------------------------------------------
 * 1 回ぶん
 * ------------------------------------------------------------------------- */

/* モードを選んで、スタートして、走らせる。走り終わったら戻る（起動後、繰り返し呼ぶ）。
 * スタート待ちでスイッチが押されたときは、何もせずに戻る（次に呼ばれたときに選び直す） */
void mode_run_once(void)
{
  select_mode();
  if (!ui_wait_start(s_mode, s_number)) {
    return;
  }

  if (s_mode == 0) {
    run_explore(s_number);
  } else if (s_mode == 7) {
    run_setting(s_number);
  }
}
