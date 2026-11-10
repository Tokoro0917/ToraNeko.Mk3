/*
 * mode.c
 * モードを選んで、走らせる（MODES.md の 2・4）
 *
 * ■ 流れ（mode_run_once() の 1 回ぶん）
 *   1. 左タイヤでモード、右タイヤで No. を選ぶ。スイッチで決定する
 *   2. スタート待ち（前センサに手をかざして離すと、1 秒後に始まる。スイッチで 1 に戻る）。
 *      センサ値の表示（モード5 No.1）と壁までの距離の表示（No.3）は、前センサの値を見たいので、スタート待ちをせずにすぐ始める
 *   3. 選んだモードを走らせる。終わったら、同じモード・No. のまま 1 に戻る
 *      （最後まで走れたら「うまくいった」、フェイルセーフならスイッチで解除するまで待ち、道がなくなったら「できなかった」を出す）
 *   1 に戻るときは、どの道からでも（スタート待ちで取り消したとき、センサ値の表示を終えたとき、走り終わったとき、
 *   フェイルセーフを解除したとき）「ロード中」（約 1 秒）を出す。このあいだはスイッチを押しても何も起きないので、
 *   続けて押したスイッチが、次の決定にならない（2026-10-22 に決定）
 *
 * ■ 今あるモード（ほかのモードは、中身を作ったときにここに足す。2026-11-10 に、番号の穴をなくし、
 *   似たものを同じモードに集めるように並べ直した。作者が決めた）
 *   モード0 No.1〜4  探索（往復・片道・未知壁探索・未知壁探索＋既知区間加速）
 *   モード1 No.1〜6  最短走行（No.1〜3 は曲がるところが探索と同じ小回りで、直進の速さが No.1 → 3 で速くなる。
 *                    No.4〜6 は大回り・斜めのターンで曲がる）。
 *                    決定したときに道がなければ、「できなかった」を出して選び直す
 *   モード2 No.1     スラローム 90°（探索の速さで左に曲がり、左の区画のまん中で止まる）
 *   モード2 No.2     U ターン（行き止まりで止まり、前壁合わせ、180°、戻って止まる）
 *   モード2 No.3     その場 180° 旋回（左に回って止まる。旋回の制御の係数を合わせる）
 *   モード2 No.4     壁合わせだけ（前の壁で位置と向きを直す）
 *   モード3 No.1〜7  最短走行のターンを 1 つずつ（大回り 90°・180°、斜め入り 45°・135°、斜め出 45°・135°、V90°。左だけ）
 *   モード3 No.8・9  斜めの直進のテスト（No.8 は斜めの補正なし、No.9 はあり）
 *   モード4 No.1〜13 加速度テスト（No.1〜5 は吸引なし 2000〜4000mm/s²、No.6〜13 は吸引あり 4000〜21000mm/s²）
 *   モード5 No.1     センサ値の表示（スイッチを押すと終わる）
 *   モード5 No.2     センサ値の記録（区画のまん中で 1 秒測り、平均をモード7 No.1 で出す）
 *   モード5 No.3     壁までの距離の表示（センサの値と距離 [mm] を出し続ける。スイッチを押すと終わる）
 *   モード5 No.4〜6  壁切れ・柱の見え方テスト
 *   モード6 No.1     直進（ゆっくり 280mm 走って止まる。タイヤの直径と、制御の係数を合わせる）
 *   モード6 No.2     壁に沿って直進（横の壁に沿って走る補正を入れて、4 区画走って止まる）
 *   モード6 No.3     吸引 ON（吸引ファンを回し、スイッチを押すまで吸い続ける）
 *   モード6 No.4     吸引の力（吸引の強さを段ごとに上げて回す。はかりで測る）
 *   モード6 No.5     エンコーダの歪みを測る（タイヤを浮かせて、左右のモーターを回す）
 *   モード7 No.1     ログ出力（迷路と走行ログを PC へ出す）
 *   モード7 No.2     ゴールの位置を入れる
 *   モード7 No.3     倍率合わせ（区画の境目で測って、感度の倍率を Flash に保存する。置いた位置がずれないように、
 *                    スタート待ちをせず、決定してから 2 秒後に測る）
 *   モード7 No.4     壁制御の目標（区画の中心で測って、横の壁に沿う補正の目標を Flash に保存する。No.3 と同じく 2 秒後に測る）
 *   モード7 No.5     倍率と壁制御の目標を元に戻す（倍率 1、目標は家の値にして保存する。決定するとすぐ）
 *   モード7 No.6     迷路呼び出し（Flash に保存した迷路を RAM に戻す）
 * Flash に書く・読む操作（ゴール、倍率、目標、迷路呼び出し）は、モード7 にまとめる（2026-11-10。作者が決めた）。
 * No.0 と、まだ何もない番号では、スイッチを押しても決定しない（何も起きない）。
 */
#include "sys/mode.h"

#include "maze/maze_map.h"
#include "sys/adjust.h"
#include "sys/encoder_cal.h"
#include "sys/explore.h"
#include "sys/failsafe.h"
#include "sys/flash.h"
#include "sys/goal.h"
#include "sys/output.h"
#include "sys/sensor_record.h"
#include "sys/venue.h"
#include "sys/shortest.h"
#include "sys/turn_test.h"
#include "sys/edge_test.h"
#include "sys/sensor_view.h"
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
  venue_init();  /* Flash に保存した会場ごとの感度の倍率と、壁制御の目標を読む */
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
    if (number >= 1 && number <= 4) {
      return true;  /* 探索 */
    }
    return false;
  }
  if (mode == 1) {
    if (number >= 1 && number <= 6) {
      return true;  /* 最短走行 */
    }
    return false;
  }
  if (mode == 3) {
    if (number >= 1 && number <= 9) {
      return true;  /* 最短走行のターン（No.1〜7）、斜めの直進のテスト（No.8・9） */
    }
    return false;
  }
  if (mode == 2) {
    if (number >= 1 && number <= 4) {
      return true;  /* スラローム 90°、U ターン、その場 180° 旋回、壁合わせだけ */
    }
    return false;
  }
  if (mode == 4) {
    if (number >= 1 && number <= 13) {
      return true;  /* 加速度テスト（No.1〜5 は吸引なし、No.6〜13 は吸引あり） */
    }
    return false;
  }
  if (mode == 5) {
    if (number >= 1 && number <= 6) {
      return true;  /* センサ値の表示・記録、壁までの距離の表示、壁切れ・柱の見え方テスト */
    }
    return false;
  }
  if (mode == 6) {
    if (number >= 1 && number <= 5) {
      return true;  /* 直進、壁に沿って直進、吸引 ON、吸引の力、エンコーダの歪み */
    }
    return false;
  }
  if (mode == 7) {
    if (number >= 1 && number <= 6) {
      return true;  /* ログ出力、ゴールの位置、倍率合わせ、壁制御の目標、元に戻す、迷路呼び出し */
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

    /* ui_switch_pushed() は、押されたかを毎回読む必要がある（チャタリングを取り除くため）ので、&& の左に置く。
     * && は左が false なら右を調べないので、左右を入れ替えると、することのない番号のあいだスイッチを読まなくなる */
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

/* 探索走行をする（No.1 往復、No.2 片道、No.3 未知壁探索、No.4 未知壁探索＋既知区間加速） */
static void run_explore(int number)
{
  explore_type_t type = EXPLORE_ROUND_TRIP;
  if (number == 2) {
    type = EXPLORE_ONE_WAY;
  } else if (number == 3) {
    type = EXPLORE_UNKNOWN_WALLS;
  } else if (number == 4) {
    type = EXPLORE_UNKNOWN_WALLS_KNOWN;
  }

  explore_result_t result = explore_run(type, goal_get_x(), goal_get_y(), GOAL_WIDTH, GOAL_HEIGHT);

  if (result == EXPLORE_RESULT_DONE) {
    ui_signal_ok();  /* 走り終わった（迷路の保存は explore_run() の中で済んでいる） */
  } else if (result == EXPLORE_RESULT_FAILSAFE) {
    /* 7 個全部を点滅させて、スイッチが押されるまで待ち、フェイルセーフを解除する（「ロード中」は mode_run_once() が出す） */
    ui_wait_failsafe_reset();
    failsafe_clear();
  } else if (result == EXPLORE_RESULT_NO_ROUTE) {
    ui_signal_error();  /* 道がなくなった（行き・帰りのどちらか）か、動きすぎた。迷路は保存しない */
  }
}

/* ---------------------------------------------------------------------------
 * モード1：最短走行
 * ------------------------------------------------------------------------- */

/* 最短走行をする（No. で直進の速さを選ぶ） */
static void run_shortest(int number)
{
  shortest_result_t result = shortest_run(number, goal_get_x(), goal_get_y());

  if (result == SHORTEST_RESULT_DONE) {
    ui_signal_ok();  /* ゴールで止まった。スタートへは手で戻す */
  } else if (result == SHORTEST_RESULT_FAILSAFE) {
    /* 7 個全部を点滅させて、スイッチが押されるまで待ち、フェイルセーフを解除する */
    ui_wait_failsafe_reset();
    failsafe_clear();
  } else {
    ui_signal_error();  /* 道がない（決定したときに調べているので、ふつうは起きない） */
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

/* Flash に保存した迷路を RAM に戻す（MODES.md のモード7 No.6 迷路呼び出し） */
static void load_maze(void)
{
  if (flash_load_maze()) {
    ui_signal_ok();
  } else {
    ui_signal_error();  /* 保存した迷路がない */
  }
}

/* 出力と設定（No.1 ログ出力、No.2 ゴールの位置、No.6 迷路呼び出し。No.3〜5 はスタート待ちをしないので mode_run_once() で呼ぶ） */
static void run_setting(int number)
{
  if (number == 1) {
    adjust_print_last_result();  /* 最後に走らせた調整の動きの結果（走ったときはケーブルを挿していないので、ここで出す） */
    sensor_record_print();       /* 最後に測ったセンサ値の記録（モード5 No.2） */
    venue_print();               /* 今の会場ごとの感度の倍率と、壁制御の目標（モード7 No.3・4） */
    output_all(goal_get_x(), goal_get_y());
    ui_signal_ok();  /* 出し終わった */
  } else if (number == 2) {
    input_goal();
  } else if (number == 6) {
    load_maze();
  }
}

/* ---------------------------------------------------------------------------
 * 1 回ぶん
 * ------------------------------------------------------------------------- */

/* 選んだモードを走らせる（スタートしたあと） */
static void run_selected(void)
{
  if (s_mode == 0) {
    run_explore(s_number);
  } else if (s_mode == 1) {
    run_shortest(s_number);
  } else if (s_mode == 2 && s_number == 1) {
    adjust_slalom_90();
  } else if (s_mode == 2 && s_number == 2) {
    adjust_uturn();
  } else if (s_mode == 2 && s_number == 3) {
    adjust_pivot_180();
  } else if (s_mode == 2 && s_number == 4) {
    adjust_front_align();
  } else if (s_mode == 3) {
    turn_test_run(s_number);
  } else if (s_mode == 4) {
    adjust_accel_test(s_number);
  } else if (s_mode == 5 && s_number == 2) {
    sensor_record_run();
  } else if (s_mode == 5 && s_number >= 4 && s_number <= 6) {
    edge_test_run(s_number);
  } else if (s_mode == 6 && s_number == 1) {
    adjust_straight();
  } else if (s_mode == 6 && s_number == 2) {
    adjust_straight_along_wall();
  } else if (s_mode == 6 && s_number == 3) {
    adjust_suction_on();
  } else if (s_mode == 6 && s_number == 4) {
    adjust_suction_force();
  } else if (s_mode == 6 && s_number == 5) {
    encoder_cal_run();
  } else if (s_mode == 7) {
    run_setting(s_number);
  }
}

/* モードを選んで、スタートして、走らせる。走り終わったら戻る（起動後、繰り返し呼ぶ）。
 * スタート待ちでスイッチが押されたときは、走らずに戻る（次に呼ばれたときに選び直す）。
 * どの道から戻るときも、最後に「ロード中」を出す */
void mode_run_once(void)
{
  select_mode();

  if (s_mode == 1 && !shortest_has_route(s_number, goal_get_x(), goal_get_y())) {
    ui_signal_error();  /* 最短走行で、スタートからゴールへの道がない。走らずに選び直す */
  } else if (s_mode == 5 && s_number == 1) {
    sensor_view_run();  /* スタート待ちをしない */
  } else if (s_mode == 5 && s_number == 3) {
    sensor_view_distance_run();  /* スタート待ちをしない（前センサの値を見たいので） */
  } else if (s_mode == 7 && s_number == 3) {
    venue_measure_scale_run();  /* スタート待ちをしない（手をかざすと、置いた位置がずれやすい） */
  } else if (s_mode == 7 && s_number == 4) {
    venue_measure_reference_run();  /* 同じ */
  } else if (s_mode == 7 && s_number == 5) {
    venue_reset_run();  /* スタート待ちをしない（動かないので） */
  } else if (ui_wait_start(s_mode, s_number)) {
    run_selected();
  }

  ui_show_loading();
}
