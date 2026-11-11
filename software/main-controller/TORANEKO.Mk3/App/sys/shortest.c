/*
 * shortest.c
 * 最短走行（モード1。MODES.md）
 *
 * RAM の迷路で、スタートからゴールまでの最短経路を作り、その経路のとおりに走って、ゴールの区画で止まる。
 * まだ見ていない壁は「ある」とみなす。No. で走り方と速さを選ぶ（config/run_params.h の SHORTEST1〜5）。
 *
 * ■ No.1〜3：曲がるところは、探索と同じ小回りだけ
 *   1. 経路を作る（maze/path.c の path_make_shortest_small_turn()）
 *   2. 直進は最高速まで速くしてから、小回りの速さ（300mm/s）まで遅くする。左・右は探索と同じスラローム 90°
 *   3. 最後の直進で、ゴールの最初の区画のまん中で止まる
 *
 * ■ No.4〜7：大回り・斜めのターンで曲がる（2026-10-28 に作った。2026-11-04 に、ターンごとの速さで曲がるようにした）
 *   No.4・5 はターンの組 1（ゆるいターン 800mm/s、きついターン 600mm/s）、No.6 は組 2（7 つとも 1000mm/s。2026-11-10）、
 *   No.7 は組 3（ゆるいターン 1200mm/s・きついターン 1000mm/s。2026-11-11）で
 *   曲がる（config/turn_params.h の「■ ターンの組」）。走り終わったら組 1 に戻す
 *   1. 吸引を上げて（0.5 秒）、0.5 秒待つ（ターンは吸引して合わせた。モード3。吸引なしでは 800mm/s のターンで滑る）
 *   2. 経路を作る（maze/path.c の path_make_shortest()。大回り 90°・180°、斜め入り・斜め出 45°・135°、V90°、斜めの直進）
 *   3. ターンは、そのターンの速さ（config/turn_params.h の *_SPEED_MM_S。ゆるいターンは 800mm/s、きついターンは 600mm/s）で
 *      曲がる。直進・斜めの直進は、最高速まで速くしてから、次のターンの速さまで遅くする。
 *      ただし、次のことがあるときは、それより遅い速さで直進を終わり、次のターンをその速さで曲がる
 *      （同じ形のまま遅く曲がる。ctrl/run.c の fast_turn()）
 *        - 短くて上げきれないとき（スタートの直後など）
 *        - ターンが直進をはさまずに続くとき（ターンのあいだは速さを変えられないので、続くターンの遅いほうに合わせる）
 *        - その先の短い直進で、次の次のターンの速さまで遅くしきれないとき
 *      どこまで遅くしておくかは、経路の終わりまで先を見て決める（speed_limit_ahead_mm_s()）
 *   4. ゴールの区画に入ってから、もう 1 区画進んで、その区画のまん中で止まる（下の「■ ゴールで止まるところ」）
 *      もう 1 区画進めないとき（ふつうは起きない）は、最後が直進ならゴールの最初の区画のまん中で、最後がターンなら
 *      区画のまん中に着いたところから SHORTEST_STOP_ACCEL_MM_S2 で、SHORTEST_STOP_DISTANCE_MM 以内で止まる
 *   5. 吸引を止める
 *   6. 止まった区画の壁を見て、入ってきた向きを向き、位置を覚える（下の「■ ゴールで止まったあと」）
 *   小回り（MOTION_TURN_LEFT・RIGHT）は経路に残らない（PC で 500 個の迷路を調べて 0 個）が、残ったら走らずに「道がない」にする
 *
 * 経路は、その走り方と速さで本当にかかる時間が短いものを選ぶ（ダイクストラの走行時間のコスト。maze/dijkstra.c）。
 * No.4〜7 は「直線 ＋ ターン 1 つ」ごとの時間で数える（KOGUMA の #100）ので、ターンの種類ごとの時間の違いまで比べる。
 * 時間を数えるときは、どのターンも FAST_TURN_SPEED_MM_S（500mm/s）で曲がるとして数える（本当はターンごとに速さが違うが、
 * 直線の時間が前後のターンの速さで変わり、ダイクストラで数えにくいため。ターンの形はどれも同じなので、選ぶ道はあまり変わらない）。
 * No.6 は、ターンが 7 つとも 1000mm/s なので、T1000_PATH_TURN_SPEED_MM_S（1000mm/s）で数える。
 * No.7 は、いちばん遅いターン（きついターン）の 1000mm/s で数える（T1200_PATH_TURN_SPEED_MM_S）。
 *
 * ■ ゴールで止まるところ（2026-11-10。作者の案。No.1〜7 とも）
 * 経路は、ゴールの最初の区画のまん中で終わる。ゴールは 2×2 なので、そこから同じ向きにもう 1 区画進んでも、まだゴールの中にいる。
 * そこで、経路の最後に「1 区画（90mm）の直進」を足して（最後が直進ならその直進を 1 区画長くして）、その区画のまん中で止まる
 * （extend_into_goal()）。こうすると、ゴールに直進で入っても、ターンで入っても、止まるところは同じ「ゴールの 2 つ目の区画の
 * まん中」になり、止まるまでに 1 区画ぶんの余裕ができる（ターンで入ったときに、ターンの速さを落とさなくてよい）。
 * 次の区画がゴールの外のとき・あいだに壁があるときは、足さない（今までどおり、最初の区画で止まる）
 *
 * ■ ゴールで止まったあと（2026-11-11。作者の案。No.1〜7 とも）
 * ゴールの区画のまん中で止まったら、次の順に動く（turn_back_at_goal()）。
 *   1. 止まった区画の前・左・右の壁を、センサで見る（探索で、この区画の壁を使うため）
 *   2. 前に壁があれば前壁合わせをしながら、その場で左に 90° を 2 回回って、入ってきた向きを向く
 *      （探索でスタートに戻ったときと同じ動き。ctrl/run.c の run_search_turn_back_at_center()）
 *   3. 止まった区画・入ってきた向き・見た壁を覚える（shortest_get_end_pose()。電源を切ると消える。今は覚えるだけで、
 *      使うところはない。2026-11-11 にゴールから始める探索〈モード0 No.5・6〉を作ったが、使わないことにして消した）
 * 最後がターンで止まったとき（ゴールの次の区画へ進めず、最後のターンのあとで止まったとき。ふつうは起きない）は、
 * 止まった位置が区画のまん中からずれているかもしれないので、何もしない（覚えない）
 *
 * ■ スタートの位置
 * 経路は、スタート区画 (0,0) の南の辺から始まる。機体は、お尻を後ろの壁に付けて置くので、
 * タイヤの軸は南の辺より HALF_CELL_MM − START_OFFSET_MM（35mm）前にある。最初の直進は、その分だけ短くする。
 */
#include "sys/shortest.h"

#include <math.h>
#include <stdbool.h>

#include "config/run_params.h"
#include "config/turn_params.h"
#include "ctrl/run.h"
#include "ctrl/wall_ctrl.h"
#include "drv/fan.h"
#include "maze/dijkstra.h"
#include "maze/maze_map.h"
#include "maze/path.h"
#include "sys/failsafe.h"
#include "sys/goal.h"
#include "sys/suction.h"
#include "sys/ui.h"

#define SHORTEST_SUCTION_WAIT_MS 500      /* 吸引を上げてから、走り出す前に待つ時間 [ms]（ファンの回り方を落ち着かせる） */
#define SPEED_NO_LIMIT_MM_S      100000.0f /* 「速さの上限なし」を表す、どの速さより大きい値 [mm/s] */

static path_t s_path;  /* 最短経路（大きいので、関数の中ではなくここに置く） */

/* ゴールで止まったときに覚えた位置（先頭のコメントの「■ ゴールで止まったあと」） */
static shortest_end_pose_t s_end_pose = { false, 0, 0, DIR_NORTH, false, false, false };

/* 大回り・斜めで走る No. か（No.4〜7） */
static bool uses_diagonal(int number)
{
  if (number >= 4) {
    return true;
  }
  return false;
}

/* No. ごとの、直進の最高速 [mm/s] */
static float max_speed_mm_s(int number)
{
  if (number == 2) {
    return SHORTEST2_MAX_SPEED_MM_S;
  }
  if (number == 3) {
    return SHORTEST3_MAX_SPEED_MM_S;
  }
  if (number == 4) {
    return SHORTEST4_MAX_SPEED_MM_S;
  }
  if (number == 5) {
    return SHORTEST5_MAX_SPEED_MM_S;
  }
  if (number == 6) {
    return SHORTEST6_MAX_SPEED_MM_S;
  }
  if (number == 7) {
    return SHORTEST7_MAX_SPEED_MM_S;
  }
  return SHORTEST1_MAX_SPEED_MM_S;
}

/* No. ごとの、直進の加速度 [mm/s²] */
static float accel_mm_s2(int number)
{
  if (number == 2) {
    return SHORTEST2_ACCEL_MM_S2;
  }
  if (number == 3) {
    return SHORTEST3_ACCEL_MM_S2;
  }
  if (number == 4) {
    return SHORTEST4_ACCEL_MM_S2;
  }
  if (number == 5) {
    return SHORTEST5_ACCEL_MM_S2;
  }
  if (number == 6) {
    return SHORTEST6_ACCEL_MM_S2;
  }
  if (number == 7) {
    return SHORTEST7_ACCEL_MM_S2;
  }
  return SHORTEST1_ACCEL_MM_S2;
}

/* No. ごとの、斜めの直進の最高速 [mm/s]（No.4〜7 だけ使う） */
static float diag_max_speed_mm_s(int number)
{
  if (number == 5) {
    return SHORTEST5_DIAG_MAX_SPEED_MM_S;
  }
  if (number == 6) {
    return SHORTEST6_DIAG_MAX_SPEED_MM_S;
  }
  if (number == 7) {
    return SHORTEST7_DIAG_MAX_SPEED_MM_S;
  }
  return SHORTEST4_DIAG_MAX_SPEED_MM_S;
}

/* No. ごとの、斜めの直進の加速度 [mm/s²]（No.4〜7 だけ使う） */
static float diag_accel_mm_s2(int number)
{
  if (number == 5) {
    return SHORTEST5_DIAG_ACCEL_MM_S2;
  }
  if (number == 6) {
    return SHORTEST6_DIAG_ACCEL_MM_S2;
  }
  if (number == 7) {
    return SHORTEST7_DIAG_ACCEL_MM_S2;
  }
  return SHORTEST4_DIAG_ACCEL_MM_S2;
}

/* No. ごとの、吸引の電圧 [V]（No.4〜7 だけ使う。config/run_params.h の SHORTEST*_SUCTION_V） */
static float suction_voltage_v(int number)
{
  if (number == 5) {
    return SHORTEST5_SUCTION_V;
  }
  if (number == 6) {
    return SHORTEST6_SUCTION_V;
  }
  if (number == 7) {
    return SHORTEST7_SUCTION_V;
  }
  return SHORTEST4_SUCTION_V;
}

/* No. ごとの、ターンの値の組（No.6 は組 2〈7 つとも 1000mm/s〉、No.7 は組 3〈ゆるいターン 1200mm/s・きついターン 1000mm/s〉、
 * ほかは組 1。ctrl/run.c の run_fast_set_turn_set()） */
static int turn_set_of(int number)
{
  if (number == 6) {
    return 2;
  }
  if (number == 7) {
    return 3;
  }
  return 1;
}

/* No. ごとの、経路を選ぶときに、どのターンも曲がるとする速さ [mm/s]（config/turn_params.h） */
static float path_turn_speed_mm_s(int number)
{
  if (number == 6) {
    return T1000_PATH_TURN_SPEED_MM_S;
  }
  if (number == 7) {
    return T1200_PATH_TURN_SPEED_MM_S;
  }
  return FAST_TURN_SPEED_MM_S;
}

/* 経路の中に、小回り（探索用の左右のターン）があるか（No.4〜7 では走れない） */
static bool has_small_turn(void)
{
  for (int i = 0; i < s_path.count; i++) {
    if (s_path.steps[i].motion == MOTION_TURN_LEFT || s_path.steps[i].motion == MOTION_TURN_RIGHT) {
      return true;
    }
  }
  return false;
}

/* 区画 (x, y) がゴール（左下 (goal_x, goal_y) から GOAL_WIDTH × GOAL_HEIGHT）の中か */
static bool is_in_goal(int x, int y, int goal_x, int goal_y)
{
  if (x < goal_x || x >= goal_x + GOAL_WIDTH) {
    return false;
  }
  if (y < goal_y || y >= goal_y + GOAL_HEIGHT) {
    return false;
  }
  return true;
}

/* 経路の終わり（ゴールの最初の区画のまん中）から、同じ向きにもう 1 区画進めるなら、経路の最後に 1 区画の直進を足す
 * （最後が直進なら、その直進を 1 区画長くする）。先頭のコメントの「■ ゴールで止まるところ」 */
static void extend_into_goal(int goal_x, int goal_y)
{
  int next_x = maze_next_x(s_path.end_x, s_path.end_dir);
  int next_y = maze_next_y(s_path.end_y, s_path.end_dir);
  if (!is_in_goal(next_x, next_y, goal_x, goal_y)) {
    return;  /* 次の区画はゴールの外 */
  }
  if (maze_map_get_wall(s_path.end_x, s_path.end_y, s_path.end_dir) == WALL_EXIST) {
    return;  /* あいだに壁がある（ゴールの中には壁がないはずなので、ふつうは起きない） */
  }
  int last = s_path.count - 1;
  if (s_path.steps[last].motion == MOTION_STRAIGHT) {
    s_path.steps[last].half_count = s_path.steps[last].half_count + 2;  /* 半区画 2 つ = 1 区画 */
  } else {
    if (s_path.count >= PATH_STEP_MAX) {
      return;  /* 経路がいっぱいで足せない（ふつうは起きない） */
    }
    s_path.steps[s_path.count].motion = MOTION_STRAIGHT;
    s_path.steps[s_path.count].half_count = 2;
    s_path.count = s_path.count + 1;
  }
  s_path.end_x = next_x;
  s_path.end_y = next_y;
}

/* 最短経路を作って s_path に入れる。number は No.（直進の速さで、選ぶ経路が変わる）。走れる経路ができなければ false。
 * 最初の動きは直進でないといけない（スタート区画の中で曲がる動きは作っていない） */
static bool make_path(int number, int goal_x, int goal_y)
{
  /* ダイクストラを、この No. の速さで走る時間で比べるようにしてから経路を作り、終わったら元に戻す
   * （未知壁探索など、ほかでダイクストラを使うときに影響しないように） */
  bool found = false;
  if (uses_diagonal(number)) {
    dijkstra_set_time_cost_diagonal(path_turn_speed_mm_s(number), max_speed_mm_s(number), accel_mm_s2(number),
                                    diag_max_speed_mm_s(number), diag_accel_mm_s2(number));
    found = path_make_shortest(goal_x, goal_y, GOAL_WIDTH, GOAL_HEIGHT, &s_path);
  } else {
    dijkstra_set_time_cost_small_turn(SEARCH_SPEED_MM_S, max_speed_mm_s(number), accel_mm_s2(number));
    found = path_make_shortest_small_turn(goal_x, goal_y, GOAL_WIDTH, GOAL_HEIGHT, &s_path);
  }
  dijkstra_set_step_cost();
  if (!found) {
    return false;
  }
  if (uses_diagonal(number) && has_small_turn()) {
    return false;  /* 小回りが残った（先頭のコメント。普通は起きない） */
  }
  if (s_path.count == 0) {
    return false;
  }
  if (s_path.steps[0].motion != MOTION_STRAIGHT) {
    return false;
  }
  extend_into_goal(goal_x, goal_y);
  return true;
}

/* スタートからゴールへの道があるか（モードを決めたときに調べる。number は No.） */
bool shortest_has_route(int number, int goal_x, int goal_y)
{
  return make_path(number, goal_x, goal_y);
}

/* 経路の i 番目の直進の長さ [mm]。最初の直進は、スタートの置き方の分だけ短くする（先頭のコメント） */
static float straight_distance_mm(int i)
{
  float distance_mm = s_path.steps[i].half_count * HALF_CELL_MM;
  if (i == 0) {
    distance_mm = distance_mm - (HALF_CELL_MM - START_OFFSET_MM);
  }
  return distance_mm;
}

/* 経路の i 番目の直進を始める位置が、うしろの区画の境目から何 mm 先か（壁切れに使う。ctrl/run.c の run_fast_straight）。
 *   最初の直進：スタート区画の南の辺から、タイヤの軸まで（先頭のコメントの「■ スタートの位置」。35mm）
 *   小回りのあと（No.1〜3）：小回りは区画の入口から次の区画の入口まで曲がるので、境目の上（0mm）
 *   大回り・斜め出のあと（No.4〜7）：区画のまん中（45mm） */
static float straight_start_in_cell_mm(int i)
{
  if (i == 0) {
    return HALF_CELL_MM - START_OFFSET_MM;
  }
  motion_t before = s_path.steps[i - 1].motion;
  if (before == MOTION_TURN_LEFT || before == MOTION_TURN_RIGHT) {
    return 0.0f;
  }
  return HALF_CELL_MM;
}

/* No.1〜3：経路の i 番目の動きを走る。最後の動き（ゴールの区画のまん中まで進む直進）では止まる */
static void run_small_turn_step(int i, float max_speed, float accel)
{
  motion_t motion = s_path.steps[i].motion;
  bool is_last = (i == s_path.count - 1);

  if (motion == MOTION_STRAIGHT && is_last) {
    run_fast_stop(straight_distance_mm(i), max_speed, accel);
  } else if (motion == MOTION_STRAIGHT) {
    /* 次の小回りの速さで終わる */
    run_fast_straight(straight_distance_mm(i), max_speed, SEARCH_SPEED_MM_S, accel, straight_start_in_cell_mm(i));
  } else if (motion == MOTION_TURN_LEFT) {
    run_fast_turn_left();
  } else if (motion == MOTION_TURN_RIGHT) {
    run_fast_turn_right();
  }
}

/* No.4〜7：動き motion が、どの大回り・斜めのターンか */
static run_fast_turn_t fast_turn_kind(motion_t motion)
{
  if (motion == MOTION_BIG_TURN_LEFT_90 || motion == MOTION_BIG_TURN_RIGHT_90) {
    return RUN_FAST_TURN_BIG90;
  }
  if (motion == MOTION_BIG_TURN_LEFT_180 || motion == MOTION_BIG_TURN_RIGHT_180) {
    return RUN_FAST_TURN_BIG180;
  }
  if (motion == MOTION_DIAG_IN45_LEFT || motion == MOTION_DIAG_IN45_RIGHT) {
    return RUN_FAST_TURN_IN45;
  }
  if (motion == MOTION_DIAG_IN135_LEFT || motion == MOTION_DIAG_IN135_RIGHT) {
    return RUN_FAST_TURN_IN135;
  }
  if (motion == MOTION_DIAG_OUT45_LEFT || motion == MOTION_DIAG_OUT45_RIGHT) {
    return RUN_FAST_TURN_OUT45;
  }
  if (motion == MOTION_DIAG_OUT135_LEFT || motion == MOTION_DIAG_OUT135_RIGHT) {
    return RUN_FAST_TURN_OUT135;
  }
  return RUN_FAST_TURN_V90;
}

/* No.4〜7：動き motion が左に曲がるターンか */
static bool is_left_turn(motion_t motion)
{
  if (motion == MOTION_BIG_TURN_LEFT_90 || motion == MOTION_BIG_TURN_LEFT_180
      || motion == MOTION_DIAG_IN45_LEFT || motion == MOTION_DIAG_IN135_LEFT
      || motion == MOTION_DIAG_OUT45_LEFT || motion == MOTION_DIAG_OUT135_LEFT
      || motion == MOTION_DIAG_V90_LEFT) {
    return true;
  }
  return false;
}

/* 2 つの速さの、遅いほう [mm/s] */
static float slower_mm_s(float a_mm_s, float b_mm_s)
{
  if (a_mm_s < b_mm_s) {
    return a_mm_s;
  }
  return b_mm_s;
}

/* 経路の i 番目の斜めの直進の長さ [mm] */
static float diag_distance_mm(int i)
{
  return s_path.steps[i].half_count * DIAG_STEP_MM;
}

/* No.4〜7：経路の first 番目の動きを始めるときに、出していてよい速さの上限 [mm/s]（先頭のコメントの 3）。
 * その先のどのターンも、そのターンの速さ以下で曲がれて、最後に止まれる速さにする。
 * 直進で遅くできる分は「速さの 2 乗」で 2 × 加速度 × 距離（速さ v から、距離 d で遅くできるのは √(v² − 2ad) まで）。
 * なので、あるターンの手前までに遅くできる分を slow_down とすると、今の速さは √(ターンの速さ² ＋ slow_down) までよい。
 * これを経路の終わりまで、ターンごとに計算して、いちばん遅いものにする */
static float speed_limit_ahead_mm_s(int first, float accel, float diag_accel)
{
  float limit_mm_s = SPEED_NO_LIMIT_MM_S;
  float slow_down_mm2_s2 = 0.0f;  /* first 番目から、今見ている動きの手前までの直進で遅くできる分（速さの 2 乗） */

  for (int i = first; i < s_path.count; i++) {
    motion_t motion = s_path.steps[i].motion;
    bool is_last = (i == s_path.count - 1);

    if (motion == MOTION_STRAIGHT && is_last) {
      /* 最後の直進：その終わり（ゴールの区画のまん中）で止まれる速さ */
      float stop_mm2_s2 = slow_down_mm2_s2 + 2.0f * accel * straight_distance_mm(i);
      return slower_mm_s(limit_mm_s, sqrtf(stop_mm2_s2));
    }
    if (motion == MOTION_STRAIGHT) {
      slow_down_mm2_s2 = slow_down_mm2_s2 + 2.0f * accel * straight_distance_mm(i);
    } else if (motion == MOTION_DIAG_STRAIGHT) {
      slow_down_mm2_s2 = slow_down_mm2_s2 + 2.0f * diag_accel * diag_distance_mm(i);
    } else {
      /* ターン：そのターンの速さで曲がれる速さ */
      float turn_mm_s = run_fast_turn_speed_mm_s(fast_turn_kind(motion));
      float turn_limit_mm_s = sqrtf(turn_mm_s * turn_mm_s + slow_down_mm2_s2);
      limit_mm_s = slower_mm_s(limit_mm_s, turn_limit_mm_s);
    }
  }

  /* 最後がターン（ゴールにターンで入る）：区画のまん中に着いてから SHORTEST_STOP_DISTANCE_MM で止まれる速さ */
  float stop_mm_s = sqrtf(2.0f * SHORTEST_STOP_ACCEL_MM_S2 * SHORTEST_STOP_DISTANCE_MM);
  float stop_limit_mm_s = sqrtf(stop_mm_s * stop_mm_s + slow_down_mm2_s2);
  return slower_mm_s(limit_mm_s, stop_limit_mm_s);
}

/* No.4〜7：直進の終わりの速さ [mm/s]。速さ start_mm_s から distance_mm で上げられる速さ、最高速 max_speed、
 * その先を見た上限 limit_mm_s の、いちばん遅いもの（上げきれない速さを頼むと、着いたときに速さが飛んでしまうので） */
static float straight_end_speed_mm_s(float start_mm_s, float distance_mm, float accel, float max_speed, float limit_mm_s)
{
  float reachable_mm_s = sqrtf(start_mm_s * start_mm_s + 2.0f * accel * distance_mm);
  float end_mm_s = slower_mm_s(reachable_mm_s, max_speed);
  end_mm_s = slower_mm_s(end_mm_s, limit_mm_s);
  return end_mm_s;
}

/* No.4〜7：経路を前から順に走る。speed_mm_s は今の速さ（直進が終わった速さで、次のターンを曲がる） */
static void run_diagonal_path(int number)
{
  float max_speed = max_speed_mm_s(number);
  float accel = accel_mm_s2(number);
  float diag_max_speed = diag_max_speed_mm_s(number);
  float diag_accel = diag_accel_mm_s2(number);
  float speed_mm_s = 0.0f;
  bool last_was_turn = false;

  for (int i = 0; i < s_path.count; i++) {
    motion_t motion = s_path.steps[i].motion;
    bool is_last = (i == s_path.count - 1);
    last_was_turn = false;

    if (motion == MOTION_STRAIGHT && is_last) {
      run_fast_stop(straight_distance_mm(i), max_speed, accel);  /* ゴールの区画のまん中で止まる */
      speed_mm_s = 0.0f;
    } else if (motion == MOTION_STRAIGHT) {
      float distance_mm = straight_distance_mm(i);
      float limit_mm_s = speed_limit_ahead_mm_s(i + 1, accel, diag_accel);
      float end_mm_s = straight_end_speed_mm_s(speed_mm_s, distance_mm, accel, max_speed, limit_mm_s);
      run_fast_straight(distance_mm, max_speed, end_mm_s, accel, straight_start_in_cell_mm(i));
      speed_mm_s = end_mm_s;
    } else if (motion == MOTION_DIAG_STRAIGHT) {
      float distance_mm = diag_distance_mm(i);
      float limit_mm_s = speed_limit_ahead_mm_s(i + 1, accel, diag_accel);
      float end_mm_s = straight_end_speed_mm_s(speed_mm_s, distance_mm, diag_accel, diag_max_speed, limit_mm_s);
      run_fast_diag_straight(distance_mm, diag_max_speed, end_mm_s, diag_accel);
      speed_mm_s = end_mm_s;
    } else {
      /* ターン：今の速さで曲がる（speed_limit_ahead_mm_s() で、そのターンの速さ以下にしてある） */
      run_fast_turn(fast_turn_kind(motion), is_left_turn(motion), speed_mm_s);
      last_was_turn = true;
    }
    if (failsafe_is_stopped()) {
      return;
    }
  }

  if (last_was_turn) {
    /* ゴールにターンで入って、区画のまん中に今の速さで着いた：そこから止まる（SHORTEST_STOP_DISTANCE_MM 以内） */
    float stop_mm = speed_mm_s * speed_mm_s / (2.0f * SHORTEST_STOP_ACCEL_MM_S2);
    run_fast_stop(stop_mm, speed_mm_s, SHORTEST_STOP_ACCEL_MM_S2);
  }
}

/* No.4〜7：吸引を No. ごとの電圧まで上げて、少し待つ（先頭のコメントの 1） */
static void start_suction(int number)
{
  suction_ramp_up_to(suction_voltage_v(number));  /* 0.5 秒かけて吸引を上げる（sys/suction.c） */
  ui_wait_ms(SHORTEST_SUCTION_WAIT_MS);
}

/* ゴールで止まったあと（先頭のコメントの「■ ゴールで止まったあと」）：止まった区画の壁を見て、入ってきた向きを向き、
 * その位置を覚える。最後がターンなら、止まった位置がずれているかもしれないので、何もしない */
static void turn_back_at_goal(void)
{
  int last = s_path.count - 1;
  if (s_path.steps[last].motion != MOTION_STRAIGHT) {
    return;
  }

  /* 1. 止まった区画の壁を見る（まだ、入ってきた向きを向いている） */
  bool wall_front = wall_ctrl_has_wall_front();
  bool wall_left = wall_ctrl_has_wall_left();
  bool wall_right = wall_ctrl_has_wall_right();

  /* 2. 前壁合わせをしながら、入ってきた向きを向く */
  run_search_turn_back_at_center();
  if (failsafe_is_stopped()) {
    return;  /* 回っている途中でぶつかった：覚えない */
  }

  /* 3. 覚える */
  s_end_pose.x = s_path.end_x;
  s_end_pose.y = s_path.end_y;
  s_end_pose.arrive_dir = s_path.end_dir;
  s_end_pose.wall_front = wall_front;
  s_end_pose.wall_left = wall_left;
  s_end_pose.wall_right = wall_right;
  s_end_pose.is_set = true;
}

/* 最後の最短走行で、ゴールで止まったときに覚えた位置（覚えていなければ is_set が false） */
shortest_end_pose_t shortest_get_end_pose(void)
{
  return s_end_pose;
}

/* 最短走行をする（流れは先頭のコメント）。number は No.（走り方と速さを選ぶ） */
shortest_result_t shortest_run(int number, int goal_x, int goal_y)
{
  s_end_pose.is_set = false;  /* 前の最短走行で覚えた位置は、もう使えない */
  if (!make_path(number, goal_x, goal_y)) {
    return SHORTEST_RESULT_NO_ROUTE;
  }
  float max_speed = max_speed_mm_s(number);
  float accel = accel_mm_s2(number);

  if (uses_diagonal(number)) {
    start_suction(number);
  }
  run_fast_set_turn_set(turn_set_of(number));  /* No.6 は組 2、No.7 は組 3 で曲がる */
  run_fast_start();  /* ジャイロのゼロ点を測り直してから、制御を始める（No.4〜7 は、吸ったまま測る） */
  if (uses_diagonal(number)) {
    run_diagonal_path(number);
  } else {
    for (int i = 0; i < s_path.count; i++) {
      run_small_turn_step(i, max_speed, accel);
      if (failsafe_is_stopped()) {
        break;
      }
    }
  }

  fan_stop();  /* 吸引を止める（No.1〜3 は回していないので、何も変わらない） */
  run_fast_set_turn_set(1);  /* 組 1 に戻す */

  if (failsafe_is_stopped()) {
    run_fast_stop(0.0f, 0.0f, accel);  /* 制御とログを止める（フェイルセーフのときは、動かさずに止めるだけ） */
    return SHORTEST_RESULT_FAILSAFE;
  }

  turn_back_at_goal();  /* 止まった区画の壁を見て、入ってきた向きを向き、位置を覚える */
  if (failsafe_is_stopped()) {
    return SHORTEST_RESULT_FAILSAFE;  /* 回っている途中でぶつかった */
  }
  return SHORTEST_RESULT_DONE;
}
