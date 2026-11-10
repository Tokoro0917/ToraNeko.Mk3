/*
 * wall_ctrl.c
 * 壁のあり・なしの判定（壁に沿って走るための補正は、ステップ6・7で足す）
 *
 * ■ 受光の番号とセンサ（2026-10-22 に実機で確かめた。drv/wall.h）
 *   左前 = 受光 1（組A で光らせる）、右の壁 = 受光 2（組A）、左の壁 = 受光 3（組B）、右前 = 受光 4（組B）
 * 横の 2 つは、光の向きが機体の真ん中で交差する。右を見るセンサは機体の左側に付いている。
 *
 * ■ しきい値（2026-11-02 に、センサを付け直してから、区画のまん中で測った値（モード5 No.2 の 1 秒の平均）から決めた）
 *                                              左前   左     右     右前
 *     壁なし（周りに何もない）                   79    117     70    346
 *     3 枚とも壁あり                           3334   1879   1137   3392
 *     前の壁なし（2 回）                     374〜387 1426〜1495 922〜961 689〜702
 *     左の壁なし（前の壁は 1 区画先）            -    124    803     -
 *     右の壁なし（前の壁は 1 区画先）            -   1645     86     -
 *   - 横：左は 124（なし）と 1426（ありで一番小さい）のあいだの 650、右は 86 と 803 のあいだの 400。
 *     横は、前の壁があるかで 2〜3 割変わる（前の壁の光も入る）ので、低めに置いた
 *   - 前：探索で前の壁を見るのは区画の入口（前の壁まで約 87mm）なので、入口の位置（基板の前の端と前の壁のすきま 55mm、
 *     横の壁あり）でも測った：前の壁あり 左前 881・右前 1390、前の壁なし 左前 257・右前 439。
 *     左前はそのまん中の 570、右前は 900。区画のまん中で前の壁がないとき（左前 約 380・右前 約 700）よりも上にある
 *   （付け直す前、2026-10-22 の値：3 枚とも壁あり 895・1457・1148・1142。しきい値は 600・700・550・770）
 *
 * ■ 手のしきい値（仮）
 *   前の壁が入口の距離にあるときより十分大きい値。センサを付け直したあと（2026-11-02）は、まだ確かめていない。
 */
#include "ctrl/wall_ctrl.h"

#include "config/run_params.h"
#include "ctrl/wall_dist.h"
#include "drv/wall.h"

#define FRONT_LEFT_WALL_THRESHOLD  300   /* 左前：これより大きければ前に壁 */
#define FRONT_RIGHT_WALL_THRESHOLD 700   /* 右前：同じく（2026-11-10 に 900 → 700。横の壁がないところから行き止まりに入ると、入口で右前が 900 を下回って前の壁を見落とし、ぶつかっていた） */
#define LEFT_WALL_THRESHOLD        500   /* 左：これより大きければ左に壁 */
#define RIGHT_WALL_THRESHOLD       400   /* 右：これより大きければ右に壁 */
/* 区画のまん中にいるときの、横のセンサの値。壁に沿って走る補正の目標。
 * 2026-11-02 に、前の壁なしで測った 2 回の平均（左 1426・1495、右 922・961）。壁に沿って走るのは、前に壁がない直進のときが
 * ほとんどなので、前の壁なしの値にした（前の壁があると、左 1879・右 1137 に上がる）。
 * 右は 2026-11-03 に 942 → 815 にした：探索の走行ログで、左右に壁がある直進で左が約 42mm（まん中）のとき、右は 805〜830 だった
 * （止めて測ったときより 1〜2 割低い。右のセンサは斜め前を見るので、走っているときは光の当たる場所が違うらしい）。
 * 942 のままだと、右の壁から遠いと読んで、まん中より右に寄ったところで落ち着いていた。
 * 2026-11-10 に作者が、ほかの迷路で 1205・758 にしたが、それは感度合わせの倍率が間違っていた（横の値が小さく直されていた）
 * のを埋め合わせたものだった。同じ日に、会場では、区画の中心に置いて測った値を目標にできるようにした
 * （モード7 No.4。下の wall_reference_left()）。ここの値は、会場で測っていないときに使う、家の迷路の値 */
#define LEFT_WALL_REFERENCE  WALL_HOME_CENTER_LEFT   /* 1460（ctrl/wall_ctrl.h） */
#define RIGHT_WALL_REFERENCE WALL_HOME_CENTER_RIGHT  /* 815 */

/* 壁に沿って走る補正に、その壁を使う下限。壁のあり・なしのしきい値（左 650・右 400）より高くする。
 * 壁の端（柱の近く）では、壁があっても値がまだ上がりきっておらず、補正が「壁から遠い」と読み違えて
 * 壁のほうへ曲げてしまう（2026-10-24 の探索で、右 574 を遠いと読んで右の壁に当たった）。
 * まん中の値のおよそ 7 割より大きいときだけ使う */
#define LEFT_WALL_CONTROL_MIN  1000  /* 左のまん中の値 1460 の約 7 割 */
#define RIGHT_WALL_CONTROL_MIN 570   /* 右のまん中の値 815 の約 7 割 */

/* 壁の終わりの見分け方（2026-11-04）。壁が終わるところでは、値が下限（上の CONTROL_MIN）を切る前から下がり始め、
 * 補正が「壁から遠い」と読んで、壁のほうへ曲げてしまう（柱のテストで、左の壁の終わりで 71deg/s 左へ曲げた）。
 * そこで、進んだ距離あたりの下がり方を見る：WALL_END_WINDOW_MM 進むあいだに WALL_END_DROP より下がったら、
 * 壁の終わりとみなし、そこから WALL_END_HOLD_MM 進むまで、その壁を補正に使わない。
 * 値の決め方：今ある実機のログ（探索・最短走行・ターン調整・柱のテスト）に当てはめた。60 にすると、壁の終わり 24 回の
 * すべてで、値が「壁あり」のしきい値を切る 12〜22mm 手前で見分けられ、壁に沿って走っているとき（約 13000 点）に
 * 見分けてしまったのは 0.3%（見分けたあいだは、向きを迷路の向きに戻す補正が効くだけなので、まちがえても害は小さい） */
#define WALL_END_WINDOW_MM 5     /* 下がり方を見る距離 [mm]（1mm ごとに値を覚えておく） */
#define WALL_END_DROP      60    /* WALL_END_WINDOW_MM 進むあいだに、これより下がったら壁の終わり */
#define WALL_END_HOLD_MM   15.0f /* 壁の終わりとみなしてから、その壁を使わない距離 [mm] */

/* 区画のまん中にいるときの、前のセンサの値。前壁合わせの目標。
 * 2026-11-02 に、センサを付け直してから、3 枚とも壁ありの区画のまん中で、センサ値の記録（モード5 No.2）で 1 秒の平均を測った値
 * （左前 3333.7、右前 3391.8。付け直す前は 3244・3240） */
#define FRONT_LEFT_REFERENCE  3334
#define FRONT_RIGHT_REFERENCE 3392

/* 横の壁からのずれの上限（センサの値の差。約 2mm 分）。急に大きく曲げないようにする */
#define SIDE_ERROR_LIMIT 600.0f
/* 斜めの直進で、柱や壁に近づいたとみなす前のセンサの値（斜めの補正。KOGUMA の calWallConrol_NANAME）。
 * 斜めの直進では、通り道の両側に柱が並ぶ（通り道のまん中の線から、柱までは 31.8mm）。まん中を走っていれば、前のセンサは
 * 柱を少し見るだけだが、どちらかに寄ると、寄った側の前のセンサの値が上がる。この値を超えた分だけ、反対へ曲げる。
 * 2026-11-04 に、斜めの直進のテスト（モード3 No.8。補正なし、800mm/s、斜め 6 つ）を 2 回走らせて決めた
 * （logs/20261104-043508_log.txt・20261104-043546_log.txt）。柱は、通り道の左右に 127mm ごとに並び、前のセンサの値は、
 * 柱を 1 つ見るたびに山になる：
 *   1 回目（最後まで走れた。ほぼまん中）：左前の山 1740・990、右前の山 1134・773・641
 *   2 回目（右へずれていった）：右前の山 2876、次の柱で 3526 になって右の柱に当たった（フェイルセーフで止まった）
 * まん中を走ったときの山より上で、ずれていったときの 1 つ目の山（2876）より十分下にした。
 * 左前は、ずれていったときの値をまだ測っていない（右前と同じくらいの余裕にした）。
 * そのあと、補正を入れて（モード3 No.9）右へずれていく置き方で走らせたら、右前が 1500 を超えたところで左へ曲げ、
 * 当たらずに最後まで走れたので、この値に決めた（2026-11-04。logs/20261104-044732_log.txt） */
#define DIAG_FRONT_LEFT_THRESHOLD  2000  /* 左前：これより大きければ、左の柱・壁に近い（まん中の山 1740 より上） */
#define DIAG_FRONT_RIGHT_THRESHOLD 1500  /* 右前：これより大きければ、右の柱・壁に近い（まん中の山 1134 より上） */

#define FRONT_LEFT_HAND_THRESHOLD  2800  /* 左前：これより大きければ手がかざされている（仮） */
#define FRONT_RIGHT_HAND_THRESHOLD 2000  /* 右前：同じく（仮） */

/* 会場ごとの感度の倍率（wall_ctrl.h の sensor_scale_t）。はじめは 1（直さない）。
 * メインの処理（感度合わせ、電源を入れたとき）で書き、制御周期の割り込みでも読むので volatile にする。
 * 書くのは機体が止まっているときだけなので、4 つを書く途中で割り込みが入っても、走りには影響しない */
static volatile float s_scale_front_left = 1.0f;
static volatile float s_scale_left = 1.0f;
static volatile float s_scale_right = 1.0f;
static volatile float s_scale_front_right = 1.0f;

/* 会場ごとの感度の倍率を入れる（sys/venue.c） */
void wall_ctrl_set_sensor_scale(sensor_scale_t scale)
{
  s_scale_front_left = scale.front_left;
  s_scale_left = scale.left;
  s_scale_right = scale.right;
  s_scale_front_right = scale.front_right;
}

/* 今の会場ごとの感度の倍率 */
sensor_scale_t wall_ctrl_get_sensor_scale(void)
{
  sensor_scale_t scale;
  scale.front_left = s_scale_front_left;
  scale.left = s_scale_left;
  scale.right = s_scale_right;
  scale.front_right = s_scale_front_right;
  return scale;
}

/* 横の壁に沿って走る補正の目標（wall_ctrl.h の wall_reference_t）。会場で測った値（そのままの値）。
 * メインの処理（モード7 No.4、電源を入れたとき）で書き、制御周期の割り込みでも読むので volatile にする */
static volatile bool s_reference_is_set = false;
static volatile float s_reference_left_raw = 0.0f;
static volatile float s_reference_right_raw = 0.0f;

/* 横の壁に沿って走る補正の目標を入れる（sys/venue.c） */
void wall_ctrl_set_wall_reference(wall_reference_t reference)
{
  s_reference_is_set = reference.is_set;
  s_reference_left_raw = reference.left;
  s_reference_right_raw = reference.right;
}

/* 今の横の壁に沿って走る補正の目標（会場で測った値） */
wall_reference_t wall_ctrl_get_wall_reference(void)
{
  wall_reference_t reference;
  reference.is_set = s_reference_is_set;
  reference.left = s_reference_left_raw;
  reference.right = s_reference_right_raw;
  return reference;
}

/* 左の壁の目標。会場で測った値があれば、それを倍率で割ったもの（wall_ctrl_get_left() と同じく、家の迷路での値に直す。
 * こうすると、左のセンサのそのままの値が、測った値になるところへ寄っていく）。なければ LEFT_WALL_REFERENCE */
static float wall_reference_left(void)
{
  if (s_reference_is_set) {
    return s_reference_left_raw / s_scale_left;
  }
  return LEFT_WALL_REFERENCE;
}

/* 右の壁の目標（左と同じ） */
static float wall_reference_right(void)
{
  if (s_reference_is_set) {
    return s_reference_right_raw / s_scale_right;
  }
  return RIGHT_WALL_REFERENCE;
}

/* そのままの値 raw_value を、倍率 scale で割って、表を作った迷路での値に直す（小数は切り捨てて整数にする） */
static int32_t to_home_value(int32_t raw_value, float scale)
{
  int32_t home_value = raw_value / scale;
  return home_value;
}

/* 左前のセンサの、そのままの値（受光 1、組A） */
int32_t wall_ctrl_get_front_left_raw(void)
{
  return wall_get_a_minus_dark(1);
}

/* 左の壁のセンサの、そのままの値（受光 3、組B） */
int32_t wall_ctrl_get_left_raw(void)
{
  return wall_get_b_minus_dark(3);
}

/* 右の壁のセンサの、そのままの値（受光 2、組A） */
int32_t wall_ctrl_get_right_raw(void)
{
  return wall_get_a_minus_dark(2);
}

/* 右前のセンサの、そのままの値（受光 4、組B） */
int32_t wall_ctrl_get_front_right_raw(void)
{
  return wall_get_b_minus_dark(4);
}

/* 左前のセンサの値（会場ごとの感度の倍率で直したもの） */
int32_t wall_ctrl_get_front_left(void)
{
  return to_home_value(wall_ctrl_get_front_left_raw(), s_scale_front_left);
}

/* 左の壁のセンサの値（同じ） */
int32_t wall_ctrl_get_left(void)
{
  return to_home_value(wall_ctrl_get_left_raw(), s_scale_left);
}

/* 右の壁のセンサの値（同じ） */
int32_t wall_ctrl_get_right(void)
{
  return to_home_value(wall_ctrl_get_right_raw(), s_scale_right);
}

/* 右前のセンサの値（同じ） */
int32_t wall_ctrl_get_front_right(void)
{
  return to_home_value(wall_ctrl_get_front_right_raw(), s_scale_front_right);
}

/* 横の壁が近すぎた記録（探索の横壁合わせ。探索を始めるたびに 0 に戻す） */
static int s_too_close_count = 0;        /* 近すぎた回数 */
static float s_too_close_min_mm = 0.0f;  /* いちばん近かった距離 [mm] */

/* 横の壁があって、その距離 distance_mm が SIDE_TOO_CLOSE_MM より近いか。近ければ記録する。
 * SIDE_ALIGN_ENABLE が 0 なら、記録だけして false を返す（探索は止まらない） */
static bool check_too_close(bool has_wall, float distance_mm)
{
  if (!has_wall || distance_mm >= SIDE_TOO_CLOSE_MM) {
    return false;
  }
  if (s_too_close_count == 0 || distance_mm < s_too_close_min_mm) {
    s_too_close_min_mm = distance_mm;
  }
  s_too_close_count = s_too_close_count + 1;
  if (SIDE_ALIGN_ENABLE == 0) {
    return false;
  }
  return true;
}

/* 左の壁が近すぎるか（区画の入口で呼ぶ。探索の横壁合わせ） */
bool wall_ctrl_is_left_too_close(void)
{
  return check_too_close(wall_ctrl_has_wall_left(), wall_ctrl_get_left_mm());
}

/* 右の壁が近すぎるか */
bool wall_ctrl_is_right_too_close(void)
{
  return check_too_close(wall_ctrl_has_wall_right(), wall_ctrl_get_right_mm());
}

/* 横の壁が近すぎた記録を 0 に戻す（探索を始めるときに呼ぶ） */
void wall_ctrl_clear_too_close(void)
{
  s_too_close_count = 0;
  s_too_close_min_mm = 0.0f;
}

/* 横の壁が近すぎた回数と、いちばん近かった距離 [mm]（モード7 No.1 で出す） */
int wall_ctrl_get_too_close_count(void)
{
  return s_too_close_count;
}

float wall_ctrl_get_too_close_min_mm(void)
{
  return s_too_close_min_mm;
}

/* 前の壁までの距離 [mm]（左前と右前の平均。区画のまん中で 42mm） */
float wall_ctrl_get_front_distance_mm(void)
{
  return (wall_ctrl_get_front_left_mm() + wall_ctrl_get_front_right_mm()) / 2.0f;
}

/* 前の壁までの距離 [mm]（左前のセンサから。表は ctrl/wall_dist.c） */
float wall_ctrl_get_front_left_mm(void)
{
  return wall_dist_front_left_mm(wall_ctrl_get_front_left());
}

/* 左の壁までの距離 [mm] */
float wall_ctrl_get_left_mm(void)
{
  return wall_dist_left_mm(wall_ctrl_get_left());
}

/* 右の壁までの距離 [mm] */
float wall_ctrl_get_right_mm(void)
{
  return wall_dist_right_mm(wall_ctrl_get_right());
}

/* 前の壁までの距離 [mm]（右前のセンサから） */
float wall_ctrl_get_front_right_mm(void)
{
  return wall_dist_front_right_mm(wall_ctrl_get_front_right());
}

/* 前に壁があるか（左前と右前の両方が、しきい値より大きいとき。KOGUMA と同じ） */
bool wall_ctrl_has_wall_front(void)
{
  if (wall_ctrl_get_front_left() > FRONT_LEFT_WALL_THRESHOLD
      && wall_ctrl_get_front_right() > FRONT_RIGHT_WALL_THRESHOLD) {
    return true;
  }
  return false;
}

/* 左に壁があるか */
bool wall_ctrl_has_wall_left(void)
{
  if (wall_ctrl_get_left() > LEFT_WALL_THRESHOLD) {
    return true;
  }
  return false;
}

/* 右に壁があるか */
bool wall_ctrl_has_wall_right(void)
{
  if (wall_ctrl_get_right() > RIGHT_WALL_THRESHOLD) {
    return true;
  }
  return false;
}

/* ---- 壁の終わりの見分け（上の WALL_END_* のコメント） ----
 * 1mm 進むごとに、左右のセンサの値を覚える（いちばん新しい WALL_END_WINDOW_MM + 1 個）。
 * 覚えておく場所は、s_end_index の場所に書いたら 1 つ進め、端まで来たら 0 に戻す（いちばん古い値は、次に書く場所にある） */
#define END_HISTORY_COUNT (WALL_END_WINDOW_MM + 1)
static int32_t s_left_history[END_HISTORY_COUNT];
static int32_t s_right_history[END_HISTORY_COUNT];
static int s_end_index = 0;                  /* 次に書く場所 */
static int s_end_history_filled = 0;         /* 覚えた数（END_HISTORY_COUNT まで） */
static float s_end_last_mm = 0.0f;           /* 最後に覚えたときの、走った距離 [mm] */
static float s_now_mm = 0.0f;                /* 今の、走った距離 [mm] */
static float s_left_unusable_until_mm = 0.0f;   /* 左の壁を、この距離まで補正に使わない */
static float s_right_unusable_until_mm = 0.0f;  /* 右の壁を、この距離まで補正に使わない */

/* 壁の終わりの見分けを、最初からやり直す（制御を始めるとき。ctrl/control.c） */
void wall_ctrl_reset_end_check(float distance_mm)
{
  s_end_index = 0;
  s_end_history_filled = 0;
  s_end_last_mm = distance_mm;
  s_now_mm = distance_mm;
  s_left_unusable_until_mm = distance_mm;
  s_right_unusable_until_mm = distance_mm;
}

/* 制御周期ごとに呼ぶ（ctrl/control.c）。distance_mm は、今の走った距離（エンコーダで測った値）。
 * 1mm 進んでいたら左右の値を覚え、WALL_END_WINDOW_MM 前の値から WALL_END_DROP より下がっていたら、壁の終わりとみなす */
void wall_ctrl_update_end_check(float distance_mm)
{
  s_now_mm = distance_mm;
  if (distance_mm < s_end_last_mm + 1.0f) {
    return;  /* まだ 1mm 進んでいない（止まっている、下がっているときも、覚えない） */
  }
  s_end_last_mm = distance_mm;

  int32_t left = wall_ctrl_get_left();
  int32_t right = wall_ctrl_get_right();
  if (s_end_history_filled >= END_HISTORY_COUNT) {
    /* いちばん古い値（WALL_END_WINDOW_MM 前）は、これから書く場所にある */
    int32_t left_before = s_left_history[s_end_index];
    int32_t right_before = s_right_history[s_end_index];
    if (left_before - left > WALL_END_DROP) {
      s_left_unusable_until_mm = distance_mm + WALL_END_HOLD_MM;
    }
    if (right_before - right > WALL_END_DROP) {
      s_right_unusable_until_mm = distance_mm + WALL_END_HOLD_MM;
    }
  } else {
    s_end_history_filled = s_end_history_filled + 1;
  }
  s_left_history[s_end_index] = left;
  s_right_history[s_end_index] = right;
  s_end_index = s_end_index + 1;
  if (s_end_index >= END_HISTORY_COUNT) {
    s_end_index = 0;
  }
}

/* 左の壁を、壁に沿って走る補正に使えるか（値がまん中の値に近く、壁の終わりではない） */
static bool can_use_left_wall(void)
{
  if (s_now_mm < s_left_unusable_until_mm) {
    return false;  /* 壁の終わり */
  }
  if (wall_ctrl_get_left() > LEFT_WALL_CONTROL_MIN) {
    return true;
  }
  return false;
}

/* 右の壁を、壁に沿って走る補正に使えるか */
static bool can_use_right_wall(void)
{
  if (s_now_mm < s_right_unusable_until_mm) {
    return false;  /* 壁の終わり */
  }
  if (wall_ctrl_get_right() > RIGHT_WALL_CONTROL_MIN) {
    return true;
  }
  return false;
}

/* 横の壁からのずれ（+ なら左の壁に近い）。上限は ±SIDE_ERROR_LIMIT。
 * 両方の壁が使えれば (左 − 左の目標) − (右 − 右の目標)、片方だけなら、その壁のずれを 2 倍にする
 * （両方のときと、同じずれで同じくらいの大きさになるように）。どちらも使えなければ 0 */
float wall_ctrl_get_side_error(void)
{
  float left_error = wall_ctrl_get_left() - wall_reference_left();
  float right_error = wall_ctrl_get_right() - wall_reference_right();
  bool left = can_use_left_wall();
  bool right = can_use_right_wall();

  float error = 0.0f;
  if (left && right) {
    error = left_error - right_error;
  } else if (left) {
    error = 2.0f * left_error;
  } else if (right) {
    error = -2.0f * right_error;
  }

  if (error > SIDE_ERROR_LIMIT) {
    error = SIDE_ERROR_LIMIT;
  }
  if (error < -SIDE_ERROR_LIMIT) {
    error = -SIDE_ERROR_LIMIT;
  }
  return error;
}

/* 横の壁からのずれを求めるのに使う壁（0 なし、1 左、2 右、3 両方。ログに残す） */
int wall_ctrl_get_side_walls_used(void)
{
  int used = 0;
  if (can_use_left_wall()) {
    used = used + 1;
  }
  if (can_use_right_wall()) {
    used = used + 2;
  }
  return used;
}

/* 斜めの直進で、柱や壁に近づいたずれ（+ なら左の柱・壁に近い。斜めの補正で使う）。KOGUMA の calWallConrol_NANAME と同じく、
 * 左前がしきい値を超えていれば（左前 − しきい値）、そうでなく右前が超えていれば −（右前 − しきい値）、どちらでもなければ 0 */
float wall_ctrl_get_diag_error(void)
{
  int32_t front_left = wall_ctrl_get_front_left();
  int32_t front_right = wall_ctrl_get_front_right();
  if (front_left > DIAG_FRONT_LEFT_THRESHOLD) {
    return front_left - DIAG_FRONT_LEFT_THRESHOLD;
  }
  if (front_right > DIAG_FRONT_RIGHT_THRESHOLD) {
    return -(front_right - DIAG_FRONT_RIGHT_THRESHOLD);
  }
  return 0.0f;
}

/* 斜めの補正で、どちらの前のセンサがしきい値を超えたか（0 なし、1 左前、2 右前。ログに残す） */
int wall_ctrl_get_diag_used(void)
{
  if (wall_ctrl_get_front_left() > DIAG_FRONT_LEFT_THRESHOLD) {
    return 1;
  }
  if (wall_ctrl_get_front_right() > DIAG_FRONT_RIGHT_THRESHOLD) {
    return 2;
  }
  return 0;
}

/* 前の壁との距離のずれ（+ なら近すぎる）。左前と右前の、目標との差の平均 */
float wall_ctrl_get_front_distance_error(void)
{
  float left_error = wall_ctrl_get_front_left() - FRONT_LEFT_REFERENCE;
  float right_error = wall_ctrl_get_front_right() - FRONT_RIGHT_REFERENCE;
  return (left_error + right_error) / 2.0f;
}

/* 前の壁に対する向きのずれ（+ なら左前のほうが近い）。
 * 機体が右を向くと、左前のセンサが壁に近づき、右前のセンサが遠ざかるので、左前の差 − 右前の差 が + になる */
float wall_ctrl_get_front_angle_error(void)
{
  float left_error = wall_ctrl_get_front_left() - FRONT_LEFT_REFERENCE;
  float right_error = wall_ctrl_get_front_right() - FRONT_RIGHT_REFERENCE;
  return left_error - right_error;
}

/* 前センサに手がかざされているか（左前と右前の両方が、壁よりずっと近いときの値を超えている） */
bool wall_ctrl_is_hand_front(void)
{
  if (wall_ctrl_get_front_left() > FRONT_LEFT_HAND_THRESHOLD
      && wall_ctrl_get_front_right() > FRONT_RIGHT_HAND_THRESHOLD) {
    return true;
  }
  return false;
}
