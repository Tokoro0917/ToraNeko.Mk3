/*
 * wall_ctrl.c
 * 壁のあり・なしの判定（壁に沿って走るための補正は、ステップ6・7で足す）
 *
 * ■ 受光の番号とセンサ（2026-10-22 に実機で確かめた。drv/wall.h）
 *   左前 = 受光 1（組A で光らせる）、右の壁 = 受光 2（組A）、左の壁 = 受光 3（組B）、右前 = 受光 4（組B）
 * 横の 2 つは、光の向きが機体の真ん中で交差する。右を見るセンサは機体の左側に付いている。
 *
 * ■ しきい値（2026-10-22 に、区画のまん中に置いて測った値から決めた）
 *   ホルダを付け直したあとに測った値（ON−OFF）：
 *                              左前   左     右     右前
 *     壁なし（周りに何もない）   326    226    146    520
 *     3 枚とも壁あり             895   1457   1148   1142
 *     左の壁だけなし             780    225   1123   1111
 *     右の壁だけなし             921   1484    165    982
 *     前の壁だけなし             396   1834    947    556
 *   しきい値は、壁があるときのいちばん小さい値と、壁がないときの値の、だいたいまん中に置いた。
 *   - 前：左前は 396（なし）と 780（あり）のあいだの 600、右前は 556 と 982 のあいだの 770
 *     （横の壁の光も少し入るので、横の壁がないと 100〜160 下がる。「あり」はその下がった値で考えた）
 *   - 横：左は 226 と 1457 のあいだの 700、右は 165 と 947 のあいだの 550
 *     （横は、機体が左右にずれると値が大きく変わる（前の壁を外したとき、左 1457 → 1834）ので、低めに置いた）
 *   走りながら壁を見る場所は区画のまん中とは限らないので、走らせるようになったら（ステップ7・9）見直す。
 *
 * ■ 手のしきい値（仮）
 *   前の壁があるとき（左前 約 900、右前 約 1100）より十分大きい値。ホルダを付け直したあとは、まだ確かめていない。
 */
#include "ctrl/wall_ctrl.h"

#include "ctrl/wall_dist.h"
#include "drv/wall.h"

#define FRONT_LEFT_WALL_THRESHOLD  600   /* 左前：これより大きければ前に壁 */
#define FRONT_RIGHT_WALL_THRESHOLD 770   /* 右前：同じく */
#define LEFT_WALL_THRESHOLD        700   /* 左：これより大きければ左に壁 */
#define RIGHT_WALL_THRESHOLD       550   /* 右：これより大きければ右に壁 */
/* 区画のまん中にいるときの、横のセンサの値（2026-10-22、3 枚とも壁ありで測った値）。壁に沿って走る補正の目標 */
#define LEFT_WALL_REFERENCE  1457
#define RIGHT_WALL_REFERENCE 1148

/* 壁に沿って走る補正に、その壁を使う下限。壁のあり・なしのしきい値（左 700・右 550）より高くする。
 * 壁の端（柱の近く）では、壁があっても値がまだ上がりきっておらず、補正が「壁から遠い」と読み違えて
 * 壁のほうへ曲げてしまう（2026-10-24 の探索で、右 574 を遠いと読んで右の壁に当たった）。
 * まん中の値のおよそ 7 割より大きいときだけ使う */
/* 区画のまん中にいるときの、前のセンサの値。前壁合わせの目標。
 * 2026-10-24 に、3 枚とも壁ありの区画のまん中で、センサ値の記録（モード5 No.2）で 1 秒の平均を測った値
 * （左前 3243.9、右前 3240.2。前は 895・1142 にしていた） */
#define FRONT_LEFT_REFERENCE  3244
#define FRONT_RIGHT_REFERENCE 3240

#define LEFT_WALL_CONTROL_MIN  1000
#define RIGHT_WALL_CONTROL_MIN 800

/* 横の壁からのずれの上限（センサの値の差。約 2mm 分）。急に大きく曲げないようにする */
#define SIDE_ERROR_LIMIT 600.0f
#define FRONT_LEFT_HAND_THRESHOLD  2800  /* 左前：これより大きければ手がかざされている（仮） */
#define FRONT_RIGHT_HAND_THRESHOLD 2000  /* 右前：同じく（仮） */

/* 左前のセンサの値（受光 1、組A） */
int32_t wall_ctrl_get_front_left(void)
{
  return wall_get_a_minus_dark(1);
}

/* 左の壁のセンサの値（受光 3、組B） */
int32_t wall_ctrl_get_left(void)
{
  return wall_get_b_minus_dark(3);
}

/* 右の壁のセンサの値（受光 2、組A） */
int32_t wall_ctrl_get_right(void)
{
  return wall_get_a_minus_dark(2);
}

/* 右前のセンサの値（受光 4、組B） */
int32_t wall_ctrl_get_front_right(void)
{
  return wall_get_b_minus_dark(4);
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

/* 左の壁を、壁に沿って走る補正に使えるか（値がまん中の値に近いか） */
static bool can_use_left_wall(void)
{
  if (wall_ctrl_get_left() > LEFT_WALL_CONTROL_MIN) {
    return true;
  }
  return false;
}

/* 右の壁を、壁に沿って走る補正に使えるか */
static bool can_use_right_wall(void)
{
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
  float left_error = wall_ctrl_get_left() - LEFT_WALL_REFERENCE;
  float right_error = wall_ctrl_get_right() - RIGHT_WALL_REFERENCE;
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
