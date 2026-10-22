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

#include "drv/wall.h"

#define FRONT_LEFT_WALL_THRESHOLD  600   /* 左前：これより大きければ前に壁 */
#define FRONT_RIGHT_WALL_THRESHOLD 770   /* 右前：同じく */
#define LEFT_WALL_THRESHOLD        700   /* 左：これより大きければ左に壁 */
#define RIGHT_WALL_THRESHOLD       550   /* 右：これより大きければ右に壁 */
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

/* 前センサに手がかざされているか（左前と右前の両方が、壁よりずっと近いときの値を超えている） */
bool wall_ctrl_is_hand_front(void)
{
  if (wall_ctrl_get_front_left() > FRONT_LEFT_HAND_THRESHOLD
      && wall_ctrl_get_front_right() > FRONT_RIGHT_HAND_THRESHOLD) {
    return true;
  }
  return false;
}
