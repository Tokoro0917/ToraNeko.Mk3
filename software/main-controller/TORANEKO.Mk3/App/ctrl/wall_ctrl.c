/*
 * wall_ctrl.c
 * 壁のあり・なしの判定（壁に沿って走るための補正は、ステップ6・7で足す）
 *
 * ■ 受光の番号とセンサ（2026-10-22 に実機で確かめた。drv/wall.h）
 *   左前 = 受光 1（組A で光らせる）、右の壁 = 受光 2（組A）、左の壁 = 受光 3（組B）、右前 = 受光 4（組B）
 * 横の 2 つは、光の向きが機体の真ん中で交差する。右を見るセンサは機体の左側に付いている。
 *
 * ■ しきい値（仮）
 * 何もないときの値が、まだセンサによってばらばら（左前 約 1200、ほか 300〜600。2026-10-22）なので、
 * それより十分大きい値を仮に置いた。壁までの距離と値の表を作るときに決め直す。
 */
#include "ctrl/wall_ctrl.h"

#include "drv/wall.h"

#define FRONT_LEFT_WALL_THRESHOLD  2000  /* 左前：これより大きければ前に壁（仮） */
#define FRONT_RIGHT_WALL_THRESHOLD 1000  /* 右前：同じく（仮） */
#define LEFT_WALL_THRESHOLD        1000  /* 左：これより大きければ左に壁（仮） */
#define RIGHT_WALL_THRESHOLD       1000  /* 右：これより大きければ右に壁（仮） */
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
