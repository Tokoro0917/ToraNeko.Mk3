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

#define FRONT_LEFT_WALL_THRESHOLD  570   /* 左前：これより大きければ前に壁 */
#define FRONT_RIGHT_WALL_THRESHOLD 900   /* 右前：同じく */
#define LEFT_WALL_THRESHOLD        650   /* 左：これより大きければ左に壁 */
#define RIGHT_WALL_THRESHOLD       400   /* 右：これより大きければ右に壁 */
/* 区画のまん中にいるときの、横のセンサの値。壁に沿って走る補正の目標。
 * 2026-11-02 に、前の壁なしで測った 2 回の平均（左 1426・1495、右 922・961）。壁に沿って走るのは、前に壁がない直進のときが
 * ほとんどなので、前の壁なしの値にした（前の壁があると、左 1879・右 1137 に上がる）。
 * 右は 2026-11-03 に 942 → 815 にした：探索の走行ログで、左右に壁がある直進で左が約 42mm（まん中）のとき、右は 805〜830 だった
 * （止めて測ったときより 1〜2 割低い。右のセンサは斜め前を見るので、走っているときは光の当たる場所が違うらしい）。
 * 942 のままだと、右の壁から遠いと読んで、まん中より右に寄ったところで落ち着いていた */
#define LEFT_WALL_REFERENCE  1460
#define RIGHT_WALL_REFERENCE 815

/* 壁に沿って走る補正に、その壁を使う下限。壁のあり・なしのしきい値（左 650・右 400）より高くする。
 * 壁の端（柱の近く）では、壁があっても値がまだ上がりきっておらず、補正が「壁から遠い」と読み違えて
 * 壁のほうへ曲げてしまう（2026-10-24 の探索で、右 574 を遠いと読んで右の壁に当たった）。
 * まん中の値のおよそ 7 割より大きいときだけ使う */
#define LEFT_WALL_CONTROL_MIN  1000  /* 左のまん中の値 1460 の約 7 割 */
#define RIGHT_WALL_CONTROL_MIN 570   /* 右のまん中の値 815 の約 7 割 */

/* 区画のまん中にいるときの、前のセンサの値。前壁合わせの目標。
 * 2026-11-02 に、センサを付け直してから、3 枚とも壁ありの区画のまん中で、センサ値の記録（モード5 No.2）で 1 秒の平均を測った値
 * （左前 3333.7、右前 3391.8。付け直す前は 3244・3240） */
#define FRONT_LEFT_REFERENCE  3334
#define FRONT_RIGHT_REFERENCE 3392

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
