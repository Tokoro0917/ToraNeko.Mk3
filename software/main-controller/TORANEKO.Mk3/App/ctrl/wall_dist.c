/*
 * wall_dist.c
 * 壁センサの値を、壁までの距離 [mm] に直す（表を引いて、表の点のあいだは直線でつなぐ）
 *
 * KOGUMA の WallDistance.c と同じ考え方。HAL を使わないので、PC のテスト（test/test_wall_dist.c）でも動く。
 *
 * ■ 距離の決め方（区画のまん中にいるとき、どれも 42mm）
 *   横（左・右）：機体の中心線から、横の壁の表面まで
 *   前（左前・右前）：タイヤの軸から、前の壁の表面まで
 *   区画 90mm から壁の厚さ 6mm を引いて 2 で割ると 42mm（WALL_DIST_CENTER_MM）。
 *
 * ■ 表の作り方（MODES.md のモード5 No.2「壁までの距離の表の作り方」）
 *   1. 決めた距離に機体を置き、センサ値の記録（モード5 No.2）で 1 秒の平均を測る
 *   2. ビューアの「メモ」に「R 64mm」のように書いてから出して、logs/ に保存する
 *   3. tools/wall_table.py で logs/ のファイルを読むと、下の表に貼る行が出る
 *
 * ■ 表の書き方
 *   - 距離の近い順に並べる。センサの値は、距離が遠くなるほど小さくなること（同じ値が続いてもよい）
 *   - 表より近い（値が大きい）ときは表のいちばん近い距離、表より遠い（値が小さい）ときはいちばん遠い距離を返す
 *
 * ■ 今の表は「仮」
 *   2026-10-24 に区画のまん中で測った値（左前 3244、左 1691、右 1585、右前 3240）を 42mm とし、
 *   「値は距離の 2 乗に反比例する」として計算で作った値。センサを固定し直して測ったら、測った値に書きかえる。
 */
#include "ctrl/wall_dist.h"

/* ---- 左前（FL）の表（仮） ---- */
#define FRONT_LEFT_POINT_COUNT 9
static const float FRONT_LEFT_MM[FRONT_LEFT_POINT_COUNT] = {37, 42, 52, 62, 72, 87, 102, 122, 142};
static const int32_t FRONT_LEFT_VALUE[FRONT_LEFT_POINT_COUNT] = {4000, 3244, 2116, 1489, 1104, 756, 550, 384, 284};

/* ---- 左（L）の表（仮） ---- */
#define LEFT_POINT_COUNT 8
static const float LEFT_MM[LEFT_POINT_COUNT] = {22, 28, 34, 40, 46, 52, 58, 64};
static const int32_t LEFT_VALUE[LEFT_POINT_COUNT] = {4000, 3805, 2580, 1864, 1410, 1103, 887, 728};

/* ---- 右（R）の表（仮） ---- */
#define RIGHT_POINT_COUNT 8
static const float RIGHT_MM[RIGHT_POINT_COUNT] = {22, 28, 34, 40, 46, 52, 58, 64};
static const int32_t RIGHT_VALUE[RIGHT_POINT_COUNT] = {4000, 3566, 2419, 1747, 1321, 1034, 831, 683};

/* ---- 右前（FR）の表（仮） ---- */
#define FRONT_RIGHT_POINT_COUNT 9
static const float FRONT_RIGHT_MM[FRONT_RIGHT_POINT_COUNT] = {37, 42, 52, 62, 72, 87, 102, 122, 142};
static const int32_t FRONT_RIGHT_VALUE[FRONT_RIGHT_POINT_COUNT] = {4000, 3240, 2114, 1487, 1103, 755, 549, 384, 283};

/* センサの値 value を、表（距離 mm_table と値 value_table、point_count 点）で距離 [mm] に直す。
 * 表の i 番目と i+1 番目のあいだに value があれば、その 2 点を直線でつないだところの距離を返す */
static float value_to_mm(int32_t value, const float mm_table[], const int32_t value_table[], int point_count)
{
  int last = point_count - 1;
  if (value >= value_table[0]) {
    return mm_table[0];  /* 表よりも近い */
  }
  if (value <= value_table[last]) {
    return mm_table[last];  /* 表よりも遠い */
  }
  for (int i = 0; i < last; i++) {
    if (value >= value_table[i + 1]) {
      /* value は、i 番目（近いほう）と i+1 番目（遠いほう）のあいだ。
       * ratio は、i+1 番目から i 番目へ、どれだけ近づいたか（0〜1） */
      float value_span = value_table[i] - value_table[i + 1];
      if (value_span <= 0.0f) {
        return mm_table[i];  /* 同じ値が続いている（割り算ができない）ので、近いほうの距離にする */
      }
      float ratio = (value - value_table[i + 1]) / value_span;
      return mm_table[i + 1] + ratio * (mm_table[i] - mm_table[i + 1]);
    }
  }
  return mm_table[last];  /* ここには来ない（表が近い順に並んでいれば、上のどれかで戻る） */
}

/* 左前のセンサの値 → 前の壁までの距離 [mm] */
float wall_dist_front_left_mm(int32_t value)
{
  return value_to_mm(value, FRONT_LEFT_MM, FRONT_LEFT_VALUE, FRONT_LEFT_POINT_COUNT);
}

/* 左のセンサの値 → 左の壁までの距離 [mm] */
float wall_dist_left_mm(int32_t value)
{
  return value_to_mm(value, LEFT_MM, LEFT_VALUE, LEFT_POINT_COUNT);
}

/* 右のセンサの値 → 右の壁までの距離 [mm] */
float wall_dist_right_mm(int32_t value)
{
  return value_to_mm(value, RIGHT_MM, RIGHT_VALUE, RIGHT_POINT_COUNT);
}

/* 右前のセンサの値 → 前の壁までの距離 [mm] */
float wall_dist_front_right_mm(int32_t value)
{
  return value_to_mm(value, FRONT_RIGHT_MM, FRONT_RIGHT_VALUE, FRONT_RIGHT_POINT_COUNT);
}
