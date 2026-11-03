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
 * ■ 今の表
 *   右（R）・左（L）：2026-11-03 に作者が測った値（センサを付け直したあと）。左右に壁がある区画で、機体の横に壁の板（6mm）を
 *     0〜6 枚はさんで反対側へずらし、**板を抜いてから**測った（板が機体にくっついていると、もれた光が跳ね返って値が上がるため）。
 *     22mm（7 枚）は測っていないので、28mm より近いときは 28mm を返す。42mm は 46mm と 40mm のあいだ（右 約 950、左 約 1490）になり、
 *     区画のまん中で測った値（右 863〜961、左 1426〜1495）と合う
 *   左前・右前：2026-11-03 に作者が測った値。お尻を後ろの壁に付けたところ（52mm）から、後ろに板をはさんで近づけた
 *     （46・40・34mm、板は抜いて測る。左右の壁なし）・板 1 枚ずつ遠ざけた（58〜100mm。**左右の壁あり**で測り直した値）。
 *     探索で前の壁を見るときは、たいてい左右にも壁があるので、遠いほうは左右の壁ありの値にした
 *     （左右の壁なしでは 58mm で 2361・2612 と、ありより大きかった。近いほうは 3000 を超えていて、差は小さい）。
 *     42mm は 約 3325・3350 になり、区画のまん中で測った値（3334・3392）と合う。
 *     区画の入口で前の壁を見る 87mm は、82mm と 88mm のあいだ（左前 約 763、右前 約 1133）。
 *     100mm より遠い点は測っていないので、それより小さい値は 100mm を返す
 */
#include "ctrl/wall_dist.h"

/* ---- 左前（FL）の表（2026-11-03 に測った。52mm まで左右の壁なし、58mm から左右の壁あり） ---- */
#define FRONT_LEFT_POINT_COUNT 12
static const float FRONT_LEFT_MM[FRONT_LEFT_POINT_COUNT] = {34, 40, 46, 52, 58, 64, 70, 76, 82, 88, 94, 100};
static const int32_t FRONT_LEFT_VALUE[FRONT_LEFT_POINT_COUNT] = {3460, 3367, 3240, 2941, 1845, 1417, 1084, 954, 807, 755, 660, 576};

/* ---- 左（L）の表（2026-11-03 に測った） ---- */
#define LEFT_POINT_COUNT 7
static const float LEFT_MM[LEFT_POINT_COUNT] = {28, 34, 40, 46, 52, 58, 64};
static const int32_t LEFT_VALUE[LEFT_POINT_COUNT] = {2560, 2020, 1600, 1270, 1070, 890, 757};

/* ---- 右（R）の表（2026-11-03 に測った） ---- */
#define RIGHT_POINT_COUNT 7
static const float RIGHT_MM[RIGHT_POINT_COUNT] = {28, 34, 40, 46, 52, 58, 64};
static const int32_t RIGHT_VALUE[RIGHT_POINT_COUNT] = {1775, 1255, 975, 905, 645, 530, 362};

/* ---- 右前（FR）の表（2026-11-03 に測った。52mm まで左右の壁なし、58mm から左右の壁あり） ---- */
#define FRONT_RIGHT_POINT_COUNT 12
static const float FRONT_RIGHT_MM[FRONT_RIGHT_POINT_COUNT] = {34, 40, 46, 52, 58, 64, 70, 76, 82, 88, 94, 100};
static const int32_t FRONT_RIGHT_VALUE[FRONT_RIGHT_POINT_COUNT] = {3490, 3398, 3287, 3138, 2106, 1782, 1533, 1303, 1159, 1130, 1012, 920};

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
