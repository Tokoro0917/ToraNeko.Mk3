/*
 * side_edge.c
 * 横のセンサで、壁の終わり・柱の端（値が下がるところ）を見つけ、そこから区画の境目の位置を求める（2026-11-04）。
 * KOGUMA の docs/pillar_cut_design.md と同じ考え方。壁切れ（ctrl/run.c）が、見つけた境目で前後の位置を直す。
 *
 * ■ なぜ「しきい値」ではなく「山の半分」で見るか
 * 横のセンサは、区画の境目の柱の前を通るたびに値が山になる（柱だけのとき）。壁が終わるところでは、高い値から下がる。
 * 柱だけの山は低い（モード5 No.4〜6 で、右 320〜384、左 544〜564。何もないときは右 74〜150、左 113〜118）ので、
 * 「壁あり」のしきい値（右 400・左 650）には届かない。そこで、直前のいちばん高い値と、何もないときの値のまん中
 * （「半分」）を切ったところを、下がる端とする。山の高さが変わっても（壁との距離、明るさ）、半分を切る位置は変わりにくい。
 * 速さ（300〜1000mm/s）・左右でも、半分を切る位置はほとんど変わらなかった（モード5 No.4〜6 のログ）。
 *
 * ■ 流れ（side_edge_update() を制御周期ごとに呼ぶ。ctrl/control.c）
 *   1. 1mm 進むごとに、左右の値と、そのときの走った距離を覚える（いちばん新しい EDGE_HISTORY_COUNT 個）
 *   2. 覚えた中のいちばん高い値 H と、何もないときの値 open のまん中を「半分」mid とする
 *   3. 前の値が mid 以上で、今の値が mid より下なら、そのあいだ（直線でつないだところ）を下がる端とする。
 *      ただし、H が open より EDGE_MIN_RISE 以上高いとき（壁か柱があったとき）だけ
 *   4. H が EDGE_PILLAR_MAX_* より低ければ「柱だけ」、高ければ「壁の終わり」とする（柱の山は、壁の半分くらいの高さしかない。
 *      山の幅は 40〜70mm と広く、長さでは見分けられなかった）
 *   5. 区画の境目 ＝ 下がる端 ＋ 種類ごとの距離（EDGE_*_TO_BOUNDARY_MM。横のセンサは斜め前を見るので、境目より手前で下がる）
 *   6. 一度見つけたら、値が mid ＋ EDGE_REARM_MARGIN より上がるまで、次を見つけない（1 つの柱で 2 回見つけないように）
 * 直進でないとき（ターン・その場旋回）は、覚えた値を捨てる（回るとセンサが壁をなめて、端でない変化が出る）。
 * 覚え直してから EDGE_MIN_FILLED mm 進むまでは見つけないので、ターンのすぐあとも見ない。
 *
 * ■ 割り込みと、メインの処理
 * side_edge_update() は制御周期の割り込みから、side_edge_get_*() はメインの処理（ctrl/run.c）から呼ぶ。
 * 見つけた端の記録（数・距離・種類）は、割り込みが書く。メインの処理は、数を読んでから距離・種類を読み、
 * もう一度数を読んで、変わっていなければ使う（ctrl/run.c。読んでいるあいだに書き換わっていないかを確かめるため）。
 */
#include "ctrl/side_edge.h"

/* ---- 値（2026-11-04。モード5 No.4〜6 の 6 回のログを、PC で、このファイルと同じ計算に読ませて決めた。
 *      test/test_wall_ctrl.c の --replay） ---- */
#define EDGE_HISTORY_COUNT   16      /* 覚えておく値の数（1mm ごとに 1 つ。いちばん高い値を、直前 15mm から探す） */
#define EDGE_OPEN_LEFT       115     /* 左の、何もないときの値（モード5 No.4〜6 で 113〜118） */
#define EDGE_OPEN_RIGHT      100     /* 右の、何もないときの値（同じく 74〜150。場所でちがうので、あいだをとった） */
#define EDGE_MIN_RISE        100     /* 何もないときより、これだけ高い山があったときだけ、端とする */
#define EDGE_REARM_MARGIN    30      /* 端を見つけたあと、mid よりこれだけ上がったら、次の端を見つけられるようにする */
#define EDGE_MIN_FILLED      6       /* 覚え直してから、これだけ覚えたら（6mm 進んだら）見つけ始める（スタート区画の壁の終わりは、
                                        走り始めて 15mm くらいで来るので、15mm 待つと間に合わない） */
#define EDGE_PILLAR_MAX_LEFT  900    /* 左の山がこれより低ければ柱だけ（柱 552〜562、壁 1376〜1414） */
#define EDGE_PILLAR_MAX_RIGHT 550    /* 右の山がこれより低ければ柱だけ（柱 317〜366、壁 804〜829） */
/* 端から、区画の境目まで [mm]。モード5 No.4〜6 の 6 回（300・800・1000mm/s、左右）で：
 *   壁の終わり：y=180 の境目で 33.9〜35.5mm（平均 34.8）。スタート区画の出口（y=90）だけは 38.3〜41.2mm と 5mm ずれた
 *              （速さ・左右によらないので、置き方かスタート区画の壁の形のため。ctrl/run.c は、スタート区画の出口の端を使わない）
 *   柱だけ    ：y=180 で 29.3〜32.9mm（平均 30.9）、y=270 で 27.2〜30.2mm（平均 28.9）。あわせた平均 29.9 */
#define EDGE_WALL_END_TO_BOUNDARY_MM 34.8f
#define EDGE_PILLAR_TO_BOUNDARY_MM   29.9f

/* 片側（左か右）のセンサの、覚えた値と、見つけた端の記録 */
typedef struct {
  int32_t values[EDGE_HISTORY_COUNT];  /* 1mm ごとに覚えた値 */
  int next_index;                      /* 次に書く場所（端まで来たら 0 に戻す） */
  int filled;                          /* 覚えた数（EDGE_HISTORY_COUNT まで） */
  int32_t last_value;                  /* 1 つ前に覚えた値 */
  float last_distance_mm;              /* 1 つ前に覚えたときの、走った距離 */
  bool armed;                          /* 次の端を見つけてよいか */
  volatile int count;                  /* 見つけた数 */
  volatile float edge_distance_mm;     /* いちばん新しい端の、走った距離 */
  volatile bool edge_is_pillar;        /* いちばん新しい端が、柱だけか */
} side_t;

static side_t s_left;
static side_t s_right;
static float s_last_sample_mm = 0.0f;  /* 最後に値を覚えたときの、走った距離 */
static bool s_have_sample = false;     /* まだ 1 回も覚えていなければ false */

/* 片側の覚えた値を捨てる（見つけた端の記録は残す） */
static void clear_history(side_t *side)
{
  /* side は「片側の記録がある場所」。左右で同じ処理を使い回すために、場所を受け取って、そこを書き換える */
  side->next_index = 0;
  side->filled = 0;
  side->armed = true;
}

/* 覚えた値の中で、いちばん高い値 */
static int32_t highest_value(const side_t *side)
{
  int32_t highest = side->values[0];
  for (int i = 1; i < side->filled; i++) {
    if (side->values[i] > highest) {
      highest = side->values[i];
    }
  }
  return highest;
}

/* 片側に、1mm 進んだところの値 value（走った距離 distance_mm）を渡す。下がる端を見つけたら記録する（先頭のコメントの流れ）。
 * open_value は何もないときの値、pillar_max は柱だけとみなす山の高さの上限 */
static void process_side(side_t *side, int32_t value, float distance_mm, int32_t open_value, int32_t pillar_max)
{
  /* 1. 覚える */
  side->values[side->next_index] = value;
  side->next_index = side->next_index + 1;
  if (side->next_index >= EDGE_HISTORY_COUNT) {
    side->next_index = 0;
  }
  if (side->filled < EDGE_HISTORY_COUNT) {
    side->filled = side->filled + 1;
  }

  if (side->filled >= EDGE_MIN_FILLED) {
    /* 2. 半分 */
    int32_t highest = highest_value(side);
    float mid = (highest + open_value) / 2.0f;

    /* 3・4. 下がる端 */
    bool crossed = side->last_value >= mid && value < mid;
    if (side->armed && crossed && highest - open_value >= EDGE_MIN_RISE) {
      float ratio = (side->last_value - mid) / (side->last_value - value);  /* 前の点から、どれだけ先で切ったか（0〜1） */
      side->edge_distance_mm = side->last_distance_mm + ratio * (distance_mm - side->last_distance_mm);
      side->edge_is_pillar = highest < pillar_max;
      side->count = side->count + 1;  /* 最後に数を増やす（メインの処理は、数が変わったら新しい端とみなす） */
      side->armed = false;
    }
    /* 6. 次の端を見つけられるようにする */
    if (!side->armed && value >= mid + EDGE_REARM_MARGIN) {
      side->armed = true;
    }
  }

  side->last_value = value;
  side->last_distance_mm = distance_mm;
}

/* 最初からやり直す（覚えた値と、見つけた端の記録を消す。制御を始めるとき。ctrl/control.c） */
void side_edge_reset(void)
{
  clear_history(&s_left);
  clear_history(&s_right);
  s_left.count = 0;
  s_right.count = 0;
  s_have_sample = false;
}

/* 制御周期ごとに呼ぶ。distance_mm は今の走った距離（エンコーダ）、left_value・right_value は左右の横のセンサの値、
 * is_straight は直進中か（直進でなければ、覚えた値を捨てる） */
void side_edge_update(float distance_mm, int32_t left_value, int32_t right_value, bool is_straight)
{
  if (!is_straight) {
    clear_history(&s_left);
    clear_history(&s_right);
    s_have_sample = false;
    return;
  }
  if (s_have_sample && distance_mm < s_last_sample_mm + 1.0f) {
    return;  /* まだ 1mm 進んでいない（止まっている、下がっているときも、覚えない） */
  }
  s_have_sample = true;
  s_last_sample_mm = distance_mm;
  process_side(&s_left, left_value, distance_mm, EDGE_OPEN_LEFT, EDGE_PILLAR_MAX_LEFT);
  process_side(&s_right, right_value, distance_mm, EDGE_OPEN_RIGHT, EDGE_PILLAR_MAX_RIGHT);
}

/* is_left が true なら左、false なら右の記録 */
static const side_t *side_of(bool is_left)
{
  if (is_left) {
    return &s_left;
  }
  return &s_right;
}

int side_edge_get_count(bool is_left)
{
  return side_of(is_left)->count;
}

float side_edge_get_distance_mm(bool is_left)
{
  return side_of(is_left)->edge_distance_mm;
}

bool side_edge_is_pillar(bool is_left)
{
  return side_of(is_left)->edge_is_pillar;
}

/* いちばん新しい端から求めた、区画の境目の、走った距離 [mm]（先頭のコメントの流れの 5） */
float side_edge_get_boundary_mm(bool is_left)
{
  const side_t *side = side_of(is_left);
  if (side->edge_is_pillar) {
    return side->edge_distance_mm + EDGE_PILLAR_TO_BOUNDARY_MM;
  }
  return side->edge_distance_mm + EDGE_WALL_END_TO_BOUNDARY_MM;
}
