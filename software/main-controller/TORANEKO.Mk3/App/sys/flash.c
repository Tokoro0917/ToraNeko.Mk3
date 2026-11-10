/*
 * flash.c
 * 迷路とゴールの位置、会場ごとの感度の倍率を Flash に保存する・読み込む（MODES.md の 5）
 *
 * Flash を消す・書く・読むのは drv/flash_memory.c。ここは「何をどの順番で書くか」を決める。
 * HAL を使わないので、PC のテスト（test/test_flash.c）でも、そのまま動く。
 *
 * ■ 書く形
 * 保存場所の先頭から、4 バイトの数（語）を順に書く。読むときは、書いたときと同じ順番で読む。
 *
 *   ゴールと感度（FLASH_AREA_GOAL）  迷路（FLASH_AREA_MAZE）
 *   [0] 目印 GOAL_MARK              [0] 目印 MAZE_MARK
 *   [1] ゴールの x                   [1] 迷路の大きさ（MAZE_SIZE）
 *   [2] ゴールの y                   [2]〜 区画ごとに 5 語：北・東・南・西の壁の状態、入ったか（1 か 0）
 *   [3] 左前の感度の倍率 × 10000           区画の順番は (0,0)、(1,0)、…、(31,0)、(0,1)、…、(31,31)
 *   [4] 左の倍率 × 10000            [5122] 確かめの数
 *   [5] 右の倍率 × 10000
 *   [6] 右前の倍率 × 10000
 *   [7] 確かめの数
 *
 * - ゴールと感度の倍率は同じ保存場所にある（Flash の空きが、この 2 つのセクタしかないため。2026-11-10 に倍率を足した）。
 *   消すときはセクタごとなので、片方を保存するときは、もう片方を先に読んでおいて、一緒に書き直す
 *   （ゴールを保存しても倍率は消えず、倍率を保存してもゴールは消えない）。
 *   ゴールを入れたことがなく、倍率だけを保存したときは、ゴールの x・y に NO_GOAL_NUMBER を書く（読むと「ゴールなし」になる）
 * - 2026-11-10 より前の形（目印 GOAL_MARK_OLD。[0] 目印、[1] x、[2] y、[3] 確かめの数）も読める。そのときの倍率は 1
 *
 * - 目印：保存したことがあるかを見分ける数。一度も保存していなければ、消したままの 0xFFFFFFFF なので目印と違う
 * - 確かめの数：それより前に書いた数を全部足したもの。書いている途中で電源が切れたときなどは、
 *   読んだ数を足し直しても合わないので、壊れているとわかる
 *   （足していくと uint32_t に入りきらなくなるが、uint32_t の足し算は入りきらない分を捨てる決まりなので問題ない）
 * - 迷路を読むときは、まず全部を読んで確かめの数が合うかを調べ、合ったときだけ迷路に入れる
 *   （途中まで入れて、おかしな迷路になるのを防ぐ）
 * - 書く形を変えたときは、目印の数も変える（前の形で保存したものを、間違えて読まないように）
 *
 * ■ 保存しているあいだの表示（MODES.md の 3）
 * 外側の 4 個の LED（LED1・2・6・7）を点けておき、終わったら消す。Flash を消しているあいだは CPU が止まって
 * LED を動かせないので、始める前に点けておく（「今は電源を切らないで」の合図）。
 */
#include "sys/flash.h"

#include <stdint.h>

#include "ctrl/wall_ctrl.h"
#include "drv/flash_memory.h"
#include "drv/led.h"
#include "maze/maze_map.h"

#define GOAL_MARK_OLD 0x474F4131  /* 2026-11-10 より前の形（ゴールだけ）の目印（"GOA1" の文字コード） */
#define GOAL_MARK 0x474F4132      /* ゴールと感度の倍率を保存した目印（"GOA2" の文字コード。ほかと重ならない数なら何でもよい） */
#define NO_GOAL_NUMBER 0xFFFFFFFF /* ゴールを入れたことがないときに、ゴールの x・y に書く数（迷路の外なので、読むと使わない） */
#define SCALE_UNIT 10000.0f       /* 感度の倍率は、10000 倍して整数で書く（1.0 なら 10000。小数 4 けたまで残る） */
#define MAZE_MARK 0x4D5A4531  /* 迷路を保存した目印（"MZE1" の文字コード） */

#define NUMBERS_PER_CELL 5  /* 1 区画で書く数の個数（4 枚の壁と、入ったか） */

/* 書いている・読んでいる保存場所、次に書く・読む語の番号、それまでの数の合計 */
static flash_area_t s_area = FLASH_AREA_GOAL;
static int s_index = 0;
static uint32_t s_sum = 0;
static bool s_write_ok = true;  /* ここまで全部うまく書けたか */

/* ---------------------------------------------------------------------------
 * 保存しているあいだの表示
 * ------------------------------------------------------------------------- */

/* 保存しているあいだの表示を出す（on が true）・消す（false）。
 * 出すときは、それまでの表示（ゴールの値など）を全部消してから、外側の 4 個（LED1・2・6・7）だけを点ける */
static void show_saving(bool on)
{
  led_all_off();
  if (on) {
    led_set(1, true);
    led_set(2, true);
    led_set(6, true);
    led_set(7, true);
  }
}

/* ---------------------------------------------------------------------------
 * 順に書く・読む
 * ------------------------------------------------------------------------- */

/* 保存場所 area を消して、先頭から書き始める */
static void start_writing(flash_area_t area)
{
  s_area = area;
  s_index = 0;
  s_sum = 0;
  s_write_ok = flash_memory_erase(area);
}

/* 次の語に数を 1 つ書いて、合計に足す */
static void write_number(uint32_t number)
{
  if (!flash_memory_write_word(s_area, s_index, number)) {
    s_write_ok = false;
  }
  s_sum = s_sum + number;
  s_index = s_index + 1;
}

/* 最後に、確かめの数（それまでに書いた数の合計）を書く。全部うまく書けたら true */
static bool finish_writing(void)
{
  uint32_t check_number = s_sum;
  write_number(check_number);
  return s_write_ok;
}

/* 保存場所 area の先頭から読み始める */
static void start_reading(flash_area_t area)
{
  s_area = area;
  s_index = 0;
  s_sum = 0;
}

/* 次の語の数を 1 つ読んで、合計に足す */
static uint32_t read_number(void)
{
  uint32_t number = flash_memory_read_word(s_area, s_index);
  s_sum = s_sum + number;
  s_index = s_index + 1;
  return number;
}

/* 確かめの数を読んで、それまでに読んだ数の合計と同じなら true */
static bool check_number_matches(void)
{
  uint32_t sum_so_far = s_sum;
  uint32_t check_number = read_number();
  return check_number == sum_so_far;
}

/* ---------------------------------------------------------------------------
 * 迷路
 * ------------------------------------------------------------------------- */

/* 区画 (x, y) の 4 枚の壁の状態と、入ったかどうかを書く */
static void write_cell(int x, int y)
{
  write_number(maze_map_get_wall(x, y, DIR_NORTH));
  write_number(maze_map_get_wall(x, y, DIR_EAST));
  write_number(maze_map_get_wall(x, y, DIR_SOUTH));
  write_number(maze_map_get_wall(x, y, DIR_WEST));
  if (maze_map_is_visited(x, y)) {
    write_number(1);
  } else {
    write_number(0);
  }
}

/* 今の迷路（maze_map）を保存する。消してから書くので、前に保存した迷路は消える。
 * 消しているあいだ（約 1〜2 秒）は CPU が止まるので、モーターを止めているときだけ呼ぶ */
void flash_save_maze(void)
{
  show_saving(true);
  start_writing(FLASH_AREA_MAZE);
  write_number(MAZE_MARK);
  write_number(MAZE_SIZE);
  for (int y = 0; y < MAZE_SIZE; y++) {
    for (int x = 0; x < MAZE_SIZE; x++) {
      write_cell(x, y);
    }
  }
  finish_writing();
  show_saving(false);
}

/* 保存した迷路が使えるか（目印、大きさ、確かめの数が合うか）を調べる。読むだけで、迷路は変えない */
static bool saved_maze_is_good(void)
{
  start_reading(FLASH_AREA_MAZE);
  if (read_number() != MAZE_MARK) {
    return false;  /* 保存したことがない */
  }
  if (read_number() != MAZE_SIZE) {
    return false;  /* 大きさの違う迷路 */
  }
  for (int i = 0; i < MAZE_SIZE * MAZE_SIZE * NUMBERS_PER_CELL; i++) {
    read_number();
  }
  return check_number_matches();
}

/* 保存した数を、壁の状態に直す */
static wall_state_t to_wall_state(uint32_t number)
{
  if (number == WALL_NONE) {
    return WALL_NONE;
  }
  if (number == WALL_EXIST) {
    return WALL_EXIST;
  }
  return WALL_UNKNOWN;
}

/* 区画 (x, y) の 4 枚の壁の状態と、入ったかどうかを読んで、迷路に入れる */
static void read_cell(int x, int y)
{
  maze_map_set_wall(x, y, DIR_NORTH, to_wall_state(read_number()));
  maze_map_set_wall(x, y, DIR_EAST, to_wall_state(read_number()));
  maze_map_set_wall(x, y, DIR_SOUTH, to_wall_state(read_number()));
  maze_map_set_wall(x, y, DIR_WEST, to_wall_state(read_number()));
  maze_map_set_visited(x, y, read_number() == 1);
}

/* 保存した迷路を、今の迷路（maze_map）に入れる。保存した迷路がない・壊れているときは、
 * 今の迷路を変えずに false */
bool flash_load_maze(void)
{
  if (!saved_maze_is_good()) {
    return false;
  }

  start_reading(FLASH_AREA_MAZE);
  read_number();  /* 目印（調べ済み） */
  read_number();  /* 迷路の大きさ（調べ済み） */
  for (int y = 0; y < MAZE_SIZE; y++) {
    for (int x = 0; x < MAZE_SIZE; x++) {
      read_cell(x, y);
    }
  }

  /* 覚えていた「最後に壁を記録した 5 区画」は、読み込む前の迷路のものなので忘れる */
  maze_map_clear_recent_cells();
  return true;
}

/* ---------------------------------------------------------------------------
 * ゴール
 * ------------------------------------------------------------------------- */

/* ゴールと感度の保存場所から読んだもの */
typedef struct {
  bool is_good;          /* 保存してあって、壊れていないか */
  uint32_t goal_x;       /* ゴールの x（入れたことがなければ NO_GOAL_NUMBER） */
  uint32_t goal_y;       /* ゴールの y（同じ） */
  sensor_scale_t scale;  /* 感度の倍率（前の形で保存したものは 1） */
} goal_area_t;

/* 感度の倍率が全部 1（直さない）のもの */
static sensor_scale_t scale_of_one(void)
{
  sensor_scale_t scale;
  scale.front_left = 1.0f;
  scale.left = 1.0f;
  scale.right = 1.0f;
  scale.front_right = 1.0f;
  return scale;
}

/* 感度の倍率を、保存する整数に直す（10000 倍して、四捨五入する） */
static uint32_t scale_to_number(float scale)
{
  uint32_t number = scale * SCALE_UNIT + 0.5f;
  return number;
}

/* 保存した整数を、感度の倍率に直す */
static float number_to_scale(uint32_t number)
{
  return number / SCALE_UNIT;
}

/* ゴールと感度の保存場所を読む。今の形（GOAL_MARK）と前の形（GOAL_MARK_OLD）のどちらも読める */
static goal_area_t read_goal_area(void)
{
  goal_area_t area;
  area.is_good = false;
  area.goal_x = NO_GOAL_NUMBER;
  area.goal_y = NO_GOAL_NUMBER;
  area.scale = scale_of_one();

  start_reading(FLASH_AREA_GOAL);
  uint32_t mark = read_number();
  if (mark == GOAL_MARK) {
    area.goal_x = read_number();
    area.goal_y = read_number();
    area.scale.front_left = number_to_scale(read_number());
    area.scale.left = number_to_scale(read_number());
    area.scale.right = number_to_scale(read_number());
    area.scale.front_right = number_to_scale(read_number());
    area.is_good = check_number_matches();
  } else if (mark == GOAL_MARK_OLD) {
    area.goal_x = read_number();
    area.goal_y = read_number();
    area.is_good = check_number_matches();  /* 前の形には倍率がないので、1 のまま */
  }
  if (!area.is_good) {
    /* 保存したことがない・壊れている：読んだ数は使わない */
    area.goal_x = NO_GOAL_NUMBER;
    area.goal_y = NO_GOAL_NUMBER;
    area.scale = scale_of_one();
  }
  return area;
}

/* ゴールと感度の保存場所を消して、今の形で書く */
static void write_goal_area(uint32_t goal_x, uint32_t goal_y, sensor_scale_t scale)
{
  show_saving(true);
  start_writing(FLASH_AREA_GOAL);
  write_number(GOAL_MARK);
  write_number(goal_x);
  write_number(goal_y);
  write_number(scale_to_number(scale.front_left));
  write_number(scale_to_number(scale.left));
  write_number(scale_to_number(scale.right));
  write_number(scale_to_number(scale.front_right));
  finish_writing();
  show_saving(false);
}

/* ゴールの位置（左下の区画）を保存する。保存してある感度の倍率は、先に読んで一緒に書き直すので消えない（迷路も消えない）。
 * 消しているあいだ（約 1〜2 秒）は CPU が止まるので、モーターを止めているときだけ呼ぶ */
void flash_save_goal(int goal_x, int goal_y)
{
  goal_area_t saved = read_goal_area();
  write_goal_area(goal_x, goal_y, saved.scale);
}

/* 保存したゴールの位置を goal_x、goal_y に入れて true。保存したことがない・壊れているときは、
 * 何も入れずに false。使える位置かどうかは、呼ぶ側（sys/goal.c）が調べる */
bool flash_load_goal(int *goal_x, int *goal_y)
{
  goal_area_t saved = read_goal_area();
  if (!saved.is_good) {
    return false;  /* 保存したことがない・壊れている */
  }
  if (saved.goal_x >= MAZE_SIZE || saved.goal_y >= MAZE_SIZE) {
    return false;  /* 迷路の外、またはゴールを入れたことがない（int に入れる前に調べる） */
  }

  *goal_x = saved.goal_x;
  *goal_y = saved.goal_y;
  return true;
}

/* 会場ごとの感度の倍率を保存する。保存してあるゴールは、先に読んで一緒に書き直すので消えない。
 * 消しているあいだ（約 1〜2 秒）は CPU が止まるので、モーターを止めているときだけ呼ぶ */
void flash_save_sensor_scale(sensor_scale_t scale)
{
  goal_area_t saved = read_goal_area();
  write_goal_area(saved.goal_x, saved.goal_y, scale);
}

/* 保存した感度の倍率。保存したことがない・壊れているときや、前の形（倍率なし）のときは、全部 1 */
sensor_scale_t flash_load_sensor_scale(void)
{
  goal_area_t saved = read_goal_area();
  return saved.scale;
}
