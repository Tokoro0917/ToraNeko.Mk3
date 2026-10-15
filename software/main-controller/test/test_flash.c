/*
 * test_flash.c
 * 迷路とゴールの位置の保存（sys/flash.c）の PC 上でのテスト。実機がなくても動く
 *
 * ビルドと実行（このフォルダで）:  make
 *
 * ■ しくみ
 * flash.c が使う Flash の関数（drv/flash_memory.h）を、ここで仮に作る。Flash の代わりに配列を使う。
 *   - 消すと、全部の語が 0xFFFFFFFF になる（本物と同じ）
 *   - 消していない語に書こうとしたら、数えておいて、その語を壊す（本物の Flash では、前の値と混ざった値になる）
 *   - 「消すのに失敗する」「何語か書いたところで止まる（電源が切れたときのまね）」もできる
 *
 * ■ テストの内容
 *   1. 一度も保存していないとき：迷路もゴールも読めない（今の迷路は変わらない）
 *   2. 迷路を保存して読み込むと、壁と「入ったか」が全部元どおりになる（でたらめな迷路 20 個）
 *   3. 迷路を読み込むと、「最後に壁を記録した 5 区画」を忘れる
 *   4. 壊れた迷路（中の 1 語が違う、途中で止まった、消すのに失敗した、大きさが違う）は読まない。今の迷路も変わらない
 *   5. ゴールを保存して読み込む。壊れたもの・迷路の外の値は読まない
 *   6. 迷路とゴールは別の場所：片方を保存しても、もう片方は消えない
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "drv/flash_memory.h"
#include "maze/maze_map.h"
#include "sys/flash.h"

/* 迷路を保存したときに書く語の数（目印、大きさ、区画ごとに 5 語、確かめの数） */
#define MAZE_WORD_COUNT (2 + MAZE_SIZE * MAZE_SIZE * 5 + 1)
/* ゴールを保存したときに書く語の数（目印、x、y、確かめの数） */
#define GOAL_WORD_COUNT 4

/* ---------------------------------------------------------------------------
 * テスト結果の数え方
 * ------------------------------------------------------------------------- */

static int s_fail_count = 0;

/* ok が false なら失敗として数え、message を表示する */
static void check(bool ok, const char *message)
{
  if (!ok) {
    s_fail_count = s_fail_count + 1;
    printf("  失敗: %s\n", message);
  }
}

/* ---------------------------------------------------------------------------
 * 仮の Flash（drv/flash_memory.h の関数を、配列で作る）
 * ------------------------------------------------------------------------- */

static uint32_t s_goal_words[FLASH_AREA_WORD_COUNT];  /* セクタ6 の代わり */
static uint32_t s_maze_words[FLASH_AREA_WORD_COUNT];  /* セクタ7 の代わり */

static int s_goal_erase_count = 0;  /* ゴールの場所を消した回数 */
static int s_maze_erase_count = 0;  /* 迷路の場所を消した回数 */
static int s_rewrite_count = 0;     /* 消していない語に書こうとした回数（0 でなければ、flash.c がおかしい） */
static bool s_erase_fails = false;  /* true なら、消すのに失敗する */
static int s_write_limit = -1;      /* 0 以上なら、この語数を書いたあとは書けない（電源が切れたときのまね） */
static int s_write_count = 0;       /* 書いた語の数 */

/* 仮の Flash を、一度も保存していない状態（全部消した状態）にする */
static void reset_flash(void)
{
  for (int i = 0; i < FLASH_AREA_WORD_COUNT; i++) {
    s_goal_words[i] = FLASH_ERASED_WORD;
    s_maze_words[i] = FLASH_ERASED_WORD;
  }
  s_goal_erase_count = 0;
  s_maze_erase_count = 0;
  s_rewrite_count = 0;
  s_erase_fails = false;
  s_write_limit = -1;
  s_write_count = 0;
}

/* 仮の Flash の area の index 番目の語 */
static uint32_t get_word(flash_area_t area, int index)
{
  if (area == FLASH_AREA_GOAL) {
    return s_goal_words[index];
  }
  return s_maze_words[index];
}

/* 仮の Flash の area の index 番目の語を value にする */
static void set_word(flash_area_t area, int index, uint32_t value)
{
  if (area == FLASH_AREA_GOAL) {
    s_goal_words[index] = value;
  } else {
    s_maze_words[index] = value;
  }
}

bool flash_memory_erase(flash_area_t area)
{
  if (s_erase_fails) {
    return false;
  }
  for (int i = 0; i < FLASH_AREA_WORD_COUNT; i++) {
    set_word(area, i, FLASH_ERASED_WORD);
  }
  if (area == FLASH_AREA_GOAL) {
    s_goal_erase_count = s_goal_erase_count + 1;
  } else {
    s_maze_erase_count = s_maze_erase_count + 1;
  }
  return true;
}

bool flash_memory_write_word(flash_area_t area, int index, uint32_t value)
{
  if (index < 0 || index >= FLASH_AREA_WORD_COUNT) {
    return false;
  }
  if (s_write_limit >= 0 && s_write_count >= s_write_limit) {
    return false;  /* 電源が切れた */
  }
  s_write_count = s_write_count + 1;
  if (get_word(area, index) != FLASH_ERASED_WORD) {
    s_rewrite_count = s_rewrite_count + 1;
    set_word(area, index, 0);  /* 消していない語に書くと、おかしな値になる */
    return false;
  }
  set_word(area, index, value);
  return true;
}

uint32_t flash_memory_read_word(flash_area_t area, int index)
{
  if (index < 0 || index >= FLASH_AREA_WORD_COUNT) {
    return FLASH_ERASED_WORD;
  }
  return get_word(area, index);
}

/* area の先頭から count 語のうち、消したまま（0xFFFFFFFF）でない語の数 */
static int count_written_words(flash_area_t area, int count)
{
  int written = 0;
  for (int i = 0; i < count; i++) {
    if (get_word(area, i) != FLASH_ERASED_WORD) {
      written = written + 1;
    }
  }
  return written;
}

/* area の先頭の count - 1 語を足し直して、最後の語（確かめの数）に入れる。
 * 中の値を書き換えたあとに呼ぶと、「確かめの数は合っているが、中身がおかしい」保存になる */
static void fix_check_number(flash_area_t area, int count)
{
  uint32_t sum = 0;
  for (int i = 0; i < count - 1; i++) {
    sum = sum + get_word(area, i);
  }
  set_word(area, count - 1, sum);
}

/* ---------------------------------------------------------------------------
 * 迷路を作る・覚えておく・比べる
 * ------------------------------------------------------------------------- */

static unsigned int s_random_seed = 1;

/* 0〜(limit - 1) のでたらめな数（いつも同じ並びになるように、自分で計算する） */
static int random_below(int limit)
{
  s_random_seed = s_random_seed * 1103515245u + 12345u;
  int value = s_random_seed / 65536u;
  return value % limit;
}

/* でたらめな壁の状態 */
static wall_state_t random_wall_state(void)
{
  int value = random_below(3);
  if (value == 0) {
    return WALL_UNKNOWN;
  }
  if (value == 1) {
    return WALL_NONE;
  }
  return WALL_EXIST;
}

/* 全部の区画の壁と「入ったか」を、でたらめにした迷路を作る（外周とスタートの東は、決まりどおり壁になる） */
static void make_random_maze(void)
{
  maze_map_init();
  for (int y = 0; y < MAZE_SIZE; y++) {
    for (int x = 0; x < MAZE_SIZE; x++) {
      maze_map_set_wall(x, y, DIR_NORTH, random_wall_state());
      maze_map_set_wall(x, y, DIR_EAST, random_wall_state());
      maze_map_set_wall(x, y, DIR_SOUTH, random_wall_state());
      maze_map_set_wall(x, y, DIR_WEST, random_wall_state());
      maze_map_set_visited(x, y, random_below(2) == 1);
    }
  }
}

/* 覚えておいた迷路 */
static wall_state_t s_saved_walls[MAZE_SIZE][MAZE_SIZE][4];
static bool s_saved_visited[MAZE_SIZE][MAZE_SIZE];

/* 今の迷路を覚えておく */
static void remember_maze(void)
{
  for (int y = 0; y < MAZE_SIZE; y++) {
    for (int x = 0; x < MAZE_SIZE; x++) {
      s_saved_walls[x][y][DIR_NORTH] = maze_map_get_wall(x, y, DIR_NORTH);
      s_saved_walls[x][y][DIR_EAST] = maze_map_get_wall(x, y, DIR_EAST);
      s_saved_walls[x][y][DIR_SOUTH] = maze_map_get_wall(x, y, DIR_SOUTH);
      s_saved_walls[x][y][DIR_WEST] = maze_map_get_wall(x, y, DIR_WEST);
      s_saved_visited[x][y] = maze_map_is_visited(x, y);
    }
  }
}

/* 今の迷路が、覚えておいた迷路と同じなら true */
static bool maze_is_same_as_remembered(void)
{
  for (int y = 0; y < MAZE_SIZE; y++) {
    for (int x = 0; x < MAZE_SIZE; x++) {
      if (s_saved_walls[x][y][DIR_NORTH] != maze_map_get_wall(x, y, DIR_NORTH)) {
        return false;
      }
      if (s_saved_walls[x][y][DIR_EAST] != maze_map_get_wall(x, y, DIR_EAST)) {
        return false;
      }
      if (s_saved_walls[x][y][DIR_SOUTH] != maze_map_get_wall(x, y, DIR_SOUTH)) {
        return false;
      }
      if (s_saved_walls[x][y][DIR_WEST] != maze_map_get_wall(x, y, DIR_WEST)) {
        return false;
      }
      if (s_saved_visited[x][y] != maze_map_is_visited(x, y)) {
        return false;
      }
    }
  }
  return true;
}

/* でたらめな迷路を作って保存し、それとは違う迷路（でたらめな別の迷路）を今の迷路にして覚えておく。
 * 読み込みに失敗したときに「今の迷路が変わらない」ことを調べるための準備 */
static void save_one_and_keep_another(void)
{
  make_random_maze();
  flash_save_maze();
  make_random_maze();
  remember_maze();
}

/* ---------------------------------------------------------------------------
 * 1. 一度も保存していないとき
 * ------------------------------------------------------------------------- */

static void test_never_saved(void)
{
  printf("[1] 一度も保存していないとき\n");
  reset_flash();
  make_random_maze();
  remember_maze();

  check(!flash_load_maze(), "迷路は読めない");
  check(maze_is_same_as_remembered(), "今の迷路は変わらない");

  int goal_x = -7;
  int goal_y = -7;
  check(!flash_load_goal(&goal_x, &goal_y), "ゴールは読めない");
  check(goal_x == -7 && goal_y == -7, "ゴールの x・y には何も入れない");
}

/* ---------------------------------------------------------------------------
 * 2. 迷路を保存して読み込む
 * ------------------------------------------------------------------------- */

static void test_maze_round_trip(void)
{
  printf("[2] 迷路を保存して読み込む（でたらめな迷路 20 個）\n");
  reset_flash();

  int failed_count = 0;
  for (int i = 0; i < 20; i++) {
    make_random_maze();
    remember_maze();
    flash_save_maze();

    maze_map_init();  /* 今の迷路を空にしてから読み込む */
    bool loaded = flash_load_maze();
    if (!loaded || !maze_is_same_as_remembered()) {
      failed_count = failed_count + 1;
    }
  }
  check(failed_count == 0, "読み込んだ迷路が、保存した迷路と同じ");
  check(s_maze_erase_count == 20, "保存するたびに、迷路の場所を 1 回消す");
  check(s_rewrite_count == 0, "消していない語には書かない");
  check(count_written_words(FLASH_AREA_MAZE, FLASH_AREA_WORD_COUNT) == MAZE_WORD_COUNT,
        "書く語の数は 5123（目印、大きさ、32×32 区画 × 5、確かめの数）");
  check(s_maze_words[0] != FLASH_ERASED_WORD && s_maze_words[MAZE_WORD_COUNT - 1] != FLASH_ERASED_WORD,
        "先頭から続けて書く");
  check(s_goal_erase_count == 0 && count_written_words(FLASH_AREA_GOAL, FLASH_AREA_WORD_COUNT) == 0,
        "ゴールの場所には触らない");
  printf("  迷路 1 つで %d 語（%d バイト）を使う\n", MAZE_WORD_COUNT, MAZE_WORD_COUNT * 4);
}

/* ---------------------------------------------------------------------------
 * 3. 読み込むと、最後に壁を記録した 5 区画を忘れる
 * ------------------------------------------------------------------------- */

static void test_load_forgets_recent_cells(void)
{
  printf("[3] 迷路を読み込むと、最後に壁を記録した 5 区画を忘れる\n");
  reset_flash();
  make_random_maze();
  flash_save_maze();

  /* 空の迷路で 5 区画ぶん壁を記録してから（戻せる区画ができる）、保存した迷路を読み込む */
  maze_map_init();
  for (int y = 1; y <= 5; y++) {
    maze_map_record_walls(0, y, DIR_NORTH, false, true, false);
  }
  check(flash_load_maze(), "読み込める");
  remember_maze();

  /* 忘れていなければ、読み込んだ迷路の (0,1)〜(0,5) が、記録する前（空の迷路）の状態に戻ってしまう */
  maze_map_undo_recent_cells();
  check(maze_is_same_as_remembered(), "読み込んだあとに戻そうとしても、何も戻らない");
}

/* ---------------------------------------------------------------------------
 * 4. 壊れた迷路は読まない
 * ------------------------------------------------------------------------- */

static void test_broken_maze(void)
{
  printf("[4] 壊れた迷路は読まない\n");

  /* 中の 1 語（区画の壁）が違う */
  reset_flash();
  save_one_and_keep_another();
  s_maze_words[2000] = s_maze_words[2000] + 1;
  check(!flash_load_maze(), "中の 1 語が違うと読まない");
  check(maze_is_same_as_remembered(), "中の 1 語が違うとき、今の迷路は変わらない（途中まで入れない）");

  /* 最後の区画の「入ったか」が違う（最後まで調べてから入れるか） */
  reset_flash();
  save_one_and_keep_another();
  s_maze_words[MAZE_WORD_COUNT - 2] = s_maze_words[MAZE_WORD_COUNT - 2] + 1;
  check(!flash_load_maze(), "最後の区画が違うと読まない");
  check(maze_is_same_as_remembered(), "最後の区画が違うとき、今の迷路は変わらない（途中まで入れない）");

  /* 確かめの数が違う */
  reset_flash();
  save_one_and_keep_another();
  s_maze_words[MAZE_WORD_COUNT - 1] = s_maze_words[MAZE_WORD_COUNT - 1] + 1;
  check(!flash_load_maze(), "確かめの数が違うと読まない");

  /* 書いている途中で止まった（電源が切れた） */
  reset_flash();
  s_write_limit = 3000;
  save_one_and_keep_another();
  check(!flash_load_maze(), "途中までしか書けなかった迷路は読まない");
  check(maze_is_same_as_remembered(), "途中までしか書けなかったとき、今の迷路は変わらない");

  /* 前に保存した迷路があるところで、消すのに失敗した */
  reset_flash();
  make_random_maze();
  flash_save_maze();
  s_erase_fails = true;
  save_one_and_keep_another();
  check(!flash_load_maze(), "消すのに失敗して、前の迷路の上に書いてしまったものは読まない");
  check(maze_is_same_as_remembered(), "消すのに失敗したとき、今の迷路は変わらない");

  /* 大きさが違う（確かめの数は合わせておく） */
  reset_flash();
  save_one_and_keep_another();
  s_maze_words[1] = 16;
  fix_check_number(FLASH_AREA_MAZE, MAZE_WORD_COUNT);
  check(!flash_load_maze(), "大きさが違う迷路は読まない");
  check(maze_is_same_as_remembered(), "大きさが違うとき、今の迷路は変わらない");

  /* 目印が違う（確かめの数は合わせておく） */
  reset_flash();
  save_one_and_keep_another();
  s_maze_words[0] = 12345;
  fix_check_number(FLASH_AREA_MAZE, MAZE_WORD_COUNT);
  check(!flash_load_maze(), "目印が違うものは読まない");
}

/* ---------------------------------------------------------------------------
 * 5. ゴール
 * ------------------------------------------------------------------------- */

static void test_goal(void)
{
  printf("[5] ゴールを保存して読み込む\n");
  reset_flash();
  int goal_x = 0;
  int goal_y = 0;

  flash_save_goal(20, 3);
  check(flash_load_goal(&goal_x, &goal_y), "保存したゴールを読める");
  check(goal_x == 20 && goal_y == 3, "読んだゴールは (20, 3)");
  check(count_written_words(FLASH_AREA_GOAL, FLASH_AREA_WORD_COUNT) == GOAL_WORD_COUNT,
        "書く語の数は 4（目印、x、y、確かめの数）");

  flash_save_goal(5, 30);
  check(flash_load_goal(&goal_x, &goal_y) && goal_x == 5 && goal_y == 30, "保存し直すと、新しいゴール (5, 30) を読める");
  check(s_goal_erase_count == 2 && s_rewrite_count == 0, "保存するたびに、ゴールの場所を消してから書く");
  check(s_maze_erase_count == 0 && count_written_words(FLASH_AREA_MAZE, FLASH_AREA_WORD_COUNT) == 0,
        "迷路の場所には触らない");

  /* 壊れたゴール */
  s_goal_words[1] = 21;
  goal_x = -7;
  goal_y = -7;
  check(!flash_load_goal(&goal_x, &goal_y), "x が違う（確かめの数が合わない）ゴールは読まない");
  check(goal_x == -7 && goal_y == -7, "読めないときは、ゴールの x・y に何も入れない");

  /* 迷路の外の値（確かめの数は合わせておく） */
  flash_save_goal(5, 30);
  s_goal_words[1] = 40;
  fix_check_number(FLASH_AREA_GOAL, GOAL_WORD_COUNT);
  check(!flash_load_goal(&goal_x, &goal_y), "迷路の外の値は読まない");

  /* 書いている途中で止まった */
  reset_flash();
  s_write_limit = 2;
  flash_save_goal(8, 9);
  check(!flash_load_goal(&goal_x, &goal_y), "途中までしか書けなかったゴールは読まない");
}

/* ---------------------------------------------------------------------------
 * 6. 迷路とゴールは別の場所
 * ------------------------------------------------------------------------- */

static void test_separate_areas(void)
{
  printf("[6] 迷路とゴールは別の場所\n");
  reset_flash();

  make_random_maze();
  remember_maze();
  flash_save_maze();
  flash_save_goal(12, 7);

  /* ゴールを保存したあとも、迷路を読める */
  maze_map_init();
  check(flash_load_maze() && maze_is_same_as_remembered(), "ゴールを保存しても、迷路は消えない");

  /* 迷路を保存し直したあとも、ゴールを読める */
  make_random_maze();
  flash_save_maze();
  int goal_x = 0;
  int goal_y = 0;
  check(flash_load_goal(&goal_x, &goal_y) && goal_x == 12 && goal_y == 7, "迷路を保存しても、ゴールは消えない");
}

/* ---------------------------------------------------------------------------
 * main
 * ------------------------------------------------------------------------- */

int main(void)
{
  test_never_saved();
  test_maze_round_trip();
  test_load_forgets_recent_cells();
  test_broken_maze();
  test_goal();
  test_separate_areas();

  if (s_fail_count == 0) {
    printf("すべて成功\n");
    return 0;
  }
  printf("失敗: %d 件\n", s_fail_count);
  return 1;
}
