/*
 * test_output.c
 * 出力（sys/output.c）と走行ログ（sys/log.c）の PC 上でのテスト。実機がなくても動く
 *
 * ビルドと実行（このフォルダで）:  make
 *
 * ■ 実機の出力を確かめる（KOGUMA の test_shortpath --dump にあたる）
 *   ./test_output --dump ファイル
 *   ビューアで保存したファイル（または Tera Term のログ）を読み込み、迷路を作り直して、PC で計算した最短経路が
 *   実機の出した最短経路（DIJK_PATH）と同じかを表示する。同じなら 0、違えば 1 を返す。
 *
 * ■ しくみ
 * 出力は printf で出るので、テストのあいだだけ画面ではなく一時ファイルに書かせ（標準出力の付け替え）、
 * あとでそのファイルを 1 行ずつ読み直して調べる。
 *
 * ■ テストの内容
 *   1. 迷路：出力を読み直して作った迷路が、元の迷路と同じか（壁・見たことのある壁）。まだ見ていない壁の数
 *   2. 迷路：読み直した迷路でもう一度出力すると、まったく同じ文字になるか（--dump の確かめ方と同じ）
 *   3. 走行ログ：行数、間隔、列、値（単位を戻して四捨五入した値）
 *   4. 走行ログ：最短走行（8ms おき）で 8 秒を超えたら、1000 行で記録をやめて full=1 になる。走行は止めない
 *   5. 出力全部：迷路 → 走行ログ → #DONE の順。ログがなければ迷路と #DONE だけ
 */
/* 標準出力の付け替え（dup、dup2、fileno）は C の決まりではなく POSIX（Linux などの決まり）の関数なので、
 * それを使うと宣言する（-std=c11 のままだと見えない） */
#define _POSIX_C_SOURCE 200809L

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "config/robot_params.h"
#include "drv/uart.h"
#include "maze/dijkstra.h"
#include "maze/maze_map.h"
#include "sys/log.h"
#include "sys/output.h"

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
 * 仮の UART（PC ではすぐに書けるので、待たない）
 * ------------------------------------------------------------------------- */

void uart_wait_free(uint32_t count)
{
  (void)count;
}

/* ---------------------------------------------------------------------------
 * 出力を一時ファイルに受け取って、1 行ずつ読み直す
 * ------------------------------------------------------------------------- */

#define LINE_MAX_COUNT 1300   /* 読み直す行の数の上限（ログ 1000 行 + 迷路） */
#define LINE_LENGTH 4096      /* 1 行の長さの上限（最短経路の行は長い） */

static char s_lines[LINE_MAX_COUNT][LINE_LENGTH];
static int s_line_count = 0;

static FILE *s_capture_file = NULL;
static int s_saved_stdout = -1;

/* ここから printf の行き先を、一時ファイルにする */
static void begin_capture(void)
{
  fflush(stdout);
  s_capture_file = tmpfile();
  s_saved_stdout = dup(fileno(stdout));
  dup2(fileno(s_capture_file), fileno(stdout));
}

/* printf の行き先を画面に戻し、一時ファイルに書かれた文字を 1 行ずつ s_lines に読む（行末の \r\n は取る） */
static void end_capture(void)
{
  fflush(stdout);
  dup2(s_saved_stdout, fileno(stdout));
  close(s_saved_stdout);

  rewind(s_capture_file);
  s_line_count = 0;
  while (s_line_count < LINE_MAX_COUNT && fgets(s_lines[s_line_count], LINE_LENGTH, s_capture_file) != NULL) {
    char *line = s_lines[s_line_count];
    size_t length = strlen(line);
    while (length > 0 && (line[length - 1] == '\n' || line[length - 1] == '\r')) {
      line[length - 1] = '\0';
      length = length - 1;
    }
    s_line_count = s_line_count + 1;
  }
  fclose(s_capture_file);
}

/* text で始まる最初の行の番号。なければ -1 */
static int find_line(const char *text)
{
  for (int i = 0; i < s_line_count; i++) {
    if (strncmp(s_lines[i], text, strlen(text)) == 0) {
      return i;
    }
  }
  return -1;
}

/* ---------------------------------------------------------------------------
 * 迷路の出力を読み直す
 * ------------------------------------------------------------------------- */

/* ROW・COL・LROW・LCOL の値（線の番号 0〜MAZE_SIZE） */
static unsigned long s_row[MAZE_SIZE + 1];
static unsigned long s_col[MAZE_SIZE + 1];
static unsigned long s_lrow[MAZE_SIZE + 1];
static unsigned long s_lcol[MAZE_SIZE + 1];

/* 読み直した行から、ROW・COL・LROW・LCOL を読む。全部そろっていれば true */
static bool read_maze_lines(void)
{
  int found = 0;
  for (int i = 0; i < s_line_count; i++) {
    char name[8];
    int line;
    unsigned long value;
    if (sscanf(s_lines[i], "%7s %d %lx", name, &line, &value) != 3) {
      continue;
    }
    if (line < 0 || line > MAZE_SIZE) {
      continue;
    }
    if (strcmp(name, "ROW") == 0) {
      s_row[line] = value;
      found = found + 1;
    } else if (strcmp(name, "COL") == 0) {
      s_col[line] = value;
      found = found + 1;
    } else if (strcmp(name, "LROW") == 0) {
      s_lrow[line] = value;
      found = found + 1;
    } else if (strcmp(name, "LCOL") == 0) {
      s_lcol[line] = value;
      found = found + 1;
    }
  }
  return found == 4 * (MAZE_SIZE + 1);
}

/* value の i 番目の桁（2 進数）が 1 か。2 で i 回割ると、i 番目の桁がいちばん下の桁になるので、それが奇数か調べる */
static bool digit_is_one(unsigned long value, int i)
{
  for (int step = 0; step < i; step++) {
    value = value / 2;
  }
  if (value % 2 == 1) {
    return true;
  }
  return false;
}

/* 見たことがあるか（seen）と、壁があるか（wall）から、壁の状態を決める */
static wall_state_t state_from(bool seen, bool wall)
{
  if (!seen) {
    return WALL_UNKNOWN;
  }
  if (wall) {
    return WALL_EXIST;
  }
  return WALL_NONE;
}

/* 読んだ ROW・COL・LROW・LCOL から、迷路の地図（maze_map）を作り直す */
static void rebuild_maze(void)
{
  maze_map_init();
  for (int line = 0; line <= MAZE_SIZE; line++) {
    for (int i = 0; i < MAZE_SIZE; i++) {
      wall_state_t row_state = state_from(digit_is_one(s_lrow[line], i), digit_is_one(s_row[line], i));
      wall_state_t col_state = state_from(digit_is_one(s_lcol[line], i), digit_is_one(s_col[line], i));
      if (line < MAZE_SIZE) {
        maze_map_set_wall(i, line, DIR_SOUTH, row_state);
        maze_map_set_wall(line, i, DIR_WEST, col_state);
      } else {
        maze_map_set_wall(i, MAZE_SIZE - 1, DIR_NORTH, row_state);
        maze_map_set_wall(MAZE_SIZE - 1, i, DIR_EAST, col_state);
      }
    }
  }
}

/* ---------------------------------------------------------------------------
 * 迷路の写しと、テスト用の迷路
 * ------------------------------------------------------------------------- */

static wall_state_t s_copy[MAZE_SIZE][MAZE_SIZE][4];

/* 今の地図を写す */
static void copy_map(void)
{
  for (int x = 0; x < MAZE_SIZE; x++) {
    for (int y = 0; y < MAZE_SIZE; y++) {
      for (int i = 0; i < 4; i++) {
        s_copy[x][y][i] = maze_map_get_wall(x, y, (direction_t)i);
      }
    }
  }
}

/* 今の地図が、写しと同じか */
static bool same_as_copy(void)
{
  for (int x = 0; x < MAZE_SIZE; x++) {
    for (int y = 0; y < MAZE_SIZE; y++) {
      for (int i = 0; i < 4; i++) {
        if (s_copy[x][y][i] != maze_map_get_wall(x, y, (direction_t)i)) {
          return false;
        }
      }
    }
  }
  return true;
}

/* 決まった順に乱数を出す（毎回同じ迷路になるように、自分で作る） */
static unsigned int s_random_seed = 1;

static int random_below(int limit)
{
  s_random_seed = s_random_seed * 1103515245u + 12345u;
  return (int)((s_random_seed / 65536u) % (unsigned int)limit);
}

/* 壁の状態をでたらめに入れた迷路を作る（あり・なし・まだ見ていない）。
 * 内側の壁の known_percent [%] くらいを見たことにし、見た壁の wall_percent [%] くらいを「あり」にする */
static void make_random_map(int known_percent, int wall_percent)
{
  maze_map_init();
  for (int x = 0; x < MAZE_SIZE; x++) {
    for (int y = 0; y < MAZE_SIZE; y++) {
      if (y + 1 < MAZE_SIZE && random_below(100) < known_percent) {
        if (random_below(100) < wall_percent) {
          maze_map_set_wall(x, y, DIR_NORTH, WALL_EXIST);
        } else {
          maze_map_set_wall(x, y, DIR_NORTH, WALL_NONE);
        }
      }
      if (x + 1 < MAZE_SIZE && random_below(100) < known_percent) {
        if (random_below(100) < wall_percent) {
          maze_map_set_wall(x, y, DIR_EAST, WALL_EXIST);
        } else {
          maze_map_set_wall(x, y, DIR_EAST, WALL_NONE);
        }
      }
    }
  }
}

/* まだ見ていない壁の数（1 枚の壁は 2 つの区画から見えるので、北と東だけ数える。外周は見たことにしてある） */
static int count_unknown(void)
{
  int count = 0;
  for (int x = 0; x < MAZE_SIZE; x++) {
    for (int y = 0; y < MAZE_SIZE; y++) {
      if (maze_map_get_wall(x, y, DIR_NORTH) == WALL_UNKNOWN) {
        count = count + 1;
      }
      if (maze_map_get_wall(x, y, DIR_EAST) == WALL_UNKNOWN) {
        count = count + 1;
      }
    }
  }
  return count;
}

/* ---------------------------------------------------------------------------
 * 実機の出力を確かめる（--dump）
 * ------------------------------------------------------------------------- */

/* s_lines に読んだ出力の迷路から、PC で最短経路を計算し、出力の DIJK_PATH と同じか調べる。
 * 同じなら true。verbose なら結果を表示する */
static bool check_dump(bool verbose)
{
  int begin = find_line("#BEGIN MAZE");
  if (begin < 0 || !read_maze_lines()) {
    if (verbose) {
      printf("迷路（#BEGIN MAZE と ROW・COL・LROW・LCOL）が見つからない\n");
    }
    return false;
  }
  int goal_x = 0;
  int goal_y = 0;
  int size = 0;
  if (sscanf(s_lines[begin], "#BEGIN MAZE size=%d goal_x=%d goal_y=%d", &size, &goal_x, &goal_y) != 3) {
    if (verbose) {
      printf("#BEGIN MAZE の行が読めない: %s\n", s_lines[begin]);
    }
    return false;
  }

  /* 実機の出した最短経路の行（なければ NO PATH の行）を覚えておく */
  static char robot_path[LINE_LENGTH];
  int path_line = find_line("DIJK_PATH:");
  if (path_line < 0) {
    path_line = find_line("NO PATH");
  }
  robot_path[0] = '\0';
  if (path_line >= 0) {
    strcpy(robot_path, s_lines[path_line]);
  }

  /* 迷路を作り直して、PC でもう一度出力し、最短経路の行を比べる */
  rebuild_maze();
  begin_capture();
  output_maze(goal_x, goal_y);
  end_capture();
  int pc_line = find_line("DIJK_PATH:");
  if (pc_line < 0) {
    pc_line = find_line("NO PATH");
  }
  bool same = (pc_line >= 0 && strcmp(s_lines[pc_line], robot_path) == 0);

  if (verbose) {
    printf("迷路 %dx%d、ゴール (%d, %d) から 2×2、まだ見ていない壁 %d 枚\n", size, size, goal_x, goal_y, count_unknown());
    printf("実機: %s\n", robot_path);
    if (pc_line >= 0) {
      printf("PC  : %s\n", s_lines[pc_line]);
    }
    if (same) {
      printf("robot output: same as PC（実機と PC の最短経路は同じ）\n");
    } else {
      printf("robot output: DIFFERENT from PC（実機と PC の最短経路が違う）\n");
    }
  }
  return same;
}

/* ./test_output --dump ファイル */
static int run_dump(const char *path)
{
  FILE *file = fopen(path, "r");
  if (file == NULL) {
    printf("ファイルが開けない: %s\n", path);
    return 1;
  }
  s_line_count = 0;
  while (s_line_count < LINE_MAX_COUNT && fgets(s_lines[s_line_count], LINE_LENGTH, file) != NULL) {
    char *line = s_lines[s_line_count];
    size_t length = strlen(line);
    while (length > 0 && (line[length - 1] == '\n' || line[length - 1] == '\r')) {
      line[length - 1] = '\0';
      length = length - 1;
    }
    s_line_count = s_line_count + 1;
  }
  fclose(file);

  if (check_dump(true)) {
    return 0;
  }
  return 1;
}

/* ---------------------------------------------------------------------------
 * 1・2. 迷路
 * ------------------------------------------------------------------------- */

static void test_maze_output(void)
{
  printf("[1] 迷路：出力を読み直すと、元の迷路と同じになる\n");
  int fail_count = 0;
  int goal_x = 10;  /* ゴールの x を 10、11、…、14、10、… と順に変える */
  for (int i = 0; i < 50; i++) {
    s_random_seed = (unsigned int)(i + 1);
    make_random_map(i * 2, 40);   /* 見たことのある壁の割合を、0% から 98% まで変える */
    copy_map();
    int unknown_count = count_unknown();

    begin_capture();
    output_maze(goal_x, 12);
    end_capture();
    goal_x = goal_x + 1;
    if (goal_x > 14) {
      goal_x = 10;
    }

    bool ok = read_maze_lines();
    int unknown_line = find_line("unknown walls:");
    int printed_unknown = -1;
    if (unknown_line >= 0) {
      sscanf(s_lines[unknown_line], "unknown walls: %d", &printed_unknown);
    }
    if (printed_unknown != unknown_count) {
      ok = false;
    }
    if (find_line("#BEGIN MAZE size=32") != 0 && find_line("#BEGIN MAZE size=32") != 1) {
      ok = false;
    }
    if (find_line("#END MAZE") < 0) {
      ok = false;
    }
    rebuild_maze();
    if (!same_as_copy()) {
      ok = false;
    }
    if (!ok) {
      fail_count = fail_count + 1;
    }
  }
  printf("  元と違った迷路: %d 個（50 個中）\n", fail_count);
  check(fail_count == 0, "出力を読み直すと、壁・見たことのある壁・まだ見ていない壁の数が元と同じ");

  printf("[2] 迷路：読み直した迷路でもう一度計算した最短経路が、出力と同じ（--dump の確かめ方）\n");
  int differ_count = 0;
  int path_count = 0;
  int known_percent = 90;  /* 見たことのある壁の割合を、90、91、…、100、90、… と順に変える */
  for (int i = 0; i < 50; i++) {
    s_random_seed = (unsigned int)(i + 101);
    make_random_map(known_percent, 15);   /* ほとんど見てあり、壁が少ない（最短経路があることが多い） */
    known_percent = known_percent + 1;
    if (known_percent > 100) {
      known_percent = 90;
    }
    begin_capture();
    output_maze(14, 15);
    end_capture();
    if (find_line("DIJK_PATH:") >= 0) {
      path_count = path_count + 1;
    }
    if (!check_dump(false)) {
      differ_count = differ_count + 1;
    }
  }
  printf("  最短経路があった迷路 %d 個、出力と違った迷路 %d 個（50 個中）\n", path_count, differ_count);
  check(differ_count == 0, "読み直した迷路の最短経路が、出力の DIJK_PATH（なければ NO PATH）と同じ");
  check(path_count >= 25, "最短経路のある迷路も十分に試している");

  /* 確かめ方が、違うときに「違う」と言えるか：出力の DIJK_PATH の行を書き換えてから確かめる */
  s_random_seed = 8;   /* この迷路には最短経路がある（下の --dump の確かめでも使う） */
  make_random_map(100, 15);
  begin_capture();
  output_maze(14, 15);
  end_capture();
  int path_line = find_line("DIJK_PATH:");
  check(path_line >= 0, "壁が全部わかっていて少ない迷路には、最短経路がある");
  if (path_line >= 0) {
    strcat(s_lines[path_line], " 直進2");
  }
  check(!check_dump(false), "実機の最短経路が PC と違えば、違うと言う");

  /* ./test_output --dump と同じ：出力をファイルに保存して、そのファイルを確かめる */
  s_random_seed = 8;
  make_random_map(100, 15);
  const char *dump_path = "test_output_dump.txt";
  FILE *dump_file = fopen(dump_path, "w");
  if (dump_file == NULL) {
    check(false, "--dump を試すためのファイルが作れない");
    return;
  }
  fflush(stdout);
  int saved = dup(fileno(stdout));
  dup2(fileno(dump_file), fileno(stdout));
  output_all(14, 15);
  fflush(stdout);
  dup2(saved, fileno(stdout));
  close(saved);
  fclose(dump_file);
  printf("  --dump で保存したファイルを確かめる:\n");
  check(run_dump(dump_path) == 0, "保存したファイルを --dump で確かめると、同じと言う");
  remove(dump_path);

  /* ゴールの左下は、#BEGIN MAZE の行に書く（ビューアがゴールを描くのに使う） */
  maze_map_init();
  begin_capture();
  output_maze(20, 3);
  end_capture();
  check(find_line("#BEGIN MAZE size=32 goal_x=20 goal_y=3") >= 0, "#BEGIN MAZE の行に、ゴールの左下の位置を書く");
  check(find_line("NO PATH") >= 0, "何も見ていない迷路では、最短経路がない（まだ見ていない壁は「あり」とみなす）");
}

/* ---------------------------------------------------------------------------
 * 3・4. 走行ログ
 * ------------------------------------------------------------------------- */

/* i 回目に記録する値（テスト用に、i から決める） */
static log_values_t make_values(int i)
{
  log_values_t values;
  values.distance_mm = 1000.0f + i * 0.5f;
  values.speed_mm_s = i * 1.0f;
  values.speed_target_mm_s = 500.0f;
  values.omega_deg_s = 12.34f;
  values.omega_target_deg_s = -45.67f;
  values.angle_deg = 90.0f;
  values.sensor_left = 1200;
  values.sensor_right = 34;
  values.wall_dist_left_mm = 41.26f;
  values.wall_dist_right_mm = 48.0f;
  values.wall_used = 3;
  values.wall_error = -2.5f;
  values.wall_p = 0.123f;
  values.wall_d = -0.05f;
  values.voltage_left_v = 3.21f;
  values.voltage_right_v = -1.0f;
  values.battery_v = 8.123f;
  values.phase = LOG_PHASE_TURN;
  return values;
}

static void test_log(void)
{
  printf("[3] 走行ログ：行数、間隔、列、値\n");

  /* 1ms おき：制御周期（2kHz）2 回に 1 行。600 回呼ぶと 300 行 */
  log_start(1);
  for (int i = 0; i < 600; i++) {
    log_record(make_values(i));
  }
  log_stop();
  check(log_get_count() == 300, "1ms おきなら、制御周期 2 回に 1 行");
  check(!log_is_full(), "いっぱいではない");

  begin_capture();
  log_print();
  end_capture();
  int begin = find_line("#BEGIN LOG");
  check(begin >= 0 && strcmp(s_lines[begin], "#BEGIN LOG dt=0.001 n=300 div=1 full=0") == 0,
        "1 行目は #BEGIN LOG dt=0.001 n=300 div=1 full=0");
  check(begin >= 0 && strcmp(s_lines[begin + 1],
        "t,x,v,v_target,w,w_target,angle,sensor_L,sensor_R,dist_L,dist_R,wall,wall_err,wall_p,wall_d,wall_out,volt_L,volt_R,batt,phase") == 0,
        "2 行目は KOGUMA と同じ列名");
  int end = find_line("#END LOG");
  check(end - begin - 2 == 300, "値の行が 300 行");

  /* 1 行目は最初に呼んだときの値（i = 0）。11 行目は i = 20 */
  check(strcmp(s_lines[begin + 2], "0.000,0.0,0,500,12.3,-45.7,90.0,1200,34,41.3,48.0,3,-2.5,0.12,-0.05,0.07,3.2,-1.0,8.12,4") == 0,
        "1 行目：単位を戻した値（四捨五入）");
  check(strcmp(s_lines[begin + 12], "0.010,10.0,20,500,12.3,-45.7,90.0,1200,34,41.3,48.0,3,-2.5,0.12,-0.05,0.07,3.2,-1.0,8.12,4") == 0,
        "11 行目：時刻 0.010 s、走った距離 10mm（記録を始めてから）、速度 20");
  if (begin >= 0) {
    printf("  例: %s\n", s_lines[begin + 12]);
  }

  /* 入りきらない値は、端にそろえる */
  log_start(1);
  log_values_t big = make_values(0);
  big.speed_mm_s = 100000.0f;
  big.omega_deg_s = -5000.0f;
  log_record(big);
  log_stop();
  begin_capture();
  log_print();
  end_capture();
  begin = find_line("#BEGIN LOG");
  check(begin >= 0 && strncmp(s_lines[begin + 2], "0.000,0.0,32767,500,-3276.7,", 28) == 0,
        "入りきらない値は 32767（角速度は 0.1 単位なので -3276.7）にそろえる");

  printf("[4] 走行ログ：最短走行（8ms おき）で 8 秒を超えたとき\n");
  log_start(8);
  int call_count = 9 * CTRL_FREQ_HZ;   /* 9 秒ぶん */
  for (int i = 0; i < call_count; i++) {
    log_record(make_values(i));
  }
  check(log_get_count() == LOG_ROW_MAX, "1000 行で記録をやめる");
  check(log_is_full(), "いっぱいになったことを覚えている");
  log_stop();

  begin_capture();
  log_print();
  end_capture();
  begin = find_line("#BEGIN LOG");
  end = find_line("#END LOG");
  check(begin >= 0 && strcmp(s_lines[begin], "#BEGIN LOG dt=0.008 n=1000 div=8 full=1") == 0,
        "1 行目は #BEGIN LOG dt=0.008 n=1000 div=8 full=1");
  check(end - begin - 2 == LOG_ROW_MAX, "値の行が 1000 行");
  check(end > 0 && strncmp(s_lines[end - 1], "7.992,", 6) == 0, "最後の行は 7.992 秒（8 秒ぶん）");
  if (end > 0) {
    printf("  最後の行: %s\n", s_lines[end - 1]);
  }

  /* もう一度始めると、前のログは消えて最初から */
  log_start(2);
  for (int i = 0; i < 40; i++) {
    log_record(make_values(i));
  }
  log_stop();
  check(log_get_count() == 10 && !log_is_full(), "2ms おき：制御周期 4 回に 1 行。始め直すと前のログは消える");
}

/* ---------------------------------------------------------------------------
 * 5. 出力全部
 * ------------------------------------------------------------------------- */

static void test_output_all(void)
{
  printf("[5] 出力全部：迷路 → 走行ログ → #DONE\n");

  /* ログがある */
  maze_map_init();
  log_start(1);
  for (int i = 0; i < 20; i++) {
    log_record(make_values(i));
  }
  log_stop();
  begin_capture();
  output_all(15, 15);
  end_capture();
  int maze_line = find_line("#BEGIN MAZE");
  int log_line = find_line("#BEGIN LOG");
  int done_line = find_line("#DONE");
  check(maze_line >= 0 && log_line > maze_line && done_line > log_line, "迷路 → 走行ログ → #DONE の順");
  check(done_line == s_line_count - 1, "#DONE が最後の行");

  /* ログがない（電源を入れてから走っていない）：ログは出さない。
   * log.c の中の「記録した行数」を 0 に戻すため、記録しないまま始めてやめる */
  log_start(1);
  log_stop();
  begin_capture();
  output_all(15, 15);
  end_capture();
  check(find_line("#BEGIN LOG") < 0, "ログがなければ、走行ログは出さない");
  check(find_line("#BEGIN MAZE") >= 0 && find_line("#DONE") >= 0, "迷路と #DONE は出す");
}

/* ---------------------------------------------------------------------------
 * main
 * ------------------------------------------------------------------------- */

int main(int argc, char **argv)
{
  if (argc == 3 && strcmp(argv[1], "--dump") == 0) {
    return run_dump(argv[2]);
  }

  test_maze_output();
  test_log();
  test_output_all();

  if (s_fail_count == 0) {
    printf("すべて成功\n");
    return 0;
  }
  printf("失敗: %d 件\n", s_fail_count);
  return 1;
}
