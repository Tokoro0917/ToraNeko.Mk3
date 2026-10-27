/*
 * goal.c
 * ゴールの位置（MODES.md のモード7 No.2）。Flash に保存し、電源を入れたときに読む
 */
#include "sys/goal.h"

#include "sys/flash.h"

/* 今のゴールの左下の区画 */
static int s_goal_x = GOAL_DEFAULT_X;
static int s_goal_y = GOAL_DEFAULT_Y;

/* 電源を入れたときに呼ぶ。Flash に保存したゴールを読む。
 * 保存したことがない、または値がおかしいときは、基準のゴール（goal.h の GOAL_DEFAULT_X・Y）にする */
void goal_init(void)
{
  int goal_x = GOAL_DEFAULT_X;
  int goal_y = GOAL_DEFAULT_Y;
  bool loaded = flash_load_goal(&goal_x, &goal_y);

  if (loaded && goal_is_valid(goal_x, goal_y)) {
    s_goal_x = goal_x;
    s_goal_y = goal_y;
  } else {
    s_goal_x = GOAL_DEFAULT_X;
    s_goal_y = GOAL_DEFAULT_Y;
  }
}

/* 今のゴールの左下の区画の x */
int goal_get_x(void)
{
  return s_goal_x;
}

/* 今のゴールの左下の区画の y */
int goal_get_y(void)
{
  return s_goal_y;
}

/* 左下の区画が (goal_x, goal_y) のゴールを使えるか。
 * 迷路からはみ出すときと、スタートの区画 (0, 0) を含むときは使えない */
bool goal_is_valid(int goal_x, int goal_y)
{
  if (goal_x < 0 || goal_x > GOAL_X_MAX) {
    return false;
  }
  if (goal_y < 0 || goal_y > GOAL_Y_MAX) {
    return false;
  }
  if (maze_is_in_area(0, 0, goal_x, goal_y, GOAL_WIDTH, GOAL_HEIGHT)) {
    return false;  /* スタートの区画を含む */
  }
  return true;
}

/* ゴールを (goal_x, goal_y) にして、Flash に保存する。使えない位置なら何もしないで false。
 * Flash に書いているあいだは CPU が止まるので、モーターを止めているときに呼ぶ */
bool goal_set(int goal_x, int goal_y)
{
  if (!goal_is_valid(goal_x, goal_y)) {
    return false;
  }
  s_goal_x = goal_x;
  s_goal_y = goal_y;
  flash_save_goal(goal_x, goal_y);
  return true;
}
