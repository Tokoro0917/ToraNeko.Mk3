/*
 * flash.h
 * 迷路とゴールの位置を Flash に保存する・読み込む
 *
 * 中身（flash.c）はステップ9で作る（決めたことは MODES.md の 5）。
 * 今は、探索（sys/explore.c）とモード選択（sys/mode.c、sys/goal.c）が使う関数の形だけを先に決めておく。
 *   flash_save_maze()   今の迷路（maze_map）を保存する
 *   flash_load_maze()   保存した迷路を、今の迷路（maze_map）に戻す。保存した迷路がなければ false
 *   flash_save_goal()   ゴールの位置（左下の区画）を保存する
 *   flash_load_goal()   保存したゴールの位置を goal_x、goal_y に入れる。保存したことがなければ false
 * PC のテストでは、test/ の中の仮の関数（test_maze.c の「仮の機体」、test_mode.c）がこの関数を持っている。
 *
 * ■ 注意
 * Flash を消して書き込んでいるあいだは CPU が止まる（約 1〜2 秒）。
 * そのあいだは制御の割り込みも動かないので、モーターを止めているときだけ呼ぶ。
 */
#ifndef SYS_FLASH_H
#define SYS_FLASH_H

#include <stdbool.h>

void flash_save_maze(void);
bool flash_load_maze(void);
void flash_save_goal(int goal_x, int goal_y);
bool flash_load_goal(int *goal_x, int *goal_y);

#endif /* SYS_FLASH_H */
