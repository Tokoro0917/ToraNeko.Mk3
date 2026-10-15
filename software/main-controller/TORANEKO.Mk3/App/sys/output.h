/*
 * output.h
 * 出力（MODES.md のモード7 No.1、KOGUMA_PORTING.md の B4・B5）
 *
 * RAM に残っているデータ（迷路、走行ログ）を、KOGUMA のビューア（tools/viewer.html）で読める形で PC へ出す。
 * 最後に #DONE を出す。
 */
#ifndef SYS_OUTPUT_H
#define SYS_OUTPUT_H

void output_all(int goal_x, int goal_y);
void output_maze(int goal_x, int goal_y);

#endif /* SYS_OUTPUT_H */
