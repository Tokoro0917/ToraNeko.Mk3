/*
 * failsafe.h
 * 異常（ぶつかった、壁を押し続けている）を見つけて、機体を止める
 *
 * 今の中身（failsafe.c）は、止まっているかを覚えておくところまで。異常を見つける部分は
 * ステップ9で作る（決めたことは KOGUMA_PORTING.md の F）。
 *   failsafe_is_stopped()  フェイルセーフで止まっているとき true
 *   failsafe_clear()       フェイルセーフを解除して、またモーターを動かせるようにする
 *   failsafe_stop()        モーターを止めて、フェイルセーフで止まった状態にする
 * KOGUMA_PORTING.md の F3 で決めた「今は探索の直進中」を explore.c から知らせる関数は、failsafe.c を作るときに足す。
 * PC のテストでは、test/ の中の仮の関数（test_maze.c の「仮の機体」、test_mode.c）がこの関数を持っている。
 */
#ifndef SYS_FAILSAFE_H
#define SYS_FAILSAFE_H

#include <stdbool.h>

bool failsafe_is_stopped(void);
void failsafe_clear(void);
void failsafe_stop(void);

#endif /* SYS_FAILSAFE_H */
