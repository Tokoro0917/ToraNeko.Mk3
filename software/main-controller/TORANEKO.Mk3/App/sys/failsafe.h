/*
 * failsafe.h
 * 異常（ぶつかった、壁を押し続けている）を見つけて、機体を止める
 *
 * 中身（failsafe.c）はステップ9で作る（決めたことは KOGUMA_PORTING.md の F）。
 * 今は、探索（sys/explore.c）が使う「フェイルセーフで止まっているか」の形だけを先に決めておく。
 * PC のテストでは、test/test_maze.c の中の「仮の機体」がこの関数を持っている。
 */
#ifndef SYS_FAILSAFE_H
#define SYS_FAILSAFE_H

#include <stdbool.h>

bool failsafe_is_stopped(void);

#endif /* SYS_FAILSAFE_H */
