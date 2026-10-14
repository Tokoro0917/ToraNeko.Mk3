/*
 * flash.h
 * 迷路とゴールの位置を Flash に保存する・読み込む
 *
 * 中身（flash.c）はステップ9で作る（決めたことは MODES.md の 5）。
 * 今は、探索（sys/explore.c）が使う「迷路を保存する」の形だけを先に決めておく。
 * PC のテストでは、test/test_maze.c の中の「仮の機体」がこの関数を持っている。
 *
 * ■ 注意
 * Flash を消して書き込んでいるあいだは CPU が止まる（約 1〜2 秒）。
 * そのあいだは制御の割り込みも動かないので、モーターを止めているときだけ呼ぶ。
 */
#ifndef SYS_FLASH_H
#define SYS_FLASH_H

void flash_save_maze(void);

#endif /* SYS_FLASH_H */
