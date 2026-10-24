/*
 * adjust.h
 * 動きを 1 つずつ走らせて調整する（モード2・6。MODES.md）
 */
#ifndef SYS_ADJUST_H
#define SYS_ADJUST_H

void adjust_straight(void);
void adjust_pivot_180(void);
void adjust_slalom_90(void);
void adjust_straight_along_wall(void);
void adjust_print_last_result(void);

#endif /* SYS_ADJUST_H */
