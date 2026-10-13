#ifndef APP_H
#define APP_H

/* main.c から呼ばれる App 全体の入口 */
void app_init(void);  /* 周辺機能の初期化（MX_*_Init）が終わった後に 1 回だけ呼ぶ */
void app_loop(void);  /* main() の while(1) から繰り返し呼ぶ */

#endif /* APP_H */
