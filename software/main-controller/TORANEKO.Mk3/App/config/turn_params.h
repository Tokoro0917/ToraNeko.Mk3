/*
 * turn_params.h
 * 最短走行のターン（大回り・斜め）の値。ctrl/run.c と sys/turn_test.c（モード3）が使う
 *
 * ターンは「前距離（まっすぐ）→ スラローム（前に進む速さは一定、角速度は台形）→ 後距離（まっすぐ）」。
 * 曲がる角度・始めと終わりの位置は迷路の形で決まる（test/test_turn.c の先頭のコメント）。
 *
 * ■ 値の決め方
 *   1. ターンの速さ（FAST_TURN_SPEED_MM_S）と角加速度（FAST_TURN_ALPHA_DEG_S2）を決める
 *   2. software/main-controller/test で make test_turn && ./test_turn --calc を実行し、出た行を下に貼る
 *      （最高角速度は、前距離・後距離の短いほうが 3mm になるように計算する）
 *   3. 実機でモード3 の No. ごとに走らせて、ずれる分だけ前距離・後距離を直す
 * 下の値は 2026-10-27 に PC で計算しただけの「仮」の値（実機ではまだ走らせていない）。
 */
#ifndef CONFIG_TURN_PARAMS_H
#define CONFIG_TURN_PARAMS_H

#define FAST_TURN_SPEED_MM_S   500.0f    /* ターンの速さ [mm/s]（探索の小回りは 300） */
#define FAST_TURN_ALPHA_DEG_S2 10000.0f  /* 角加速度 [deg/s²]（8000 だと V90 が 3mm の前距離に収まらない） */

#define DIAG_STEP_MM 63.64f  /* 斜めの直進の 1 つぶん [mm]（壁の辺のまん中から、となりの辺のまん中まで = 45 × √2） */

/* 大回り 90° */
#define BIG90_OMEGA_DEG_S 367.8f  /* 最高角速度 [deg/s] */
#define BIG90_PRE_MM 3.0f  /* 前距離 [mm] */
#define BIG90_POST_MM 3.2f  /* 後距離 [mm] */
/* 大回り 180° */
#define BIG180_OMEGA_DEG_S 650.9f  /* 最高角速度 [deg/s] */
#define BIG180_PRE_MM 3.0f  /* 前距離 [mm] */
#define BIG180_POST_MM 3.0f  /* 後距離 [mm] */
/* 斜め入り 45° */
#define IN45_OMEGA_DEG_S 356.6f  /* 最高角速度 [deg/s] */
#define IN45_PRE_MM 3.0f  /* 前距離 [mm] */
#define IN45_POST_MM 21.8f  /* 後距離 [mm] */
/* 斜め入り 135° */
#define IN135_OMEGA_DEG_S 651.8f  /* 最高角速度 [deg/s] */
#define IN135_PRE_MM 10.5f  /* 前距離 [mm] */
#define IN135_POST_MM 3.2f  /* 後距離 [mm] */
/* 斜め出 45° */
#define OUT45_OMEGA_DEG_S 353.5f  /* 最高角速度 [deg/s] */
#define OUT45_PRE_MM 21.4f  /* 前距離 [mm] */
#define OUT45_POST_MM 3.1f  /* 後距離 [mm] */
/* 斜め出 135° */
#define OUT135_OMEGA_DEG_S 653.3f  /* 最高角速度 [deg/s] */
#define OUT135_PRE_MM 3.0f  /* 前距離 [mm] */
#define OUT135_POST_MM 10.9f  /* 後距離 [mm] */
/* V90° */
#define V90_OMEGA_DEG_S 658.7f  /* 最高角速度 [deg/s] */
#define V90_PRE_MM 3.0f  /* 前距離 [mm] */
#define V90_POST_MM 3.2f  /* 後距離 [mm] */

#endif /* CONFIG_TURN_PARAMS_H */
