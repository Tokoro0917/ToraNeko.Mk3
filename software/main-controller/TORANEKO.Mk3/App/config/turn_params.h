/*
 * turn_params.h
 * 最短走行のターン（大回り・斜め）の値。ctrl/run.c と sys/turn_test.c（モード3）が使う
 *
 * ターンは「前距離（まっすぐ）→ スラローム（前に進む速さは一定、角速度は台形）→ 後距離（まっすぐ）」。
 * 曲がる角度・始めと終わりの位置は迷路の形で決まる（test/test_turn.c の先頭のコメント）。
 *
 * ■ ターンごとの速さ（2026-11-03）
 * ターンごとに「そのターンを曲がる速さ」（*_SPEED_MM_S）と角加速度（*_ALPHA_DEG_S2）を決める。
 * 曲がると横向きに「速さ × 角速度」の加速度がかかり、吸引 5.0V では 10000mm/s² を超えると滑る
 * （モード4 の加速度テスト。12000mm/s² で滑り始めた）。余裕をみて 8000mm/s² くらいまでにする。
 *   - ゆるいターン（大回り 90°・斜め入り 45°・斜め出 45°）：800mm/s（横向きの加速度 7800〜8200mm/s²）
 *   - きついターン（大回り 180°・斜め入り 135°・斜め出 135°・V90）：600mm/s（同じく 8170〜8260mm/s²）
 * 角加速度は、500mm/s で 10000deg/s² のときと同じ形になるように、10000 ×（速さ ÷ 500）² にする。
 * 同じ形なので、前距離・後距離は 500mm/s のときと同じになる。
 *
 * ■ 決めた速さより遅く曲がるとき
 * ctrl/run.c の fast_turn() は、決めた速さより遅いときは、最高角速度を k 倍、角加速度を k² 倍にして
 * （k = 速さ ÷ 決めた速さ）、同じ形で曲がる。最短走行（sys/shortest.c）は、今はどのターンも FAST_TURN_SPEED_MM_S
 * （500mm/s）で曲がる（ターンごとに直線の終わりの速さを変えるのは、ターンの調整のあとで作る）。
 *
 * ■ 値の決め方
 *   1. ターンごとの速さと角加速度を決める
 *   2. software/main-controller/test で make test_turn && ./test_turn --calc を実行し、出た行を下に貼る
 *      （最高角速度は、前距離・後距離の短いほうが 3mm になるように計算する）
 *   3. 実機でモード3 の No. ごとに走らせて、ずれる分を、下の「実機で足す前距離・後距離」（*_PRE_ADJUST_MM・
 *      *_POST_ADJUST_MM）に入れる。2 の計算の値はそのままにしておく（test_turn が、計算の値でターンの形を確かめるため）。
 *      前距離を足すと、曲がり始めが前にずれる（止まる位置が、始めの向きに進む）。後距離を足すと、曲がったあとが長くなる
 * 下の値は PC で計算しただけの「仮」の値（実機ではまだ走らせていない）。
 */
#ifndef CONFIG_TURN_PARAMS_H
#define CONFIG_TURN_PARAMS_H

/* 最短走行で、今どのターンも曲がる速さ [mm/s]（探索の小回りは 300）。ターンごとの速さ（下）より遅くする */
#define FAST_TURN_SPEED_MM_S   500.0f

#define DIAG_STEP_MM 63.64f  /* 斜めの直進の 1 つぶん [mm]（壁の辺のまん中から、となりの辺のまん中まで = 45 × √2） */

/* ターンごとの速さ [mm/s] と角加速度 [deg/s²]（先頭のコメント） */
#define BIG90_SPEED_MM_S    800.0f
#define BIG90_ALPHA_DEG_S2  25600.0f
#define BIG180_SPEED_MM_S   600.0f
#define BIG180_ALPHA_DEG_S2 14400.0f
#define IN45_SPEED_MM_S     800.0f
#define IN45_ALPHA_DEG_S2   25600.0f
#define IN135_SPEED_MM_S    600.0f
#define IN135_ALPHA_DEG_S2  14400.0f
#define OUT45_SPEED_MM_S    800.0f
#define OUT45_ALPHA_DEG_S2  25600.0f
#define OUT135_SPEED_MM_S   600.0f
#define OUT135_ALPHA_DEG_S2 14400.0f
#define V90_SPEED_MM_S      600.0f
#define V90_ALPHA_DEG_S2    14400.0f

/* 実機で足す前距離・後距離 [mm]（先頭のコメントの 3。計算の値に足して走る）。
 * 大回り 90°：2026-11-04 に 800mm/s で走らせたら、止まる位置が始めの向き（北）に 10mm、曲がったあとの向き（西）に
 * 10mm 手前だった（エンコーダとジャイロから計算した位置は 3mm 以内に合っていたので、曲がっているあいだにタイヤが
 * 少しずつ空回りしている。まっすぐ 6000mm/s² で加速しても滑らなかった）。前距離・後距離を 10mm ずつ足す。
 * そのあと、向きの制御を直して（ctrl_gains.h の FF_SUCTION_SLALOM_V_PER_DEG_S・ANGLE_KI）走らせたら、止まる位置のずれは
 * 北・西とも 1mm、向きは 90° ちょうどになった（2026-11-04）。作者が、その 1mm も直すため、前距離を 1mm・後距離を
 * 2mm 減らした（+10 → +9、+10 → +8） */
#define BIG90_PRE_ADJUST_MM   9.0f
#define BIG90_POST_ADJUST_MM  8.0f
#define BIG180_PRE_ADJUST_MM  0.0f
#define BIG180_POST_ADJUST_MM 0.0f
#define IN45_PRE_ADJUST_MM    0.0f
#define IN45_POST_ADJUST_MM   0.0f
#define IN135_PRE_ADJUST_MM   0.0f
#define IN135_POST_ADJUST_MM  0.0f
#define OUT45_PRE_ADJUST_MM   0.0f
#define OUT45_POST_ADJUST_MM  0.0f
#define OUT135_PRE_ADJUST_MM  0.0f
#define OUT135_POST_ADJUST_MM 0.0f
#define V90_PRE_ADJUST_MM     0.0f
#define V90_POST_ADJUST_MM    0.0f

/* 最高角速度・前距離・後距離（test_turn --calc の出力） */
/* 前距離・後距離の短いほう 3.0mm で計算（test/test_turn.c --calc） */
/* 大回り 90°（800mm/s、角加速度 25600deg/s²。横向きの加速度はいちばん大きいとき 8199mm/s²） */
#define BIG90_OMEGA_DEG_S 587.2f
#define BIG90_PRE_MM 2.0f
#define BIG90_POST_MM 2.2f
/* 大回り 180°（600mm/s、角加速度 14400deg/s²。横向きの加速度はいちばん大きいとき 8179mm/s²） */
#define BIG180_OMEGA_DEG_S 781.0f
#define BIG180_PRE_MM 3.0f
#define BIG180_POST_MM 3.0f
/* 斜め入り 45°（800mm/s、角加速度 25600deg/s²。横向きの加速度はいちばん大きいとき 7915mm/s²） */
#define IN45_OMEGA_DEG_S 566.9f
#define IN45_PRE_MM 3.0f
#define IN45_POST_MM 21.6f
/* 斜め入り 135°（600mm/s、角加速度 14400deg/s²。横向きの加速度はいちばん大きいとき 8168mm/s²） */
#define IN135_OMEGA_DEG_S 780.0f
#define IN135_PRE_MM 10.4f
#define IN135_POST_MM 3.1f
/* 斜め出 45°（800mm/s、角加速度 25600deg/s²。横向きの加速度はいちばん大きいとき 7848mm/s²） */
#define OUT45_OMEGA_DEG_S 562.1f
#define OUT45_PRE_MM 21.4f
#define OUT45_POST_MM 3.2f
/* 斜め出 135°（600mm/s、角加速度 14400deg/s²。横向きの加速度はいちばん大きいとき 8204mm/s²） */
#define OUT135_OMEGA_DEG_S 783.4f
#define OUT135_PRE_MM 3.0f
#define OUT135_POST_MM 11.1f
/* V90°（600mm/s、角加速度 14400deg/s²。横向きの加速度はいちばん大きいとき 8260mm/s²） */
#define V90_OMEGA_DEG_S 788.7f
#define V90_PRE_MM 3.0f
#define V90_POST_MM 3.2f

#endif /* CONFIG_TURN_PARAMS_H */
