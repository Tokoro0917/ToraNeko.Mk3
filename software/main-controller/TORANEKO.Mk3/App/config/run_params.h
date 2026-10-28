/*
 * run_params.h
 * 探索の動きの値（速さ、スラローム、その場旋回、区画の大きさ）。ctrl/run.c と sys/adjust.c が使う
 *
 * 調整の動き（モード2）と探索（モード0）で同じ値を使うので、ここに 1 か所にまとめる。
 */
#ifndef CONFIG_RUN_PARAMS_H
#define CONFIG_RUN_PARAMS_H

/* ---- 区画と、スタートの位置 ---- */

#define HALF_CELL_MM 45.0f  /* 区画の半分 [mm]（ハーフの 1 区画は 90mm） */
#define CELL_MM      90.0f  /* 1 区画 [mm] */

/* スタート区画で、機体のお尻を後ろの壁に付けて置いたとき、タイヤの軸が区画のまん中から何 mm 後ろにあるか。
 * 壁の内側は区画のまん中から 42mm 後ろ（区画の半分 45mm − 壁の厚さの半分 3mm）、
 * タイヤの軸は基板の後ろの端から 32mm 前（hardware/kicad/TORANEKO.Mk3 の配置ガイドの図から。
 * 基板は前後 64mm で、軸はそのまん中）なので、42 − 32 = 10mm。実機で合わせる */
#define START_OFFSET_MM 10.0f

/* ---- 探索の直進 ---- */

#define SEARCH_SPEED_MM_S  300.0f   /* 探索の速さ [mm/s] */
#define SEARCH_ACCEL_MM_S2 1000.0f  /* 探索の加速度 [mm/s²]（止まったところから 45mm で 300mm/s になる） */

/* ---- 既知区間加速（探索 No.4。見たことのある区画がまっすぐ 2 区画以上続くところ）（仮） ---- */

#define KNOWN_MAX_SPEED_MM_S 600.0f   /* 最高速 [mm/s] */
#define KNOWN_ACCEL_MM_S2    2000.0f  /* 加速度 [mm/s²]（300 → 600mm/s に 68mm。2 区画あれば上げて戻せる） */

/* ---- 探索のスラローム（小回り 90°） ---- */

#define SLALOM_ANGLE_DEG    90.0f    /* 曲がる角度 [deg] */
#define SLALOM_OMEGA_DEG_S  600.0f   /* 最高の角速度 [deg/s] */
#define SLALOM_ALPHA_DEG_S2 8000.0f  /* 角加速度 [deg/s²] */
/* 前距離・後距離 [mm]（曲がる前・後の直進）。300mm/s、600deg/s、8000deg/s² なら、曲がるあいだに
 * 前へ 40.4mm、横へ 40.3mm 進むので、区画の半分 45mm から引いた値（PC で計算）。
 * 2026-10-24 に実機で、左の区画のまん中で止まった */
#define SLALOM_PRE_MM  4.6f
#define SLALOM_POST_MM 4.7f

/* ---- 最短走行（モード1）の直進。曲がるところは探索と同じスラローム（300mm/s） ---- */

/* No.1：探索と同じ速さ（道を確かめる） */
#define SHORTEST1_MAX_SPEED_MM_S 300.0f   /* 直進の最高速 [mm/s] */
#define SHORTEST1_ACCEL_MM_S2    1000.0f  /* 直進の加速度 [mm/s²] */
/* No.2 */
#define SHORTEST2_MAX_SPEED_MM_S 600.0f
#define SHORTEST2_ACCEL_MM_S2    2000.0f
/* No.3 */
#define SHORTEST3_MAX_SPEED_MM_S 1000.0f
#define SHORTEST3_ACCEL_MM_S2    3000.0f

/* ---- その場旋回 ---- */

#define PIVOT_OMEGA_DEG_S  720.0f   /* 最高の角速度 [deg/s]（KOGUMA は 2000。まずゆっくり） */
#define PIVOT_ALPHA_DEG_S2 7200.0f  /* 角加速度 [deg/s²]（KOGUMA は 50000。まずゆっくり） */

/* ---- 前壁合わせ ---- */

#define ALIGN_TIME_MS 500  /* 前壁合わせを続ける時間 [ms]（KOGUMA の Alignment_TIME と同じ 0.5 秒） */

/* ---- 止まるとき ---- */

#define STOP_SETTLE_MS 100  /* 区画のまん中で止まってから、制御を続けて待つ時間 [ms]（ぴたりと止めるため） */

#endif /* CONFIG_RUN_PARAMS_H */
