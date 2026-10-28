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

/* ---- 最短走行（モード1）No.4・5：大回り・斜めのターン（config/turn_params.h、今は 500mm/s）で曲がる（仮） ---- */

/* No.4 */
#define SHORTEST4_MAX_SPEED_MM_S      600.0f   /* 縦横の直進の最高速 [mm/s] */
#define SHORTEST4_ACCEL_MM_S2         2000.0f  /* 縦横の直進の加速度 [mm/s²] */
#define SHORTEST4_DIAG_MAX_SPEED_MM_S 500.0f   /* 斜めの直進の最高速 [mm/s] */
#define SHORTEST4_DIAG_ACCEL_MM_S2    2000.0f  /* 斜めの直進の加速度 [mm/s²] */
/* No.5 */
#define SHORTEST5_MAX_SPEED_MM_S      1000.0f
#define SHORTEST5_ACCEL_MM_S2         3000.0f
#define SHORTEST5_DIAG_MAX_SPEED_MM_S 700.0f
#define SHORTEST5_DIAG_ACCEL_MM_S2    3000.0f

/* ゴールにターンで入って終わるときに止まる加速度 [mm/s²]。500mm/s から 42mm で止まる（ゴールの区画の残り 45mm に入る） */
#define SHORTEST_STOP_ACCEL_MM_S2 3000.0f

/* ---- 探索の前壁補正（ctrl/run.c。2026-10-28） ----
 * 区画の入口で、その区画に前の壁があるとき、前センサで測った壁までの距離が FRONT_WALL_AT_ENTRY_MM からずれていれば、
 * そのぶん、曲がり始め（スラロームの前距離）と、区画のまん中で止まる位置を直す。
 * 走るうちに前後の位置がずれても、前に壁がある区画で毎回もとに戻る。 */

/* 補正を実際にかけるか（1 かける、0 かけない）。0 でも、かけるはずだった量は測って、モード7 No.1 で出す。
 * 今の壁までの距離の表（ctrl/wall_dist.c）は仮なので、表を測って入れ、出た量がおかしくないのを確かめてから 1 にする */
#define FRONT_CORRECTION_ENABLE 0
#define FRONT_WALL_AT_ENTRY_MM   87.0f  /* 区画の入口の辺にいるときの、前の壁までの距離 [mm]（45 + 42） */
#define FRONT_CORRECTION_LIMIT_MM 10.0f /* 直す量の上限 [mm]（センサの読み違いで大きく直さないように） */

/* ---- 探索の壁切れ（ctrl/run.c。2026-10-28） ----
 * 探索で 1 区画まっすぐ進むとき、入口で横に壁があった側のセンサが「壁なし」に変わったら（壁の切れ目＝区画の出口の柱を過ぎた）、
 * そのときの位置は区画の入口から WALL_EDGE_POSITION_MM のはずなので、ずれた分だけ、この直進の残りの距離を直す。
 * 前に壁がなくても、横の壁が切れるところで前後の位置が戻る。 */

/* 補正を実際にかけるか（1 かける、0 かけない）。0 でも、切れ目を見つけた位置は測って、モード7 No.1 で出す */
#define WALL_EDGE_ENABLE 0
/* 切れ目を見つける位置 [mm]（区画の入口の辺から。横センサは少し前を見るので、出口の辺 90mm より手前で見つかる）。
 * 今の値は見積もり（仮）。WALL_EDGE_ENABLE を 0 のまま探索し、モード7 No.1 の Search: wall edge … の
 * average（平均）をここに入れてから、WALL_EDGE_ENABLE を 1 にする */
#define WALL_EDGE_POSITION_MM 70.0f
#define WALL_EDGE_WINDOW_MM   20.0f  /* WALL_EDGE_POSITION_MM からこれ以上ずれて見つかったら、読み違いとして使わない [mm] */

/* ---- 吸引（drv/fan.c。2026-10-28） ---- */

#define SUCTION_VOLTAGE_V 2.0f  /* 吸引ファンにかける電圧 [V]（仮。モード6 No.3 で吸い付き方を見て決める。上限は drv/fan.c の FAN_LIMIT_V） */
#define SUCTION_RAMP_MS   500   /* 0 から SUCTION_VOLTAGE_V まで上げるのにかける時間 [ms]（急に回して電池の電圧が落ちないように） */

/* ---- その場旋回 ---- */

#define PIVOT_OMEGA_DEG_S  720.0f   /* 最高の角速度 [deg/s]（KOGUMA は 2000。まずゆっくり） */
#define PIVOT_ALPHA_DEG_S2 7200.0f  /* 角加速度 [deg/s²]（KOGUMA は 50000。まずゆっくり） */

/* ---- 前壁合わせ ---- */

#define ALIGN_TIME_MS 500  /* 前壁合わせを続ける時間 [ms]（KOGUMA の Alignment_TIME と同じ 0.5 秒） */

/* ---- 止まるとき ---- */

#define STOP_SETTLE_MS 100  /* 区画のまん中で止まってから、制御を続けて待つ時間 [ms]（ぴたりと止めるため） */

#endif /* CONFIG_RUN_PARAMS_H */
