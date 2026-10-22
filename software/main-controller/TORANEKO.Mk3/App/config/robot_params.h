/*
 * robot_params.h
 * 機体の定数（制御周期、タイヤ径、ギア比、トレッド幅など）
 *
 * 制御周期、電池の下限、タイヤの直径。ほかの値（トレッド幅など）は、使うときに足す。
 */
#ifndef CONFIG_ROBOT_PARAMS_H
#define CONFIG_ROBOT_PARAMS_H

/* 制御周期 [Hz]（README の決定事項）。制御の処理（ログの記録も）は 1 秒にこの回数呼ばれる */
#define CTRL_FREQ_HZ 2000
/* 制御周期 [s]（1 ÷ 2000 = 0.0005 秒 = 0.5ms） */
#define CTRL_PERIOD_S (1.0f / CTRL_FREQ_HZ)

/* 電池の下限の電圧 [V]。電源を入れたときにこれより低ければ、何もせずに知らせる（MODES.md の 2-1）。
 * 2 セルのリポで、1 セル 3.75V（2026-10-22 に作者が決めた） */
#define BATTERY_LOW_V 7.5f

/* タイヤの直径 [mm]。モード6 No.1（直進）で、走った距離をものさしで測って合わせる */
#define TIRE_DIAMETER_MM 14.0f

/* ギア比（41T ÷ 11T ≒ 3.73）は、ここには置かない。エンコーダはタイヤの軸に付いているので、
 * 距離の計算にギア比は要らない */

#endif /* CONFIG_ROBOT_PARAMS_H */
