/*
 * robot_params.h
 * 機体の定数（制御周期、タイヤ径、ギア比、トレッド幅など）
 *
 * 今は制御周期と電池の下限だけ。ほかの値は、使うステップ（2〜7）で足す。
 */
#ifndef CONFIG_ROBOT_PARAMS_H
#define CONFIG_ROBOT_PARAMS_H

/* 制御周期 [Hz]（README の決定事項）。制御の処理（ログの記録も）は 1 秒にこの回数呼ばれる */
#define CTRL_FREQ_HZ 2000

/* 電池の下限の電圧 [V]。電源を入れたときにこれより低ければ、何もせずに知らせる（MODES.md の 2-1）。
 * 2 セルのリポで、1 セル 3.75V（2026-10-22 に作者が決めた） */
#define BATTERY_LOW_V 7.5f

#endif /* CONFIG_ROBOT_PARAMS_H */
