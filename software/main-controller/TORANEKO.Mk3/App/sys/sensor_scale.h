/*
 * sensor_scale.h
 * 会場ごとの感度合わせ（モード5 No.7・8。中身と流れは sensor_scale.c の先頭）
 *
 *   sensor_scale_init()         電源を入れたときに呼ぶ。Flash に保存した倍率を読んで使い始める
 *   sensor_scale_measure_run()  モード5 No.7：区画のまん中で 1 秒測って倍率を決め、Flash に保存する
 *   sensor_scale_reset_run()    モード5 No.8：倍率を 1（直さない）に戻して、Flash に保存する
 *   sensor_scale_print()        今の倍率をシリアルに出す（モード7 No.1 でも出す）
 */
#ifndef SYS_SENSOR_SCALE_H
#define SYS_SENSOR_SCALE_H

void sensor_scale_init(void);
void sensor_scale_measure_run(void);
void sensor_scale_reset_run(void);
void sensor_scale_print(void);

#endif /* SYS_SENSOR_SCALE_H */
