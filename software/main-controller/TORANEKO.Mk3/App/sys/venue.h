/*
 * venue.h
 * 会場で合わせるもの（モード7 No.3〜5。中身と流れは venue.c の先頭）
 *
 *   venue_init()                    電源を入れたときに呼ぶ。Flash に保存した倍率と壁制御の目標を読んで使い始める
 *   venue_measure_scale_run()       モード7 No.3：区画の境目で 1 秒測って、感度の倍率を決め、Flash に保存する
 *   venue_measure_reference_run()   モード7 No.4：区画の中心で 1 秒測って、壁制御の目標にし、Flash に保存する
 *   venue_reset_run()               モード7 No.5：倍率を 1 に、壁制御の目標を家の値に戻して、Flash に保存する
 *   venue_print()                   今の倍率と壁制御の目標をシリアルに出す（モード7 No.1 でも出す）
 */
#ifndef SYS_VENUE_H
#define SYS_VENUE_H

void venue_init(void);
void venue_measure_scale_run(void);
void venue_measure_reference_run(void);
void venue_reset_run(void);
void venue_print(void);

#endif /* SYS_VENUE_H */
