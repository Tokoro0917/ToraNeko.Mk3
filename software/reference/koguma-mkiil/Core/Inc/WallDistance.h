/*
 * WallDistance.h
 *
 *  壁センサ値 -> 距離[mm] 変換
 *
 *  距離の定義(キャリブレーション時もこの定義で測る):
 *    横センサ(L/R): 機体の中心線から横壁の表面までの距離
 *    前センサ(FL/FR): 機体の中心から前壁の表面までの距離
 *  どちらも区画中心に置いたとき 84mm (= (180 - 12) / 2) になる。
 *
 *  HALに依存しないので、ホスト(PC)のgccでもそのままテストできる。
 */

#ifndef INC_WALLDISTANCE_H_
#define INC_WALLDISTANCE_H_

/* センサ番号は g_sensor[][] と同じ並び */
#define WALLDIST_FL 0
#define WALLDIST_R  1
#define WALLDIST_L  2
#define WALLDIST_FR 3

/* 区画中心にいるときの壁までの距離[mm] */
#define WALLDIST_CENTER_MM 84.0f

/* センサ値 v を距離[mm]に変換する。
 * 表の範囲外は端の距離に張り付く(近すぎ -> 最小距離、遠すぎ -> 最大距離) */
float WallDist_mm(int sensor, int v);

/* 会場ごとの感度の倍率(会場の区画中心の値 / 表の区画中心の値)。
 * WallDist_mm() はセンサ値をこの倍率で割ってから表を引く。
 * モード3 No.5 で出た値を WallDistance.c に手で書き写す。並びは g_sensor と同じ */
extern float WallDist_Scale[4];

/* 表(倍率1)で、距離 mm のときのセンサ値。表の範囲外は端の値 */
float WallDist_Value(int sensor, float mm);

#endif /* INC_WALLDISTANCE_H_ */
