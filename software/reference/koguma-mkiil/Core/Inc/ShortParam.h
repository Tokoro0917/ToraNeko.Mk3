/*
 * ShortParam.h
 *
 * 最短走行のターンの値と、よく使う最短走行の値の組(2026-10-27)。
 * Move.c(走る)と Maze.c(ダイクストラのコスト)の両方が使うので、HAL に依存しない
 * ShortParam.c に置いた(PC のテスト・シミュレータからもリンクできる)。
 */

#ifndef INC_SHORTPARAM_H_
#define INC_SHORTPARAM_H_

/* 2000のターンの値(G_Turn2000)。BFS・ダイクストラ・モード6で共通 */
#define WC2000_NORMAL 0
#define WC2000_NANAME 1
enum {
	T2000_BIG90, T2000_BIG180, T2000_IN45, T2000_IN135, T2000_OUT45, T2000_OUT135, T2000_V90, T2000_NUM
};
typedef struct {
	int st_kind;	/* 前距離の壁切れ: WC2000_NORMAL / WC2000_NANAME */
	float st;		/* 前距離[mm] */
	float ang;		/* 角度[deg](左が正) */
	float w;		/* 最大角速度[deg/s] */
	float w_ac;		/* 角加速度[deg/s^2] */
	int end_kind;	/* 後距離の壁切れ */
	float end;		/* 後距離[mm] */
} Turn2000_Param;
extern Turn2000_Param G_Turn2000[T2000_NUM];
/* 2400のターンの値(G_Turn2400、2026-10-21)。Short_NANAME_Move2400(BFS・ダイクストラ)・モード7で共通。形と並びは G_Turn2000 と同じ */
extern Turn2000_Param G_Turn2400[T2000_NUM];

/* 最短走行の値の組(2026-10-27)。ダイクストラのコスト(Maze.c の Maze_Dijkstra_Set_Run)に使う */
typedef struct {
	int v;			/* ターンの速度[mm/s] */
	int vmax;		/* 直線の最高速度[mm/s] */
	int ac;			/* 縦横の直線の加速度[mm/s^2](減速は2倍) */
	int ac_naname;	/* 斜めの直線の加速度・減速度[mm/s^2] */
	const Turn2000_Param *turn;	/* ターンの値(T2000_NUM 個) */
} ShortRunParam;

/* よく使う最短走行の値の組。ダイクストラ誘導の探索は、全部の組の経路の未確認の壁を見に行く。
 * [0] モード1 No.5(探索のあとの最短と同じ)、[1] モード1 No.8 */
#define SHORT_RUN_SET_NUM 2
extern const ShortRunParam G_Short_Run_Set[SHORT_RUN_SET_NUM];

#endif /* INC_SHORTPARAM_H_ */
