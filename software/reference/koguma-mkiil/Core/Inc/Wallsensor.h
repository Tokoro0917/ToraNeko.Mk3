/*
 * Wallsensor.h
 *
 *  Created on: Jun 21, 2024
 *      Author: akihi
 */

#ifndef INC_WALLSENSOR_H_
#define INC_WALLSENSOR_H_

extern int G_Wall_data[];
extern int G_WallCtrl_Use_mm;
extern float Wall_error;
extern float G_Wall_P, G_Wall_D, G_Wall_dL, G_Wall_dR;
extern int G_Wall_St;

void Wall_search();

float Wall_Flont_Av();

int Sensor_Enter();
int Sensor_Start();

void Wall_search_LED();
void Wall_Distance_Calibration();
void Wall_Center_Calibration();

float calWallConrol();
float Wall_Control_Update();
int Wall_Sensor_Med3(int i);

/* 柱の端(docs/pillar_cut_design.md) */
extern float Edge_Off[2][2];
extern float Edge_Tan;
void Wall_Edge_Sample();
/* 前壁を左右の平均でも見る(1 壁あり)。左右とも閾値を超えたとき、または左右の平均が閾値を超え低いほうも閾値の1/3を超えたとき */
int Wall_Front_For_Map(void);
extern int Align_Use_mm;	//止まったときの前壁合わせを距離で行う(0 は生の値)
extern float Align_Front_mm;	//前壁合わせの目標[mm]
extern float Front_Wall_Min_Ratio;
extern int Wall_threshold[];
/* 境目の推定が [lo, hi](G_Odo_X)にあれば1。左右にあれば平均。src は EDGE_SRC_*(PillarEdge.h)。
 * sides: 使う側(EDGE_SRC_R / EDGE_SRC_L の組み合わせ) */
int Edge_Cut_Find(float lo, float hi, int sides, float *c, int *src);
/* 最後に見つけた柱の端(2026-10-19)。sides の側(EDGE_SRC_R / EDGE_SRC_L の組み合わせ)で、いちばん新しい端を返す。
 * 戻り値: 0 なし、EDGE_SRC_R か EDGE_SRC_L(その端の側)。
 * since: 端から今まで進んだ距離[mm](G_Odo_X - 端の位置)、c: その端からの境目の推定(G_Odo_X)、kind: EDGE_KIND_*。いらなければ NULL。
 * ターン(角速度で履歴を空にした)より前の端は返さない。後ろに下がると G_Odo_X が減るので since も減る */
int Edge_Last(int sides, float *since, float *c, int *kind);

/* 壁切れのログ */
#define EDGE_LOG_FALL 0
#define EDGE_LOG_TH 1
#define EDGE_LOG_ST 2
#define EDGE_LOG_ST_CUT 3
#define EDGE_LOG_CUT 4
#define EDGE_LOG_END 5
#define EDGE_LOG_END_CUT 6
void Edge_Log_Start(int with_thresholds, float border, float speed);
void Edge_Log_Stop();
void Edge_Log_Mark(int type, int info, float x, float a, float b);
void Edge_Log_Print();
int Edge_Log_Has_Data();
float calWallConrol_NANAME();
float calWallConrol_Flontwall_ST();
float calWallConrol_Flontwall_Turn();

#endif /* INC_WALLSENSOR_H_ */
