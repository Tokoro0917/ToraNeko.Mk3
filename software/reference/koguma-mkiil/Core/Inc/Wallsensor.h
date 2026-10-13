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
/* 境目の推定が [lo, hi](G_Odo_X)にあれば1。左右にあれば平均。src は EDGE_SRC_*(PillarEdge.h) */
int Edge_Cut_Find(float lo, float hi, float *c, int *src);

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
