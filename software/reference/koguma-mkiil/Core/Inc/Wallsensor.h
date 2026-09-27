/*
 * Wallsensor.h
 *
 *  Created on: Jun 21, 2024
 *      Author: akihi
 */

#ifndef INC_WALLSENSOR_H_
#define INC_WALLSENSOR_H_

extern int G_Wall_data[];

void Wall_search();

float Wall_Flont_Av();

int Sensor_Enter();
int Sensor_Start();

void Wall_search_LED();

float calWallConrol();
float calWallConrol_NANAME();
float calWallConrol_Flontwall_ST();
float calWallConrol_Flontwall_Turn();

#endif /* INC_WALLSENSOR_H_ */
