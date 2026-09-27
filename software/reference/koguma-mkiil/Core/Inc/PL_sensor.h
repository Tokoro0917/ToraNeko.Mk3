
/* * PL_sensor.h
 *
 *  Created on: Jun 1, 2022
 *      Author: sf199
 */

#ifndef INC_PL_SENSOR_H_
#define INC_PL_SENSOR_H_

#include "stm32f4xx_hal.h"

extern uint16_t g_ADCBuffer[5];
extern int16_t g_sensor[4][10];
extern uint16_t g_sensor_on[4];
extern uint16_t g_sensor_off[4];
extern float g_V_batt;

extern int16_t g_sensor_av[4];
extern int16_t g_sensor_av10[4];


float pl_getbatt();

void pl_callback_getSensor();

void pl_interupt_getSensor();

#endif /* INC_PL_SENSOR_H_ */
