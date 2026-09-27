/*
 * PL_timer.c
 *
 *  Created on: May 15, 2024
 *      Author: akihi
 */

#include "PL_timer.h"
#include "tim.h"

volatile uint32_t g_timCount;

void pl_timer_init(void) {
	g_timCount = 0;
	HAL_TIM_Base_Start_IT(&htim6);
}

void pl_timer_count(void) {
	g_timCount++;
}

void wait_ms(uint32_t wait_time) {

	g_timCount = 0;
	__HAL_TIM_SET_COUNTER(&htim6, 0);
	while (g_timCount < wait_time) {
	}

}
