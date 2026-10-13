/*
 * LOG.h
 *
 *  Created on: Jun 30, 2024
 *      Author: akihi
 */

#ifndef INC_LOG_H_
#define INC_LOG_H_

void LOG_get_start();
void LOG_get_end();
void LOG_get_interrupt();
void LOG_print();
int LOG_Is_On();
int LOG_Has_Data();

/* 何回に1回残すか(普段は1、最短走行は4) */
extern int G_Log_Div;

#endif /* INC_LOG_H_ */
