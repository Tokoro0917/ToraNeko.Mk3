/*
 * UI.h
 *
 *  Created on: May 30, 2024
 *      Author: akihi
 */

#ifndef INC_UI_H_
#define INC_UI_H_

void LED_Reset();

void LED_ALL_ON();

void LED_Setup_Robot();
void LED_batt_error();

void LED_Goal();

void LED_program_number (int );
void LED_program_mode(int );

void LED_ON_L();
void LED_ON_R();
void LED_StartWait();

void Buzzer_Number_Change();

void Buzzer_Mode_Change() ;

void Buzzer_Enter();
void Buzzer_Start();

#endif /* INC_UI_H_ */
