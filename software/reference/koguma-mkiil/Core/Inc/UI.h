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
void LED_Loading();

/* スイッチ(基板の S2、PB3。押すと 0) */
#define SW_Pin GPIO_PIN_3
#define SW_GPIO_Port GPIOB
void Switch_Init();
int Switch_Pressed();

void LED_Setup_Robot();
void LED_batt_error();

void LED_Goal();
void LED_Path_Error();	//最短経路が作れなかった

void LED_program_number (int );
void LED_program_mode(int );

void LED_ON_L();
void LED_ON_R();
void LED_StartWait();

void Buzzer_Number_Change();

void Buzzer_Mode_Change() ;

void Buzzer_Enter();
void Buzzer_Start();
void Buzzer_Finish();	//全部走り終えてスタートに戻ったときの曲

#endif /* INC_UI_H_ */
