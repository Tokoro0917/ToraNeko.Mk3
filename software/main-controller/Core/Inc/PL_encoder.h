/*
 * PL_encoder.h
 *
 *  Created on: Jun 14, 2025
 *      Author: akihi
 */

#ifndef INC_PL_ENCODER_H_
#define INC_PL_ENCODER_H_

extern float G_Tire_Speed_R,G_Tire_Speed_L;

void AS5047_DataUpdate();
void Encorder_Speed_Calculate();


void Encorder_count_mode();
void Encorder_count_reset();

int Encorder_number_out();
int Encorder_mode_out() ;

#endif /* INC_PL_ENCODER_H_ */
