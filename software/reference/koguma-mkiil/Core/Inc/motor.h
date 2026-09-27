/*
 * motor.h
 *
 *  Created on: Jun 12, 2025
 *      Author: akihi
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

extern float G_Robot_speed;

extern float G_Motor_X;
extern float G_Motor_Ac;
extern float G_Motor_V_Target;

extern float G_Motor_Angle;
extern float G_Motor_W_Ac;
extern float G_Motor_W_Target;

extern float Motor_Voltage_L;
extern float Motor_Voltage_R;

extern int G_Motor_Flag;
extern float G_Motor_Count;

void Motor_PWM_Generate();
void Motor_Setup();
void Motor_Setup_Voltage();
void Motor_Free();
void Motor_Stop();
void Motor_Back();

void Suction_Start();
void Fun_Flag_OFF();
void Suction_change(int);
void Suction_Stop();

void Motor_Speed_PID_ST(int, int);
void Motor_Speed_PID_Turn(float, float);
void Motor_Sula_COS(float , float, float, float);

void Motor_Robot_Alignment();

void Motor_trapezoid(float, float, float, float, float);
void Motor_trapezoid_PID(float, float, float, float, float);
void Motor_NANAME_PID(float, float, float, float, float);
void Motor_Multistage_PID(float, float, float, float, float);
void Motor_trapezoid_Asymmetric_PID(float, float, float, float, float);
void Motor_Sula_before(float, float , float , float , float);
void Motor_Sula_ST(float, float , float , float , float);
void Motor_Wallcut_ST(float , float, int );
void Motor_Wallcut_END(float , float, int );
void Motor_Wallcut_ST_NANAME(float , float, int );
void Motor_Wallcut_END_NANAME(float , float, int );


void Motor_trapezoid_Turn(float, float, float);

#endif /* INC_MOTOR_H_ */
