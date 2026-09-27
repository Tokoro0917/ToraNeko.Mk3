/*
 * lsm6dsr.h
 *
 *  Created on: Jun 11, 2025
 *      Author: akihi
 */

#ifndef INC_LSM6DSR_H_
#define INC_LSM6DSR_H_

#include<stdint.h>

#define WHO_AM_I 0x0f	//RETURN Value is 0x6B

//gyro output register
#define OUTX_L_G 0x22
#define OUTX_H_G 0x23
#define OUTY_L_G 0x24
#define OUTY_H_G 0x25
#define OUTZ_L_G 0x26
#define OUTZ_H_G 0x27

//accel output register
#define OUTX_L_A 0x28
#define OUTX_H_A 0x29
#define OUTY_L_A 0x2A
#define OUTY_H_A 0x2B
#define OUTZ_L_A 0x2C
#define OUTZ_H_A 0x2D

//accelerometer control register
#define CTRL1_XL 0x10
//gyroscope control register
#define CTRL2_G  0x11

#define ACCEL_ODR_SET 0x80
#define ACCEL_8G	  0x0E
#define ACCEL_4G	  0x08

#define GYRO_ODR_SET 0x80
#define GYRO_2000_DPS 0x0C
#define GYRO_4000_DPS 0x01

uint8_t read_byte(uint8_t);
void write_byte(uint8_t, uint8_t);
void lsm6dsr_init(void);

float read_gyro_x_axis();
float read_gyro_y_axis();
float read_gyro_z_axis();
float read_accel_x_axis();
float read_accel_y_axis();
float read_accel_z_axis();

float read_NoiseCut_gyro_z();

float read_average_acc_x();

extern float G_Gyro_Z;

#endif /* INC_LSM6DSR_H_ */
