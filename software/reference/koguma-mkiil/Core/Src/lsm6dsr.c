/*
 * lsm6dsr.c
 *
 *  Created on: Jun 11, 2025
 *      Author: akihi
 */

#include "lsm6dsr.h"
#include "math.h"
#include "main.h"
#include "spi.h"
#include "stdio.h"
#include<stdint.h>
/*
 @brief spi : read 1 byte
 @param uint8_t Register
 @return read 1byte data
 */

float gyro_z_old = 0;
float gyro_z_offset = 0.2;

float G_Gyro_Z=0;

float accel_x_data[10];

uint8_t read_byte(uint8_t reg) {
	uint8_t ret;
	uint8_t dammy = 0x00;
	uint8_t bureg = (reg | 0x80);

	HAL_GPIO_WritePin(ICM_NSS_GPIO_Port, ICM_NSS_Pin, RESET);
	HAL_SPI_Transmit(&hspi1, &bureg, 1, 100);
	HAL_SPI_TransmitReceive(&hspi1, &dammy, &ret, 1, 100);
	HAL_GPIO_WritePin(ICM_NSS_GPIO_Port, ICM_NSS_Pin, SET);

	return ret;
}

/*
 @brief spi : write 1 byte
 @param uint8_t Register
 @param uint8_t Write Data
 */
void write_byte(uint8_t reg, uint8_t val) {
	uint8_t bureg = reg & 0x7F;

	HAL_GPIO_WritePin(ICM_NSS_GPIO_Port, ICM_NSS_Pin, RESET);
	HAL_SPI_Transmit(&hspi1, &bureg, 1, 100);
	HAL_SPI_Transmit(&hspi1, &val, 1, 100);
	HAL_GPIO_WritePin(ICM_NSS_GPIO_Port, ICM_NSS_Pin, SET);
}

/*
 * @breif initialize mpu 6500
 */

void lsm6dsr_init(void) {
	uint8_t who_am_i;

	HAL_Delay(100); // wait start up
	who_am_i = read_byte(WHO_AM_I); // 1. read who am i
	printf("\r\n0x%x\r\n", who_am_i); // 2. check who am i value

	// 2. error check
//	if (who_am_i != 0xff) {
//		while (1) {
//			printf("gyro_error\r");
//		}
//	}

	HAL_Delay(50); // wait

//	write_byte( PWR_MGMT_1, 0x00); // 3. set pwr_might
//
//	HAL_Delay(50);
//
//	write_byte(MPU6500_RA_CONFIG, 0x00); // 4. set config
//
//	HAL_Delay(50);

	write_byte(CTRL2_G, GYRO_ODR_SET | GYRO_4000_DPS); // 5. set gyro config

	HAL_Delay(50);

	write_byte(CTRL1_XL, ACCEL_ODR_SET | ACCEL_8G); // 6. set accel config

	HAL_Delay(50);

}

float read_gyro_z_axis() {
	float gyro_z;
	int16_t data = (int16_t) ((uint16_t) read_byte(OUTZ_H_G) << 8)
			| (uint16_t) read_byte(OUTZ_L_G);
	gyro_z = (float) ((float) data * (1.0f) * 140.0f / 1000.0f);

	return gyro_z;
}

float read_accel_x_axis() {
	float accel_x = 0;

	int16_t data = (int16_t) ((uint16_t) (read_byte(OUTX_H_A) << 8)
			| (uint16_t) read_byte(OUTX_L_A));
	accel_x = (float) (data * 0.244 / 1000.0f * 9.8);

	return accel_x;
}

float read_accel_y_axis() {
	float accel_y = 0;

	int16_t data = (int16_t) ((uint16_t) (read_byte(OUTY_H_A) << 8)
			| (uint16_t) read_byte(OUTY_L_A));
	accel_y = (float) (data * 0.244 / 1000.0f * 9.8);
	return accel_y;
}

float read_accel_z_axis() {
	float accel_z = 0;

	int16_t data = (int16_t) ((uint16_t) (read_byte(OUTZ_H_A) << 8)
			| (uint16_t) read_byte(OUTZ_L_A));
	accel_z = (float) (data * 0.244 / 1000.0f * 9.8);
	return accel_z;
}

float read_NoiseCut_gyro_z() {

	float gyro_z = read_gyro_z_axis();

	float z = 1.01*(gyro_z * 0.95 + gyro_z_old * 0.05) ;//- gyro_z_offset

	gyro_z_old = gyro_z;

	if(fabs(z)<0.5){
		z=0;
	}

	return z;
}

float read_average_acc_x() {
	float sum = 0;
	accel_x_data[0] = read_accel_x_axis();

	for (int i = 9; i > 0; i--) {
		accel_x_data[i] = accel_x_data[i - 1];
	}

	for (int i = 0; i < 10; i++) {
		sum += accel_x_data[i];
	}

	return sum/10;

}
