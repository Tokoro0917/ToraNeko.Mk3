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
/* ジャイロのゼロ点[dps]。走る前に止まって測る(Gyro_Calibrate、Motor_Setup から呼ぶ)。2026-10-19 まではここが 0.2 の決め打ちで、
 * しかも引く処理がコメントアウトされ、±0.5 dps の不感帯でごまかしていた(docs/search_stability_research.md)。
 * LSM6DSR のゼロ点のずれは typ ±1 dps。引かないと、横壁のない直線で向きが流れる */
float gyro_z_offset = 0;
float Gyro_Deadband = 0.15f;	//ゼロ点を測ったあと、これより小さい角速度は0にする[dps](1 LSB = 0.14 dps)
#define GYRO_DEADBAND_UNCAL 0.5f	//まだ一度も測れていないときの不感帯(以前と同じ)
#define GYRO_CAL_SD_MAX 1.5f	//ばらつきがこれより大きければ測り直さない[dps](手で触っている、吸引を回している。吸引中は 4〜5 dps)
#define GYRO_CAL_MEAN_MAX 5.0f	//平均がこれより大きければ測り直さない[dps](止まっていない)
float G_Gyro_Offset_SD = -1;	//ゼロ点を決めたときのばらつき[dps](-1 はまだ測れていない)
int G_Gyro_Cal_Reject = 0;	//測ったが使わなかった回数(電源を入れてから)
float G_Gyro_Cal_SD[GYRO_CAL_TRY_N];	//最近 GYRO_CAL_TRY_N 回の、測ったときのばらつき[dps](使ったかどうかによらない)
float G_Gyro_Cal_Mean[GYRO_CAL_TRY_N];	//同じく平均[dps]
int G_Gyro_Cal_Count = 0;	//測った回数(電源を入れてから)
volatile uint32_t G_Gyro_Torn = 0;	//上位と下位が別の値の組み合わせになりかけて、読み直した回数(電源を入れてから)
static int gyro_calibrated = 0;
static volatile float gyro_z_filt = 0;	//フィルタのあと、ゼロ点を引く前の値(Gyro_Calibrate が平均する)

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
	/* 上位と下位を別々に読むので、そのあいだにジャイロが次の値に替わると、上位と下位が別の値の組み合わせになる
	 * (0 の近くで符号が変わると ±256 LSB = ±35.8 dps の飛び。2026-10-20、止まっていても約1%の割合で出ていて、
	 * Gyro_Calibrate のばらつきが閾値を超えていた)。上位を読み直し、替わっていたら下位も読み直す */
	uint8_t h = read_byte(OUTZ_H_G);
	uint8_t l = read_byte(OUTZ_L_G);
	uint8_t h2 = read_byte(OUTZ_H_G);
	if (h2 != h) {
		l = read_byte(OUTZ_L_G);
		h = h2;
		G_Gyro_Torn++;
	}
	int16_t data = (int16_t) (((uint16_t) h << 8) | l);
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

	float f = gyro_z * 0.95 + gyro_z_old * 0.05;
	gyro_z_filt = f;
	float z = 1.01 * (f - gyro_z_offset);

	gyro_z_old = gyro_z;

	if (fabs(z) < (gyro_calibrated ? Gyro_Deadband : GYRO_DEADBAND_UNCAL)) {
		z = 0;
	}

	return z;
}

/* ジャイロのゼロ点を測る。止まっているときに呼ぶ(1msごとに割り込みが読む値を n 回平均する。n ms かかる)。
 * ばらつきや平均が大きいとき(動いている、手で触っている、吸引を回している)は、ゼロ点を変えない(前に測った値のまま)。
 * 戻り値: 1 測った、0 変えなかった */
int Gyro_Calibrate(int n) {
	float sum = 0, sq = 0;
	for (int i = 0; i < n; i++) {
		uint32_t t = HAL_GetTick();
		while (HAL_GetTick() == t) {	//次の1msまで待つ(HAL_Delay(1) は2ms待つ)
		}
		float v = gyro_z_filt;
		sum += v;
		sq += v * v;
	}
	float mean = sum / n;
	float var = sq / n - mean * mean;
	float sd = var > 0 ? sqrtf(var) : 0;
	G_Gyro_Cal_SD[G_Gyro_Cal_Count % GYRO_CAL_TRY_N] = sd;
	G_Gyro_Cal_Mean[G_Gyro_Cal_Count % GYRO_CAL_TRY_N] = mean;
	G_Gyro_Cal_Count++;
	if (sd > GYRO_CAL_SD_MAX || fabsf(mean) > GYRO_CAL_MEAN_MAX) {
		G_Gyro_Cal_Reject++;
		return 0;
	}
	gyro_z_offset = mean;
	G_Gyro_Offset_SD = sd;
	gyro_calibrated = 1;
	return 1;
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
