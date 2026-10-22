/*
 * imu.h
 * ジャイロと加速度の読み取り（LSM6DSR、SPI1）
 *
 * 使うのは、上下の軸まわりの角速度（機体の向きの変わる速さ）と、前後・左右の加速度。
 * 制御周期（0.5ms）ごとに imu_update() で読み、角速度を足して角度を求める。
 */
#ifndef DRV_IMU_H
#define DRV_IMU_H

#include <stdbool.h>
#include <stdint.h>

bool imu_init(void);
void imu_update(void);
uint8_t imu_get_who_am_i(void);

/* ゼロ点（止まっているときに出る角速度のずれ）を測り終えたら true。測り終えるまで、角度は 0 のまま */
bool imu_is_offset_ready(void);
float imu_get_gyro_offset_dps(void);

/* 角速度 [deg/s]（ゼロ点を引いたもの。上から見て左回り（反時計回り）が +） */
float imu_get_gyro_z_dps(void);
/* ゼロ点を測り終えてからの角度 [deg]（左回りが +） */
float imu_get_angle_deg(void);
/* 加速度 [m/s²]。x・y がどちら向きかは、ステップ7 で使うときに実機で確かめる */
float imu_get_accel_x_mps2(void);
float imu_get_accel_y_mps2(void);

#endif /* DRV_IMU_H */
