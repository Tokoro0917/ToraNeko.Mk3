/*
 * motor.h
 * 走行モーターの出力（MP6551 ×2、TIM4 の PWM）
 */
#ifndef DRV_MOTOR_H
#define DRV_MOTOR_H

void motor_init(void);
void motor_set_voltage(float left_v, float right_v);
void motor_stop(void);

#endif /* DRV_MOTOR_H */
