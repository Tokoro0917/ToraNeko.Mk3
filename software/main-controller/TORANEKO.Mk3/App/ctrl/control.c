/*
 * control.c
 * 速度と角度の制御：目標の速さ・向きになるように、モーターにかける電圧を決める
 *
 * ■ 流れ（制御周期 0.5ms ごと。sys/scheduler.c から control_update() が呼ばれる）
 *   1. 目標の速さを 1 周期ぶん進める（ctrl/trajectory.c）
 *   2. 前に進む分の電圧を決める
 *        フィードフォワード：目標の速さ × FF_SPEED + 目標の加速度 × FF_ACCEL（+ 摩擦の分）
 *        フィードバック    ：速さのずれ × SPEED_KP + 位置のずれ × SPEED_KI
 *   3. 向きの分の電圧を決める（直進のときは目標の角速度 0、その場旋回・スラロームのときは ctrl/trajectory.c の目標の角速度）
 *        フィードフォワード：目標の角速度・角加速度を、タイヤ 1 つの速さ・加速度に直して、2 と同じ係数をかける
 *                            （角速度 [rad/s] × トレッド幅の半分 = タイヤ 1 つの速さ [mm/s]）
 *        フィードバック    ：角速度のずれ × OMEGA_KP + 角度のずれ × ANGLE_KI
 *      横の壁に沿って走る補正が入っているときは、直進中の目標の角速度に、横の壁からのずれで決めた分を足す
 *      （ずれ × WALL_KP + ずれの変わる速さ × WALL_KD。左の壁に近ければ右へ曲げる）
 *   4. 左 = 前に進む分 − 向きの分、右 = 前に進む分 + 向きの分 をモーターにかける
 *      （左回りが + なので、左に向けたいときは右を速く、左を遅くする）
 *   5. 走行ログに 1 行渡す（sys/log.c が、決めた間隔ごとに残す）
 *   6. 位置か角度のずれが大きすぎたら、フェイルセーフで止める
 * 係数は config/ctrl_gains.h にある。
 *
 * ■ 前壁合わせ（control_set_front_align(true) のあいだ）
 * 目標の速さ・角速度を、ctrl/trajectory.c ではなく、前の壁からのずれで決める（ctrl/wall_ctrl.c）。
 *   目標の速さ   = −距離のずれ × ALIGN_SPEED_GAIN（近すぎれば下がる）
 *   目標の角速度 =  向きのずれ × ALIGN_OMEGA_GAIN（右を向いていれば左へ回す）
 * あとは、いつもと同じように速さと角速度の制御で電圧を決める。
 *
 * ■ ずれの足し合わせ
 * 「速さのずれ × 0.5ms」を足していくと、目標の位置と本当の位置のずれ [mm] になる。
 * 同じように「角速度のずれ × 0.5ms」を足すと、角度のずれ [deg] になる。
 * 速さのずれだけで電圧を決めると、少し遅れたぶんを取り戻せないので、足し合わせたずれも使う（I の項）。
 *
 * ■ 割り込み
 * control_start()・control_stop() はメインの処理から呼ぶ。値を書いているあいだに割り込みが
 * 中途半端な値を使わないように、先に「動いていない」にしてから値を書き、最後に「動いている」にする。
 */
#include "ctrl/control.h"

#include "config/ctrl_gains.h"
#include "config/robot_params.h"
#include "ctrl/odometry.h"
#include "ctrl/trajectory.h"
#include "ctrl/wall_ctrl.h"
#include "drv/imu.h"
#include "drv/motor.h"
#include "drv/wall.h"
#include "sys/failsafe.h"
#include "sys/log.h"

#define DEG_TO_RAD (3.14159265f / 180.0f)  /* 度をラジアンに直すときにかける数 */

/* 割り込みの中で書き換える（volatile の理由は sys/scheduler.c の先頭のコメント） */
static volatile bool s_running = false;
static volatile float s_position_error_mm = 0.0f;  /* 位置のずれ（速さのずれの足し合わせ） */
static volatile float s_angle_error_deg = 0.0f;    /* 角度のずれ（角速度のずれの足し合わせ） */
static volatile float s_start_angle_deg = 0.0f;    /* 始めたときの向き */
static volatile float s_left_voltage_v = 0.0f;     /* 最後にかけた電圧（ログ用） */
static volatile float s_right_voltage_v = 0.0f;
static volatile bool s_wall_control = false;        /* 横の壁に沿って走る補正を入れるか */
static volatile bool s_front_align = false;         /* 前壁合わせをしているか */
static volatile float s_align_speed_mm_s = 0.0f;    /* 前壁合わせの目標の速さ（ログ用） */
static volatile float s_align_omega_deg_s = 0.0f;   /* 前壁合わせの目標の角速度（ログ用） */
static volatile float s_wall_error = 0.0f;          /* 横の壁からのずれ（ゆれを減らしたもの。ログ用） */
static volatile float s_wall_error_rate = 0.0f;     /* ずれの変わる速さ [1/s] */
static volatile float s_wall_omega_deg_s = 0.0f;    /* 補正で曲げた角速度（ログ用） */
static volatile int s_wall_used = 0;                /* 使った壁（ログ用） */

#define WALL_FILTER_RATIO 0.05f  /* ずれのゆれを減らす割合（1 周期で新しい値に 5% 近づける。約 10ms で落ち着く） */

/* 制御を始める（止まっているところから）。今の向きを、まっすぐの向きにする */
void control_start(void)
{
  s_running = false;  /* 書いているあいだは、割り込みに使わせない */
  trajectory_reset();
  s_position_error_mm = 0.0f;
  s_angle_error_deg = 0.0f;
  s_start_angle_deg = imu_get_angle_deg();
  s_running = true;   /* 最後に、動いているにする */
}

/* 横の壁に沿って走る補正を入れる（true）・入れない（false）。探索（ctrl/run.c）で入れる */
void control_set_wall_control(bool on)
{
  s_wall_control = on;
}

/* 前壁合わせを始める（true）・やめる（false）。止まっているときに使う（ctrl/run.c） */
void control_set_front_align(bool on)
{
  s_front_align = on;
}

/* value を -limit〜limit の中にする */
static float limit_value(float value, float limit)
{
  if (value > limit) {
    return limit;
  }
  if (value < -limit) {
    return -limit;
  }
  return value;
}

/* 前壁合わせの、目標の速さと角速度を、前の壁からのずれで決める */
static void update_align_target(void)
{
  float speed = -wall_ctrl_get_front_distance_error() * ALIGN_SPEED_GAIN;
  float omega = wall_ctrl_get_front_angle_error() * ALIGN_OMEGA_GAIN;
  s_align_speed_mm_s = limit_value(speed, ALIGN_SPEED_LIMIT_MM_S);
  s_align_omega_deg_s = limit_value(omega, ALIGN_OMEGA_LIMIT_DEG_S);
}

/* 今の目標の速さ [mm/s]（前壁合わせのあいだは、前の壁で決めた値） */
static float target_speed_mm_s(void)
{
  if (s_front_align) {
    return s_align_speed_mm_s;
  }
  return trajectory_get_speed_mm_s();
}

/* 今の目標の角速度 [deg/s] */
static float target_omega_deg_s(void)
{
  if (s_front_align) {
    return s_align_omega_deg_s;
  }
  return trajectory_get_omega_deg_s();
}

/* 今の目標の加速度 [mm/s²]・角加速度 [deg/s²]（前壁合わせのあいだは 0） */
static float target_accel_mm_s2(void)
{
  if (s_front_align) {
    return 0.0f;
  }
  return trajectory_get_accel_mm_s2();
}

static float target_alpha_deg_s2(void)
{
  if (s_front_align) {
    return 0.0f;
  }
  return trajectory_get_alpha_deg_s2();
}

/* 制御をやめて、モーターを止める */
void control_stop(void)
{
  s_running = false;
  motor_stop();
}

/* 制御の途中か */
bool control_is_running(void)
{
  return s_running;
}

/* 摩擦の分の電圧 [V]。目標の速さが + なら +friction_v、- なら -friction_v、0 なら 0 */
static float friction_voltage(float target_speed, float friction_v)
{
  if (target_speed > 0.0f) {
    return friction_v;
  }
  if (target_speed < 0.0f) {
    return -friction_v;
  }
  return 0.0f;
}

/* 前に進む分の電圧 [V] を決める */
static float forward_voltage(void)
{
  float target_speed = target_speed_mm_s();
  float speed_error_mm_s = target_speed - odometry_get_speed_mm_s();
  s_position_error_mm = s_position_error_mm + speed_error_mm_s * CTRL_PERIOD_S;

  float feedforward_v = target_speed * FF_SPEED_V_PER_MM_S
                      + target_accel_mm_s2() * FF_ACCEL_V_PER_MM_S2
                      + friction_voltage(target_speed, FF_FRICTION_V);

  float feedback_v = speed_error_mm_s * SPEED_KP + s_position_error_mm * SPEED_KI;
  return feedforward_v + feedback_v;
}

/* 横の壁からのずれを読み、ゆれを減らして、変わる速さも求める。使う壁が変わったときは、そこから測り直す
 * （壁が急に見えたり見えなくなったりしたときに、変わる速さが大きく出ないように） */
static void update_wall_error(void)
{
  float raw_error = wall_ctrl_get_side_error();
  int used = wall_ctrl_get_side_walls_used();
  if (used != s_wall_used) {
    s_wall_used = used;
    s_wall_error = raw_error;
    s_wall_error_rate = 0.0f;
    return;
  }
  float last_error = s_wall_error;
  s_wall_error = s_wall_error + (raw_error - s_wall_error) * WALL_FILTER_RATIO;
  s_wall_error_rate = (s_wall_error - last_error) / CTRL_PERIOD_S;
}

/* 横の壁に沿って走る補正で、目標の角速度に足す分 [deg/s]（+ なら左へ曲げる）。
 * 補正を入れていないとき、直進でないとき、遅いときは 0 */
static float wall_omega_deg_s(void)
{
  update_wall_error();
  if (!s_wall_control || trajectory_get_kind() != TRAJECTORY_STRAIGHT
      || trajectory_get_speed_mm_s() < WALL_CONTROL_MIN_SPEED_MM_S) {
    s_wall_omega_deg_s = 0.0f;
    return 0.0f;
  }
  float omega = -(s_wall_error * WALL_KP + s_wall_error_rate * WALL_KD);  /* 左の壁に近い（+）なら右（-）へ */
  if (omega > WALL_OMEGA_LIMIT_DEG_S) {
    omega = WALL_OMEGA_LIMIT_DEG_S;
  }
  if (omega < -WALL_OMEGA_LIMIT_DEG_S) {
    omega = -WALL_OMEGA_LIMIT_DEG_S;
  }
  s_wall_omega_deg_s = omega;
  return omega;
}

/* 向きの分の電圧 [V] を決める（+ なら左に向ける。右のタイヤに +、左のタイヤに - でかける） */
static float turn_voltage(void)
{
  float target_omega = target_omega_deg_s();
  float omega_error_deg_s = target_omega + wall_omega_deg_s() - imu_get_gyro_z_dps();
  s_angle_error_deg = s_angle_error_deg + omega_error_deg_s * CTRL_PERIOD_S;

  /* 目標の角速度・角加速度で回るときの、タイヤ 1 つの速さ [mm/s]・加速度 [mm/s²] */
  float wheel_speed_mm_s = target_omega * DEG_TO_RAD * (TREAD_MM / 2.0f);
  float wheel_accel_mm_s2 = target_alpha_deg_s2() * DEG_TO_RAD * (TREAD_MM / 2.0f);
  float feedforward_v = wheel_speed_mm_s * FF_SPEED_V_PER_MM_S
                      + wheel_accel_mm_s2 * FF_ACCEL_V_PER_MM_S2;
  /* 摩擦の分は、その場旋回のときだけ足す（左右のタイヤが逆に回り、床をこするので摩擦が大きい）。
   * スラロームでは左右とも前に回っていて、摩擦の分は前に進む分（forward_voltage）に入っている */
  if (trajectory_get_kind() == TRAJECTORY_PIVOT) {
    feedforward_v = feedforward_v + friction_voltage(wheel_speed_mm_s, FF_TURN_FRICTION_V);
  }

  float feedback_v = omega_error_deg_s * OMEGA_KP + s_angle_error_deg * ANGLE_KI;
  return feedforward_v + feedback_v;
}

/* 走行ログに、今の値を 1 行渡す */
static void record_log(void)
{
  log_values_t values = {0};  /* {0} で、全部の値を 0 にしてから入れる（壁の制御の値は、まだないので 0 のまま） */
  values.distance_mm = odometry_get_distance_mm();
  values.speed_mm_s = odometry_get_speed_mm_s();
  values.speed_target_mm_s = target_speed_mm_s();
  values.omega_deg_s = imu_get_gyro_z_dps();
  values.omega_target_deg_s = target_omega_deg_s() + s_wall_omega_deg_s;
  values.wall_used = s_wall_used;
  values.wall_error = s_wall_error;
  values.wall_p = s_wall_error * WALL_KP;
  values.wall_d = s_wall_error_rate * WALL_KD;
  values.angle_deg = imu_get_angle_deg() - s_start_angle_deg;
  values.sensor_left = wall_ctrl_get_left();
  values.sensor_right = wall_ctrl_get_right();
  values.voltage_left_v = s_left_voltage_v;
  values.voltage_right_v = s_right_voltage_v;
  values.battery_v = wall_get_battery_v();
  if (s_front_align) {
    values.phase = LOG_PHASE_ALIGN;
  } else if (trajectory_get_kind() == TRAJECTORY_PIVOT) {
    values.phase = LOG_PHASE_PIVOT;
  } else if (trajectory_get_kind() == TRAJECTORY_SLALOM) {
    values.phase = LOG_PHASE_TURN;
  } else {
    values.phase = LOG_PHASE_STRAIGHT;
  }
  log_record(values);
}

/* 位置か角度のずれが大きすぎたら true（ぶつかった、モーターの向きが逆、など） */
static bool error_too_large(void)
{
  if (s_position_error_mm > POSITION_ERROR_LIMIT_MM || s_position_error_mm < -POSITION_ERROR_LIMIT_MM) {
    return true;
  }
  if (s_angle_error_deg > ANGLE_ERROR_LIMIT_DEG || s_angle_error_deg < -ANGLE_ERROR_LIMIT_DEG) {
    return true;
  }
  return false;
}

/* 制御周期ごとに呼ぶ（流れは先頭のコメント）。制御の途中でなければ何もしない */
void control_update(void)
{
  if (!s_running) {
    return;
  }
  if (failsafe_is_stopped()) {
    s_running = false;  /* どこかでフェイルセーフになったら、制御もやめる（モーターは failsafe_stop() が止めている） */
    return;
  }

  if (s_front_align) {
    update_align_target();  /* 前壁合わせのあいだは、目標を前の壁で決める */
  } else {
    trajectory_update();
  }
  float forward_v = forward_voltage();
  float turn_v = turn_voltage();
  s_left_voltage_v = forward_v - turn_v;
  s_right_voltage_v = forward_v + turn_v;
  motor_set_voltage(s_left_voltage_v, s_right_voltage_v);
  record_log();

  if (error_too_large()) {
    s_running = false;
    failsafe_stop();  /* モーターを止めて、フェイルセーフで止まった状態にする */
  }
}

/* 位置のずれ [mm]（目標 − 測った値。+ なら目標より遅れている） */
float control_get_position_error_mm(void)
{
  return s_position_error_mm;
}

/* 角度のずれ [deg]（目標 − 測った値。+ なら目標より右を向いている＝左回りが足りない） */
float control_get_angle_error_deg(void)
{
  return s_angle_error_deg;
}
