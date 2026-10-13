/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "lsm6dsr.h"
#include "motor.h"
#include "PL_encoder.h"
#include "PL_timer.h"
#include "Motor.h"
#include "LOG.h"
#include "PL_sensor.h"
#include "UI.h"
#include "Wallsensor.h"
#include "Maze.h"
#include "Move.h"
#include "Failsafe.h"
#include "Define.h"
#include "SensorNoise.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Maze_Search() の探索の組み合わせ(モード0) */
#define SEARCH_SLALOM		0x00	//スラローム探索(500mm/s、吸引なし)
#define SEARCH_SUCTION		0x01	//吸引探索(1000mm/s、吸引あり)
#define SEARCH_ALL			0x02	//復路でエセ全面探索
#define SEARCH_ONEWAY		0x04	//片道: ゴールで止まる
#define SEARCH_KNOWN		0x08	//既知区間加速(探索済みの区間はまとめて速く走る)
#define SEARCH_DIJKSTRA		0x10	//ゴール後にダイクストラ経路上の未確認の壁を見に行く
#define SEARCH_SHORTEST		0x20	//スタートに戻ったあと最短走行(モード1 No.5と同じ)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* 1 にすると、最短走行(モード1、モード0 No.8〜10 の探索後の最短)の壁切れを柱で見る
 * (区画の境目の柱の端から境目の位置を求め、前距離を直す。motor.c の G_WallCut_Pillar、
 * 設計は docs/pillar_cut_design.md)。0 なら従来どおり。探索中と斜めの壁切れはいつも従来どおり */
static int Short_WallCut_Pillar = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void Maze_Search(int flags);
static void Maze_Shortest_Run(void (*run)(int, int), int max, int ac);
static void Output_All(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_SPI1_Init();
	MX_SPI3_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	MX_USART2_UART_Init();
	MX_TIM6_Init();
	/* USER CODE BEGIN 2 */

	//HAL_TIM_Base_Start_IT(&htim6);
	pl_timer_init();
	lsm6dsr_init();
	Maze_Initialization();
	if (g_V_batt < LIMITBATT) {
		while (1) {
			LED_Reset();
			HAL_Delay(500);
			LED_ALL_ON();
			HAL_Delay(500);
		}
	}

	LED_Setup_Robot();

//
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4); //buzeer
//
//	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		LED_program_number(Encorder_number_out());
		LED_program_mode(Encorder_mode_out());
		Maze_Unkown_ALL_ModeOFF();
		Motor_Setup_Voltage();
		if (Encorder_mode_out() == 0) {		//大会用
			/* 探索の組み合わせは Maze_Search() の引数で指定する(下の SEARCH_*)。
			 * No.0 と No.11〜15 は何もしない */
			int n = Encorder_number_out();
			int flags = -1;
			if (n == 1) {			//スラローム探索 往復
				flags = SEARCH_SLALOM;
			} else if (n == 2) {	//スラローム探索+エセ全面探索
				flags = SEARCH_SLALOM | SEARCH_ALL;
			} else if (n == 3) {	//スラローム探索 片道(ゴールで止まる)
				flags = SEARCH_SLALOM | SEARCH_ONEWAY;
			} else if (n == 4) {	//吸引探索 往復
				flags = SEARCH_SUCTION;
			} else if (n == 5) {	//吸引探索+エセ全面探索
				flags = SEARCH_SUCTION | SEARCH_ALL;
			} else if (n == 6) {	//ダイクストラ誘導の未知壁探索
				flags = SEARCH_SUCTION | SEARCH_DIJKSTRA;
			} else if (n == 7) {	//ダイクストラ誘導の未知壁探索+既知区間加速
				flags = SEARCH_SUCTION | SEARCH_DIJKSTRA | SEARCH_KNOWN;
			} else if (n == 8) {	//スラローム探索 往復 → 最短
				flags = SEARCH_SLALOM | SEARCH_SHORTEST;
			} else if (n == 9) {	//吸引探索 往復 → 最短
				flags = SEARCH_SUCTION | SEARCH_SHORTEST;
			} else if (n == 10) {	//ダイクストラ誘導+既知区間加速 → 最短
				flags = SEARCH_SUCTION | SEARCH_DIJKSTRA | SEARCH_KNOWN
						| SEARCH_SHORTEST;
			}
			if ((flags >= 0) && (Sensor_Enter() == 1)) {
				Maze_Search(flags);
			}
		} else if (Encorder_mode_out() == 1) {		//�?短
			/* 最短走行。No.0 と No.12〜15 は何もしない */
			int n = Encorder_number_out();
			void (*run)(int, int) = NULL;
			int ac = 0;
			int ac_nn = 0;	//斜めの直線の加速度(0 なら既定: ac-5000)
			if (n == 1) {			//BFS コーナー2000 直線6000 加速15000
				run = Short_NANAME_Move2000;
				ac = 15000;
				ac_nn = 10000;
			} else if (n == 2) {	//BFS コーナー2000 直線6000 加速20000
				run = Short_NANAME_Move2000;
				ac = 20000;
				ac_nn = 15000;
			} else if (n == 3) {	//BFS コーナー2400 直線6000 加速20000
				run = Short_NANAME_Move2400;
				ac = 20000;
				ac_nn = 15000;
			} else if (n == 4) {	//ダイクストラ コーナー2000 直線6000 加速15000
				run = Short_Dijkstra_Move2000;
				ac = 15000;
				ac_nn = 10000;
			} else if (n == 5) {	//ダイクストラ コーナー2000 直線6000 加速20000
				run = Short_Dijkstra_Move2000;
				ac = 20000;
				ac_nn = 15000;
			} else if (n == 6) {	//ダイクストラ コーナー2000 直線6000 加速23000
				run = Short_Dijkstra_Move2000;
				ac = 23000;
				ac_nn = 18000;
			} else if (n == 7) {	//ダイクストラ コーナー2000 直線6000 加速25000
				run = Short_Dijkstra_Move2000;
				ac = 25000;
				ac_nn = 20000;
			} else if (n == 8) {	//ダイクストラ コーナー2400 直線6000 加速20000
				run = Short_Dijkstra_Move2400;
				ac = 20000;
				ac_nn = 15000;
			} else if (n == 9) {	//ダイクストラ コーナー2400 直線6000 加速23000
				run = Short_Dijkstra_Move2400;
				ac = 23000;
				ac_nn = 18000;
			} else if (n == 10) {	//BFS ターンごとの通過速度(2400基準) 直線6000 加速20000
				run = Short_NANAME_MoveTurnV;
				ac = 20000;
				ac_nn = 15000;
			} else if (n == 11) {	//ダイクストラ ターンごとの通過速度(2400基準) 直線6000 加速20000
				run = Short_Dijkstra_MoveTurnV;
				ac = 20000;
				ac_nn = 15000;
			}
			if ((run != NULL) && (Sensor_Enter() == 1)) {
				G_Short_AC_NANAME = ac_nn;
				Maze_Shortest_Run(run, 6000, ac);
				G_Short_AC_NANAME = 0;
			}
		} else if (Encorder_mode_out() == 2) {		//サーキ�?�?

		} else if (Encorder_mode_out() == 3) {		//�?バッグ用
			if (Encorder_number_out() == 1) {		//2m/s
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();

					Motor_Setup();
					Motor_Stop();
					Short_NANAME_Move2000(2000, 20000);
					Motor_Free();
					LED_Reset();
					HAL_Delay(500);
					Encorder_count_reset();
				}
			} else if (Encorder_number_out() == 2) {		//
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();

					Motor_Setup();
					Motor_Stop();
					Short_NANAME_Move2400(2400, 20000);
					Motor_Free();
					LED_Reset();
					HAL_Delay(500);
					Encorder_count_reset();
				}
			} else if (Encorder_number_out() == 3) {		//
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();

					Motor_Setup();
					Motor_Stop();
					Short_NANAME_Move2700(2700, 20000);
					Motor_Free();
					LED_Reset();
					HAL_Delay(500);
					Encorder_count_reset();
				}
			} else if (Encorder_number_out() == 4) {		//距離キャリブレーション
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();
					Wall_Distance_Calibration();
				}
			} else if (Encorder_number_out() == 5) {		//会場ごとの感度合わせ(区画中心)
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();
					Wall_Center_Calibration();
				}
			} else if ((Encorder_number_out() >= 6)
					&& (Encorder_number_out() <= 9)) {	//壁切れの位置のテスト
				/* スタート位置(後ろの壁に付けて置く)から4区画まっすぐ走り、柱の端(fall)と、
				 * 比べるための閾値の判定(th: av 従来の g_sensor_av < Cut_*、med 3回の中央値 < Cut_*)、
				 * 1mmごとの横センサの値(sig)を残す。結果は出力(モード4 No.7)で出す。
				 * fall の c-border は、境目の推定 c と区画の境目(114 + 180k)の差。0 になるように Edge_Off を直す。
				 * No.6: 500(吸引なし)、No.7: 1000、No.8: 2000、No.9: 2400(No.7〜9は吸引70)。加速度20000。
				 * 同じ置き方で速度を変えて走らせ、c-border が速度・左右・種類でどれだけ変わるかを見る */
				if (Sensor_Enter() == 1) {
					const float v_tab[4] = { 500, 1000, 2000, 2400 };
					int no = Encorder_number_out();
					float v = v_tab[no - 6];
					Buzzer_Enter();
					Sensor_Start();
					Motor_Setup();
					if (no >= 7) {
						Suction_Start(70);
						HAL_Delay(500);
					}
					Edge_Log_Start(1, 114, v);
					Motor_trapezoid_PID(0, v, 0, 20000, 114 + 180 * 4);
					Motor_Stop();
					Edge_Log_Stop();
					Suction_Stop();
					Motor_Free();
					Encorder_count_reset();
				}
			} else if (Encorder_number_out() == 10) {	//吸引のときのセンサのノイズ
				/* 止まったまま(モーターは使わない)、吸引(0/35/50/70)とセンサの読み方(LEDを点けてから読むまでの待ち、
				 * ADCの取り込み時間)を変えて、点灯時・消灯時の生の値のばらつきを測る。30秒ほど。
				 * USBをつないだまま走らせると、測ったそばから出す(tools/viewer.html でグラフになる)。
				 * つながずに走らせたときは、出力(モード4 No.7)でばらつきの表だけ出る。docs/sensor_noise.md */
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();
					SensorNoise_Run();
					Buzzer_Enter();
					Encorder_count_reset();
				}
			}
		} else if (Encorder_mode_out() == 4) {		//基本調整
			if (Encorder_number_out() == 0) {		//センサ
				Wall_search_LED();
//								__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 35);
//								__HAL_TIM_SET_AUTORELOAD(&htim2, 100);
//								HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
				//Suction_Start(35);
//				printf("FL = %4d //", g_sensor_av[0]);
//				printf(" L = %4d //", g_sensor_av[2]);
//				printf(" R = %4d //", g_sensor_av[1]);
//				printf("FR = %4d //", g_sensor_av[3]);
				printf("FL = %4d //", g_sensor_av10[0]);
				printf(" L = %4d //", g_sensor_av10[2]);
				printf(" R = %4d //", g_sensor_av10[1]);
				printf("FR = %4d //", g_sensor_av10[3]);
//				printf("FL = %4d //", g_sensor[0][0]);
//				printf(" L = %4d //", g_sensor[2][0]);
//				printf(" R = %4d //", g_sensor[1][0]);
//				printf("FR = %4d //", g_sensor[3][0]);
				printf("BATT = %.2f//", g_V_batt);
				printf("GyroZ = %.3f\n\r", read_NoiseCut_gyro_z());
			} else if (Encorder_number_out() == 1) {		//直進
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();
					///
					/* タイヤ径の測定用(2026-10-09): 本番と同じ吸引で、滑らないようにゆっくり 180mm×8 = 1440mm 走る。
					 * 実際に進んだ距離を測り、Define.h の TIREDIAMETER = 今の値 × 実際の距離 / 1440 にする。
					 * 吸引は 2400 などと同じ 70(2000 の 50 で測るときは 50 に変える) */
					Motor_Setup();
					Suction_Start(70);
					LOG_get_start();
					//					Motor_trapezoid_PID(0, 2000, 2000, 60000, 24);
					//					Motor_trapezoid_PID(2000, 2000, 0, 35000, 90);
					//Motor_Multistage_PID(0, 7000, 0, 25000, 180 * 9);
					//Motor_trapezoid_PID(0, 3000, 0, 20000, 180 * 9);
					//Motor_trapezoid_Asymmetric_PID(0, 4000, 0, 23000, 180 * 9);
					//Motor_trapezoid_PID(0, 3000, 0, 10000, 180*8);	//2026-10-09 まで(吸引35)
					Motor_trapezoid_PID(0, 1000, 0, 5000, 180 * 8);
					Motor_Stop();
					Motor_Stop();
					Motor_Stop();
					Suction_Stop();
					LOG_get_end();
					///
					//Motor_trapezoid_PID(0, 6000, 0, 50000, 180 * 4);
					Encorder_count_reset();
				}
			} else if (Encorder_number_out() == 2) {//??????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?シン??????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();
					///
					Motor_Setup();
					Suction_Start(35);
					LOG_get_start();
					//					for (int i = 0; i < 20; i++) {
					//						Motor_trapezoid_Turn(90, 600, 20000);
					//						Motor_Stop();
					//					}
					Motor_trapezoid_Turn(180, 1000, 30000);
					//Motor_trapezoid_Turn(3600, 600, 20000);
					///
					Motor_Stop();
					Suction_Stop();
					LOG_get_end();
					Encorder_count_reset();
				}
			} else if (Encorder_number_out() == 3) {//スラロー??????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();
					//
					Motor_Setup();
					LOG_get_start();
					Motor_trapezoid_PID(0, 500, 500, 5000, 90 + 180 + 20);
					Motor_Sula_COS(500, 90, 700, 10000);
					Motor_trapezoid(500, 500, 0, 5000, 180 + 25);
					Motor_Stop();
					LOG_get_end();
					//
					Encorder_count_reset();
				}
			} else if (Encorder_number_out() == 4) {		//斜め
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();
					Suction_Start(70);
					Motor_Setup();
					//Motor_NANAME_PID(0, 2000, 0, 5000, 127.3 * 6);
					Motor_NANAME_PID(0, 5000, 0, 20000, 127.3 * 6);
					Motor_Stop();
					Suction_Stop();
					Encorder_count_reset();
				}
			} else if (Encorder_number_out() == 5) {//吸??????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();
					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 90);
					__HAL_TIM_SET_AUTORELOAD(&htim2, 100);
					HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
					while (1) {
						if (Sensor_Enter() == 1) {
							break;
						}
					}
					HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);

				}
			} else if (Encorder_number_out() == 6) {		//壁合わせ
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();

					Motor_Setup();
					Suction_Start(35);
					Motor_Robot_Alignment();
					Motor_Stop();
					Suction_Stop();
					Encorder_count_reset();
				}
			} else if (Encorder_number_out() == 7) {		//出力(残っているデータを全部)
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();
					Output_All();
					HAL_Delay(1000);
				}
			} else if (Encorder_number_out() == 8) {		//直進(横壁制御をセンサ値版で)
				/* No.1と同じ直進を、この走行のあいだだけ G_WallCtrl_Use_mm = 0
				 * (従来のセンサ値版、ゲインは Kp / Kd)で走る。標準(距離版)との比較・切り戻し用 */
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();
					Motor_Setup();
					Suction_Start(35);
					LOG_get_start();
					int use_mm_prev = G_WallCtrl_Use_mm;
					G_WallCtrl_Use_mm = 0;
					Motor_trapezoid_PID(0, 3000, 0, 10000, 180*8);
					Motor_Stop();
					G_WallCtrl_Use_mm = use_mm_prev;
					Suction_Stop();
					LOG_get_end();
					Encorder_count_reset();
				}
			} else if ((Encorder_number_out() >= 9)
					&& (Encorder_number_out() <= 14)) {	//最初のターンの調整
				/* 最短走行の最初のターン(右)だけを、スタート位置から走る。
				 * No.9: 大回り90、No.10: 斜め入り45、No.11: 斜め入り135。
				 * No.12〜14: 同じターンを、加速しながら曲がる方式(G_First_Turn_Accel = 1)で走る。
				 * ターンのあとは後距離で2400へ加速し、そのまままっすぐ(斜めは斜めに)減速して止まる。
				 * パラメータは Move.c の G_First_Turn。ログ(2秒)はモード4 No.7で出力 */
				if (Sensor_Enter() == 1) {
					int no = Encorder_number_out();
					int m = (no - 9) % 3;
					int kind = (m == 0) ? FIRST_TURN_BIG90 :
								(m == 1) ? FIRST_TURN_IN45 : FIRST_TURN_IN135;
					int accel_prev = G_First_Turn_Accel;
					G_First_Turn_Accel = (no >= 12);
					Buzzer_Enter();
					Sensor_Start();
					Suction_Start(70);
					HAL_Delay(500);
					Motor_Setup();
					float v = Short_First_Turn_Run(kind, 1, 2400);
					G_First_Turn_Accel = accel_prev;
					if (kind == FIRST_TURN_BIG90) {
						Motor_trapezoid_PID(v, v, 0, 30000, 180);
					} else {
						Motor_NANAME_PID(v, v, 0, 30000, 127.3 * 2);
					}
					Motor_Stop();
					HAL_Delay(300);
					Suction_Stop();
					LOG_get_end();
					Encorder_count_reset();
				}
			}
		} else if (Encorder_mode_out() == 5) {		//1m/s
			if (Encorder_number_out() == 0) {		//90小回�?
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();

					Motor_Setup();
					Suction_Start(35);

					HAL_Delay(1000);
					LOG_get_start();
					Motor_trapezoid_PID(0, 1000, 1000, 8000, 90 + 180);
					//Motor_trapezoid_PID(1000, 1000, 1000, 15000, 15);
					Motor_Sula_ST(1000, 1000, 1000, 15000, 15);
					Motor_Sula_COS(1000, 90, 1640, 60000);
					Motor_trapezoid(1000, 1000, 1000, 15000, 50);
					Motor_trapezoid(1000, 1000, 0, 15000, 180);

					Motor_Stop();

					HAL_Delay(500);
					Suction_Stop();
					LOG_get_end();

					Encorder_count_reset();

				}
			} else if (Encorder_number_out() == 1) {		//90
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();

					Motor_Setup();
					Suction_Start(35);

					HAL_Delay(1000);
					LOG_get_start();
					Motor_trapezoid_PID(0, 1000, 1000, 15000, 90 + 90);
					Motor_trapezoid_PID(1000, 1000, 1000, 15000, 50);
					//Motor_Wallcut_ST(1000, 48, 0);
					Motor_Sula_COS(1000, 90, 750, 20000);
					Motor_trapezoid(1000, 1000, 1000, 15000, 85);
					Motor_trapezoid(1000, 1000, 0, 15000, 180);

					Motor_Stop();

					HAL_Delay(500);
					Suction_Stop();
					LOG_get_end();

					Encorder_count_reset();

				}
			} else if (Encorder_number_out() == 2) {		//180
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();

					Motor_Setup();
					Suction_Start(35);

					HAL_Delay(1000);
					LOG_get_start();
					Motor_trapezoid_PID(0, 1000, 1000, 15000, 90 + 90);
					Motor_trapezoid_PID(1000, 1000, 1000, 15000, 40);
					//Motor_Wallcut_ST(1000, 60, 0);
					Motor_Sula_COS(1000, 180, 680, 13000);
					Motor_trapezoid(1000, 1000, 1000, 15000, 70);
					Motor_trapezoid(1000, 1000, 0, 15000, 180);

					Motor_Stop();
					HAL_Delay(500);
					Suction_Stop();
					LOG_get_end();

					Encorder_count_reset();
				}
			} else if (Encorder_number_out() == 3) {		//in45
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();

					Motor_Setup();
					Suction_Start(15);

					HAL_Delay(1000);
					LOG_get_start();
					Motor_trapezoid_PID(0, 1000, 1000, 15000, 90 + 90);
					Motor_trapezoid_PID(1000, 1000, 1000, 15000, 15);
					Motor_Sula_COS(1000, 44, 750, 10000);
					Motor_trapezoid(1000, 1000, 1000, 15000, 42);
					Motor_trapezoid(1000, 1000, 0, 15000, 127.3);

					Motor_Stop();
					HAL_Delay(500);
					Suction_Stop();
					LOG_get_end();

					Encorder_count_reset();
				}
			} else if (Encorder_number_out() == 4) {		//in135
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();

					Motor_Setup();
					Suction_Start(15);

					HAL_Delay(1000);
					LOG_get_start();
					Motor_trapezoid_PID(0, 1000, 1000, 15000, 90 + 90);
					Motor_trapezoid_PID(1000, 1000, 1000, 15000, 38);
					Motor_Sula_COS(1000, 134, 800, 10000);
					Motor_trapezoid(1000, 1000, 1000, 15000, 8);
					Motor_trapezoid(1000, 1000, 0, 15000, 127.3);

					Motor_Stop();
					HAL_Delay(500);
					Suction_Stop();
					LOG_get_end();

					Encorder_count_reset();
				}
			} else if (Encorder_number_out() == 5) {		//out45
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();

					Motor_Setup();
					Suction_Start(15);

					HAL_Delay(1000);
					LOG_get_start();
					Motor_trapezoid(0, 1000, 1000, 10000, 127.3);
					Motor_trapezoid(1000, 1000, 1000, 5000, 73);
					Motor_Sula_COS(1000, 41.5, 700, 20000);
					Motor_trapezoid(1000, 1000, 1000, 5000, 30);
					Motor_trapezoid(1000, 1000, 0, 10000, 180);

					Motor_Stop();
					HAL_Delay(500);
					Suction_Stop();
					LOG_get_end();

					Encorder_count_reset();
				}
			} else if (Encorder_number_out() == 6) {		//out135
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();

					Motor_Setup();
					Suction_Start(15);

					HAL_Delay(1000);
					LOG_get_start();
					Motor_trapezoid(0, 1000, 1000, 10000, 127.3);
					Motor_trapezoid(1000, 1000, 1000, 5000, 25);
					Motor_Sula_COS(1000, 132, 700, 14000);
					Motor_trapezoid(1000, 1000, 1000, 5000, 25);
					Motor_trapezoid(1000, 1000, 0, 10000, 180);

					Motor_Stop();
					HAL_Delay(500);
					Suction_Stop();
					LOG_get_end();

					Encorder_count_reset();
				}
			} else if (Encorder_number_out() == 7) {		//V90
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();

					Motor_Setup();
					Suction_Start(15);

					HAL_Delay(1000);
					LOG_get_start();
					Motor_trapezoid(0, 1000, 1000, 10000, 127.3);
					Motor_trapezoid(1000, 1000, 1000, 5000, 37);
					Motor_Sula_COS(1000, 88.5, 800, 20000);
					Motor_trapezoid(1000, 1000, 1000, 5000, 27);
					Motor_trapezoid(1000, 1000, 0, 10000, 127.3);

					Motor_Stop();
					HAL_Delay(500);
					Suction_Stop();
					LOG_get_end();

					Encorder_count_reset();
				}
			}
		} else if (Encorder_mode_out() == 6) {	//2.0m/s
			if (Encorder_number_out() == 0) {		//
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();

					Motor_Setup();
					Suction_Start(51);

					HAL_Delay(1000);
					LOG_get_start();
					Motor_trapezoid_PID(0, 6000, 0, 40000, 180 * 6);

					Motor_Stop();

					HAL_Delay(500);
					Suction_Stop();
					LOG_get_end();

					Encorder_count_reset();
				}
			} else if ((Encorder_number_out() >= 1)
					&& (Encorder_number_out() <= 7)) {	//2000のターン単体(G_Turn2000、左旋回)
				/* No.1 大回り90、2 大回り180、3 斜め入り45、4 斜め入り135、5 斜め出45、6 斜め出135、7 V90。
				 * 値は Move.c の G_Turn2000(BFS・ダイクストラの2000と共通、2026-10-09) */
				if (Sensor_Enter() == 1) {
					int n = Encorder_number_out();
					Buzzer_Enter();
					Sensor_Start();

					Motor_Setup();
					Suction_Start(50);

					HAL_Delay(1000);
					LOG_get_start();
					Turn2000_Test(n - 1);

					Motor_Stop();

					HAL_Delay(500);
					Suction_Stop();
					LOG_get_end();

					Encorder_count_reset();
				}
			}
		} else if (Encorder_mode_out() == 7) {	//2.4m/s
			if (Encorder_number_out() == 0) {		//
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();

				}
			} else if (Encorder_number_out() == 1) {		//90
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();

					Motor_Setup();
					Suction_Start(70);

					HAL_Delay(1000);
					LOG_get_start();
					Motor_trapezoid_PID(0, 2400, 2400, 30000, 90 + 90 + 180);
					Motor_trapezoid_PID(2400, 2400, 2400, 15000, 25);
					//Motor_Wallcut_ST(1000, 60, 0);
					Motor_Sula_COS(2400, 90, 1600, 80000);
					Motor_trapezoid(2400, 2400, 2400, 15000, 100);
					Motor_trapezoid(2400, 2400, 0, 25000, 180);

					Motor_Stop();

					HAL_Delay(500);
					Suction_Stop();
					LOG_get_end();

					Encorder_count_reset();
				}
			} else if (Encorder_number_out() == 2) {		//180
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();

					Motor_Setup();
					Suction_Start(70);

					HAL_Delay(1000);
					LOG_get_start();
					Motor_trapezoid_PID(0, 2400, 2400, 30000, 90 + 90 + 180);
					Motor_trapezoid_PID(2400, 2400, 2400, 15000, 15);
					//Motor_Wallcut_ST(1000, 60, 0);
					Motor_Sula_COS(2400, 180, 1500, 70000);
					Motor_trapezoid(2400, 2400, 2400, 15000, 88);
					Motor_trapezoid(2400, 2400, 0, 30000, 180);

					Motor_Stop();

					HAL_Delay(500);
					Suction_Stop();
					LOG_get_end();

					Encorder_count_reset();
				}
			} else if (Encorder_number_out() == 3) {		//in45
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();

					Motor_Setup();
					Suction_Start(70);

					HAL_Delay(1000);
					LOG_get_start();
					Motor_trapezoid_PID(0, 2400, 2400, 25000, 90 + 90 + 180);
					Motor_trapezoid_PID(2400, 2400, 2400, 15000, 3);//8
					Motor_Sula_COS(2400, 45, 2500, 160000);
					Motor_trapezoid(2400, 2400, 2400, 15000, 113);
					Motor_trapezoid(2400, 2400, 0, 30000, 127.3 * 2);

					Motor_Stop();

					HAL_Delay(500);
					Suction_Stop();
					LOG_get_end();

					Encorder_count_reset();
				}
			} else if (Encorder_number_out() == 4) {		//in135
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();

					Motor_Setup();
					Suction_Start(70);

					HAL_Delay(1000);
					LOG_get_start();
					Motor_trapezoid_PID(0, 2400, 2400, 25000, 90 + 90 + 180);
					Motor_trapezoid_PID(2400, 2400, 2400, 15000, 15);
					Motor_Sula_COS(2400, 135, 2000, 55000);
					Motor_trapezoid(2400, 2400, 2400, 15000, 85);
					Motor_trapezoid(2400, 2400, 0, 25000, 127.3 * 2);

					Motor_Stop();

					HAL_Delay(500);
					Suction_Stop();
					LOG_get_end();

					Encorder_count_reset();
				}
			} else if (Encorder_number_out() == 5) {		//out45
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();

					Motor_Setup();
					Suction_Start(70);

					HAL_Delay(1000);
					LOG_get_start();

					//					Motor_trapezoid(0, 2400, 2400, 25000, 127.3);
					//					Motor_trapezoid(2400, 2400, 2400, 15000, 31);
					//					Motor_Sula_COS(2400, 41, 1550, 60000);
					//					Motor_trapezoid(2400, 2400, 2400, 15000, 18);
					//					Motor_trapezoid(2400, 2400, 0, 25000, 180);

					Motor_trapezoid_PID(0, 2400, 2400, 25000, 90 + 90 + 180);
					Motor_trapezoid_PID(2400, 2400, 2400, 15000, 8);
					Motor_Sula_COS(2400, 45, 2500, 160000);
					Motor_trapezoid(2400, 2400, 2400, 15000, 113);

					Motor_trapezoid(2400, 2400, 2400, 15000, 8);
					Motor_Sula_COS(2400, -45, 1550, 70000);
					Motor_trapezoid(2400, 2400, 2400, 15000, 40);
					Motor_trapezoid(2400, 2400, 0, 35000, 180);

					Motor_Stop();

					HAL_Delay(500);
					Suction_Stop();
					LOG_get_end();

					Encorder_count_reset();
				}
			} else if (Encorder_number_out() == 6) {		//out135
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();

					Motor_Setup();
					Suction_Start(70);

					//					HAL_Delay(1000);
					//					LOG_get_start();
					//					Motor_NANAME_PID(0, 2400, 2400, 30000, 127.3);
					//					Motor_NANAME_PID(2400, 2400, 2400, 15000, 20);
					//					Motor_Sula_COS(2400, 123, 2300, 70000);
					//					Motor_trapezoid(2400, 2400, 2400, 15000, 80);
					//					Motor_trapezoid(2400, 2400, 0, 30000, 180);

					Motor_trapezoid_PID(0, 2400, 2400, 25000, 90 + 90 + 180);
					Motor_trapezoid_PID(2400, 2400, 2400, 15000, 8);
					Motor_Sula_COS(2400, 45, 2500, 160000);
					Motor_trapezoid(2400, 2400, 2400, 15000, 113);
					Motor_NANAME_PID(2400, 2400, 2400, 30000, 127.3);

					Motor_NANAME_PID(2400, 2400, 2400, 15000, 17);
					Motor_Sula_COS(2400, 134, 2100, 80000);
					Motor_trapezoid(2400, 2400, 2400, 15000, 123);
					Motor_trapezoid(2400, 2400, 0, 30000, 180);

					Motor_Stop();

					HAL_Delay(500);
					Suction_Stop();
					LOG_get_end();

					Encorder_count_reset();
				}
			} else if (Encorder_number_out() == 7) {		//v90
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();

					Motor_Setup();
					Suction_Start(70);

					HAL_Delay(1000);
					LOG_get_start();
					//					Motor_trapezoid(0, 2400, 2400, 40000, 127.3);
					//					Motor_trapezoid(2400, 2400, 2400, 15000, 19);
					//					Motor_Sula_COS(2400, 83, 2500, 100000);
					//					Motor_trapezoid(2400, 2400, 2400, 15000, 29);
					//					Motor_trapezoid(2400, 2400, 0, 25000, 127.3);

					Motor_trapezoid_PID(0, 2400, 2400, 25000, 90 + 90 + 180);
					Motor_trapezoid_PID(2400, 2400, 2400, 15000, 8);
					Motor_Sula_COS(2400, 45, 2500, 160000);
					Motor_trapezoid(2400, 2400, 2400, 15000, 113);
					Motor_NANAME_PID(2400, 2400, 2400, 30000, 127.3);

					Motor_trapezoid(2400, 2400, 2400, 15000, 5);
					Motor_Sula_COS(2400, 87, 2600, 150000);
					Motor_trapezoid(2400, 2400, 2400, 15000, 90);
					Motor_trapezoid(2400, 2400, 0, 25000, 127.3 * 2);

					Motor_Stop();

					HAL_Delay(500);
					Suction_Stop();
					LOG_get_end();

					Encorder_count_reset();
				}
			}
		} else if (Encorder_mode_out() == 8) {			//2.7m/s
			if (Encorder_number_out() == 0) {		//
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();
				}
			} else if (Encorder_number_out() == 1) {		//
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();

					Motor_Setup();
					Suction_Start(85);

					HAL_Delay(1000);
					LOG_get_start();
					Motor_trapezoid_PID(0, 2700, 2700, 25000, 180 + 90 + 90);
					Motor_trapezoid_PID(2700, 2700, 2700, 15000, 25);
					//Motor_Wallcut_ST(2000, 50, 0);
					Motor_Sula_COS(2700, 90, 2300, 70000);
					Motor_trapezoid(2700, 2700, 2700, 15000, 68);
					Motor_trapezoid(2700, 2700, 0, 30000, 180);

					Motor_Stop();

					HAL_Delay(500);
					Suction_Stop();
					LOG_get_end();

					Encorder_count_reset();
				}
			} else if (Encorder_number_out() == 2) {		//
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();

					Motor_Setup();
					Suction_Start(85);

					HAL_Delay(1000);
					LOG_get_start();
					Motor_trapezoid_PID(0, 2700, 2700, 25000, 180 + 90 + 90);
					Motor_trapezoid_PID(2700, 2700, 2700, 15000, 10);
					//Motor_Wallcut_ST(2000, 50, 0);
					Motor_Sula_COS(2700, 180, 1700, 40000);
					Motor_trapezoid(2700, 2700, 2700, 15000, 58);
					Motor_trapezoid(2700, 2700, 0, 30000, 180);

					Motor_Stop();

					HAL_Delay(500);
					Suction_Stop();
					LOG_get_end();

					Encorder_count_reset();
				}
			} else if (Encorder_number_out() == 3) {		//
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();
					Motor_Setup();
					Suction_Start(85);

					HAL_Delay(1000);
					LOG_get_start();
					Motor_trapezoid_PID(0, 2700, 2700, 25000, 90 + 90 + 180);
					Motor_trapezoid_PID(2700, 2700, 2700, 15000, 2);
					Motor_Sula_COS(2700, 45, 2850, 100000);
					Motor_trapezoid(2700, 2700, 2700, 15000, 85);
					Motor_trapezoid(2700, 2700, 0, 30000, 127.3 * 2);

					Motor_Stop();

					HAL_Delay(500);
					Suction_Stop();
					LOG_get_end();

					Encorder_count_reset();
				}
			} else if (Encorder_number_out() == 4) {		//
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();
					Motor_Setup();
					Suction_Start(85);

					HAL_Delay(1000);
					LOG_get_start();
					Motor_trapezoid_PID(0, 2700, 2700, 25000, 90 + 90 + 180);
					Motor_trapezoid_PID(2700, 2700, 2700, 15000, 11);
					Motor_Sula_COS(2700, 135, 2200, 60000);
					Motor_trapezoid(2700, 2700, 2700, 15000, 70);
					Motor_trapezoid(2700, 2700, 0, 30000, 127.3 * 2);

					Motor_Stop();

					HAL_Delay(500);
					Suction_Stop();
					LOG_get_end();

					Encorder_count_reset();
				}
			} else if (Encorder_number_out() == 5) {		//
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();

					Motor_Setup();
					Suction_Start(85);

					HAL_Delay(1000);
					LOG_get_start();

					Motor_trapezoid_PID(0, 2700, 2700, 25000, 90 + 90 + 180);
					Motor_trapezoid_PID(2700, 2700, 2700, 15000, 2);
					Motor_Sula_COS(2700, 45, 2850, 100000);
					Motor_trapezoid(2700, 2700, 2700, 15000, 85);
					Motor_NANAME_PID(2700, 2700, 2700, 30000, 127.3);

					Motor_trapezoid(2700, 2700, 2700, 15000, 12);
					Motor_Sula_COS(2700, 45, 1800, 70000);
					Motor_trapezoid(2700, 2700, 2700, 15000, 10);
					Motor_trapezoid(2700, 2700, 0, 30000, 180);

					Motor_Stop();

					HAL_Delay(500);
					Suction_Stop();
					LOG_get_end();

					Encorder_count_reset();
				}
			} else if (Encorder_number_out() == 6) {		//
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();

					Motor_Setup();
					Suction_Start(85);

					HAL_Delay(1000);
					LOG_get_start();

					Motor_trapezoid_PID(0, 2700, 2700, 25000, 90 + 90 + 180);
					Motor_trapezoid_PID(2700, 2700, 2700, 15000, 2);
					Motor_Sula_COS(2700, 45, 2850, 100000);
					Motor_trapezoid(2700, 2700, 2700, 15000, 85);
					Motor_NANAME_PID(2700, 2700, 2700, 30000, 127.3);

					Motor_NANAME_PID(2700, 2700, 2700, 15000, 23);
					Motor_Sula_COS(2700, 135, 2000, 100000);
					Motor_trapezoid(2700, 2700, 2700, 15000, 117);
					Motor_trapezoid(2700, 2700, 0, 30000, 180);

					Motor_Stop();

					HAL_Delay(500);
					Suction_Stop();
					LOG_get_end();

					Encorder_count_reset();
				}
			} else if (Encorder_number_out() == 7) {		//
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();

					Motor_Setup();
					Suction_Start(85);

					HAL_Delay(1000);
					LOG_get_start();

					Motor_trapezoid_PID(0, 2700, 2700, 25000, 90 + 90 + 180);
					Motor_trapezoid_PID(2700, 2700, 2700, 15000, 2);
					Motor_Sula_COS(2700, 45, 2850, 100000);
					Motor_trapezoid(2700, 2700, 2700, 15000, 85);
					Motor_NANAME_PID(2700, 2700, 2700, 30000, 127.3);

					Motor_trapezoid(2700, 2700, 2700, 15000, 18);
					Motor_Sula_COS(2700, 82, 2800, 180000);
					Motor_trapezoid(2700, 2700, 2700, 15000, 97);
					Motor_trapezoid(2700, 2700, 0, 30000, 127.3 * 2);

					Motor_Stop();

					HAL_Delay(500);
					Suction_Stop();
					LOG_get_end();

					Encorder_count_reset();
				}
			}
		} else if (Encorder_mode_out() == 9) {		//3m/s
			if (Encorder_number_out() == 0) {		//
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();
				}
			} else if (Encorder_number_out() == 1) {		//
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();

					Motor_Setup();
					Suction_Start(90);

					HAL_Delay(1000);
					LOG_get_start();
					Motor_trapezoid_PID(0, 3000, 3000, 20000, 180 + 90 + 90);
					Motor_trapezoid_PID(3000, 3000, 3000, 15000, 5);
					//Motor_Wallcut_ST(2000, 50, 0);
					Motor_Sula_COS(3000, 87.5, 2500, 80000);
					Motor_trapezoid(3000, 3000, 3000, 15000, 85);
					Motor_trapezoid(3000, 3000, 0, 25000, 270);

					Motor_Stop();

					HAL_Delay(500);
					Suction_Stop();
					LOG_get_end();

					Encorder_count_reset();
				}
			} else if (Encorder_number_out() == 2) {		//
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();

				}
			} else if (Encorder_number_out() == 3) {		//
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();
				}
			} else if (Encorder_number_out() == 4) {		//
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();
				}
			} else if (Encorder_number_out() == 5) {		//
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();
				}
			} else if (Encorder_number_out() == 6) {		//
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();
				}
			} else if (Encorder_number_out() == 7) {		//
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();
				}
			}
		} else if (Encorder_mode_out() == 10) {		//ターンごとの通過速度のターン単体テスト
			/* No.1〜7: Short_NANAME_MoveTurnV の TurnV_Table の速度とパラメータで、ターンを1つ(左)。
			 * 1=大回り90, 2=大回り180, 3=斜め入り45, 4=斜め入り135, 5=斜め出45, 6=斜め出135, 7=V90 */
			int n = Encorder_number_out();
			if ((n >= 1) && (n <= 7) && (Sensor_Enter() == 1)) {
				Buzzer_Enter();
				Sensor_Start();

				Motor_Setup();
				Suction_Start(70);

				HAL_Delay(1000);
				LOG_get_start();
				TurnV_Test(n - 1);

				Motor_Stop();

				HAL_Delay(500);
				Suction_Stop();
				LOG_get_end();

				Encorder_count_reset();
			}
		}
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 5;
	RCC_OscInitStruct.PLL.PLLN = 180;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Activate the Over-Drive mode
	 */
	if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

/* 1区画ぶんの探索動作。flags で走り方を選ぶ */
static void Maze_Search_Action(int flags) {
	if (flags & SEARCH_KNOWN) {
		Robot_Maze_Pass_Action();
	} else if (flags & SEARCH_SUCTION) {
		Robot_Maze_Suction_Action();
	} else {
		Robot_Maze_Sula_Action();
	}
}

/* (G_Gool_X, G_Gool_Y) に着くまで探索する */
static void Maze_Search_Until_Goal(int flags) {
	while (G_MAZE_Explored[G_Gool_X][G_Gool_Y] == 0) {
		if (Failsafe_Flag() == 1) {
			break;
		}
		Maze_Search_Action(flags);
		G_MAZE_Explored[G_Robot_MAZE_X][G_Robot_MAZE_Y] = 1;
	}
}

/* ダイクストラ誘導の未知壁探索で、経路上の未確認の壁がなくなったら1 */
static int s_unknown_done = 0;

/* 止まっているあいだに呼ぶ計算(G_Stop_Hook)。スタート→ゴールのダイクストラ経路を
 * 作り直し、経路上でまだ見ていない最初の壁の手前の区画を目標にした歩数マップを作る。
 * 見る壁がなければ、スタートへ帰る歩数マップにする */
static void Maze_Unknown_Walls_Plan(void) {
	G_Gool_X = MAZE_GOOL_X;
	G_Gool_Y = MAZE_GOOL_Y;
	Maze_Dijkstra_Calculation();
	if (Maze_Unknown_Wall_Scan() == 1) {
		Maze_Unknown_Target_ModeSet(G_Unknown_Target_X, G_Unknown_Target_Y);
		s_unknown_done = 0;
	} else {
		Maze_Unkown_ALL_ModeOFF();
		G_Gool_X = 0;
		G_Gool_Y = 0;
		G_MAZE_Explored[0][0] = 0;
		s_unknown_done = 1;
	}
	Maze_Step_Calculate();
}

/* ゴールに着いたあと、スタート→ゴールのダイクストラ経路上に残っている
 * 未確認の壁を、今いる位置から直接見に行く。
 * ダイクストラの計算は機体が止まっているときだけ行う:
 *   - ゴールに着いた直後: 次の区画の中央で止まって計算してから発進
 *     (Robot_Maze_Stop_And_Go)
 *   - Uターンしたとき: Uターンで止まっているあいだに計算する(G_Stop_Hook)。
 *     行き止まりでなければ、計算した結果で進む向きを決め直す
 *   - 目標に辿り着けなかったとき(異常系): 次の区画で止まって計算する
 *   - 見に行った壁が実際にあったとき(経路がふさがれた): 次の区画で止まって計算する
 *     (2026-10-13 追加。以前は計算し直さずに古い経路の残りの壁を確認して終わり、
 *      もっと短い別の経路を一度も見に行かないことがあった。東北大会で下回りが55区画、
 *      上回りが57区画の迷路で、下回りを確認せずに上回りを走った)
 * 止まって計算するときは、前に壁があれば壁合わせもする(Robot_Maze_Stop_And_Go)。
 * 壁がなかった(通れた)ときは、止まらずに同じ経路データのまま次の未確認の壁を探す
 * (Maze_Unknown_Wall_Scan()は壁の確認状況をその都度ライブ判定する。
 *  ダイクストラは計算し直さず、歩数マップだけ走りながら作り直す) */
static void Maze_Search_Unknown_Walls(int flags) {
	int suction = (flags & SEARCH_SUCTION) ? 1 : 0;

	if (Failsafe_Flag() == 1) {
		return;
	}
	s_unknown_done = 0;
	G_Stop_Hook = Maze_Unknown_Walls_Plan;
	Robot_Maze_Stop_And_Go(suction);
	G_MAZE_Explored[G_Robot_MAZE_X][G_Robot_MAZE_Y] = 1;

	while ((Failsafe_Flag() == 0) && (s_unknown_done == 0)) {
		int sub_steps = 0;
		while ((Maze_Unknown_Wall_Still_Unknown() == 1)
				&& (sub_steps < MAX_STEP) && (s_unknown_done == 0)) {
			if (Failsafe_Flag() == 1) {
				break;
			}
			Maze_Search_Action(flags);
			G_MAZE_Explored[G_Robot_MAZE_X][G_Robot_MAZE_Y] = 1;
			sub_steps++;
		}
		if ((Failsafe_Flag() == 1) || (s_unknown_done == 1)) {
			break;
		}
		Maze_Save();

		if (sub_steps >= MAX_STEP) {
			//目標に辿り着けなかった: 止まってダイクストラから計算し直す
			Robot_Maze_Stop_And_Go(suction);
			G_MAZE_Explored[G_Robot_MAZE_X][G_Robot_MAZE_Y] = 1;
		} else if (Maze_Unknown_Wall_Is_Wall()) {
			//壁があって経路がふさがれた: 止まって(壁合わせをして)ダイクストラから計算し直す
			Robot_Maze_Stop_And_Go(suction);
			G_MAZE_Explored[G_Robot_MAZE_X][G_Robot_MAZE_Y] = 1;
		} else if (Maze_Unknown_Wall_Scan() == 1) {
			//壁がなかった(通れた): 同じ経路で次の未確認の壁へ
			Maze_Unknown_Target_ModeSet(G_Unknown_Target_X, G_Unknown_Target_Y);
			Maze_Step_Calculate();
		} else {
			//経路上の未確認の壁がなくなった
			Maze_Unkown_ALL_ModeOFF();
			s_unknown_done = 1;
		}
	}
	G_Stop_Hook = NULL;
	Maze_Unkown_ALL_ModeOFF();
	if (Failsafe_Flag() == 0) {
		Maze_Save();
	}
}

/* モード0の探索。往路(スタート→ゴール)のあと、flags に応じて
 * 片道ならゴールで止まり、そうでなければ(全面探索・未知壁探索をしながら)
 * スタートへ戻る。区間の終わりごとに Maze_Save() する。
 * SEARCH_SHORTEST なら、スタートに戻ったあと2秒待って最短走行する */
static void Maze_Search(int flags) {
	int v = (flags & (SEARCH_SUCTION | SEARCH_KNOWN)) ? 1000 : 500;

	Buzzer_Enter();
	Sensor_Start();

	Motor_Setup();
	Motor_Stop();
	if (flags & SEARCH_SUCTION) {
		Suction_Start(35);
	}

	//往路: スタート→ゴール
	G_Gool_X = MAZE_GOOL_X;
	G_Gool_Y = MAZE_GOOL_Y;
	G_Robot_MAZE_X = 0;
	G_Robot_MAZE_Y = 0;
	G_Robot_Direction = 0;
	G_MAZE_Explored[G_Gool_X][G_Gool_Y] = 0;

	/* 走り出しの114mmで探索速度まで加速する(cos加速で必要な距離は
	 * π*v^2/(4*ac))。1000mm/sは5000だと157mm必要で届かず、約930mm/sで
	 * 約136mm進んでから目標速度が1000に跳び、最初の判断位置が約22mm
	 * 遅れていた。7000なら112mmで届く。500mm/sは5000で39mmなので足りている */
	float start_ac = (v > 500) ? 7000 : 5000;
	Motor_trapezoid_PID(0, v, v, start_ac, 24 + 90);
	Maze_Search_Until_Goal(flags);
	Maze_Save();
	//ゴール後の最初の吸引探索のUターンからログを取る(モード4 No.7で出力)
	G_Log_Next_UTurn = 1;

	if (flags & SEARCH_ONEWAY) {
		//片道: ゴール区画に入ったところで、Uターンのときと同じ減速で止まる
		if (Failsafe_Flag() == 0) {
			Motor_trapezoid(v, v, 0, 5000, 70);
		}
	} else {
		if (flags & SEARCH_DIJKSTRA) {
			Maze_Search_Unknown_Walls(flags);
		}
		if (flags & SEARCH_ALL) {
			Maze_Unkown_ALL_ModeSet();
		}

		//復路: スタートへ戻る
		if (Failsafe_Flag() == 0) {
			G_Gool_X = 0;
			G_Gool_Y = 0;
			G_MAZE_Explored[G_Gool_X][G_Gool_Y] = 0;
			Maze_Step_Calculate();
			Maze_Search_Until_Goal(flags);
		}
	}

	Motor_Stop();
	if (flags & SEARCH_SUCTION) {
		Suction_Stop();
	}
	if (Failsafe_Flag() == 0) {
		Maze_Save();
		if ((flags & SEARCH_SHORTEST) && !(flags & SEARCH_ONEWAY)) {
			//最短走行(モード1 No.5と同じ: ダイクストラ、コーナー2000、直線6000mm/s、加速20000)
			HAL_Delay(2000);
			Motor_Setup();
			Motor_Stop();
			G_WallCut_Pillar = Short_WallCut_Pillar;
			Edge_Log_Start(0, -1, -1);
			G_Log_Div = 4;		//走行ログは4msおき(8秒)
			LOG_get_start();
			Short_Dijkstra_Move2000(6000, 20000);
			LOG_get_end();
			G_Log_Div = 1;
			Edge_Log_Stop();
			G_WallCut_Pillar = 0;
			LED_Reset();
			HAL_Delay(500);
		}
	} else {
		Failsafe_Flag_OFF();
	}
	G_Log_Next_UTurn = 0;
	Motor_Free();
	LED_Reset();
	Encorder_count_reset();
}

/* 出力(モード4 No.7)。いまRAMに残っているデータを全部、tools/viewer.html で読める形で出す
 * (迷路、壁切れのログ、走行ログの順。ないものは出さない。ビューアが種類ごとに分ける) */
static void Output_All(void) {
	Maze_Debug_Dump();
	if (Edge_Log_Has_Data()) {
		Edge_Log_Print();
	}
	if (LOG_Has_Data()) {
		LOG_print();
	}
	if (SensorNoise_Has_Data()) {
		SensorNoise_Print();
	}
	printf("\r\n#DONE\r\n");
}

/* モード1の最短走行。run に Short_*_Move* を渡す */
static void Maze_Shortest_Run(void (*run)(int, int), int max, int ac) {
	Buzzer_Enter();
	Sensor_Start();

	Motor_Setup();
	Motor_Stop();
	G_WallCut_Pillar = Short_WallCut_Pillar;
	Edge_Log_Start(0, -1, -1);	//壁切れのログ(出力はモード4 No.7)
	G_Log_Div = 4;		//走行ログは4msおき(8秒)
	LOG_get_start();
	run(max, ac);
	LOG_get_end();
	G_Log_Div = 1;
	Edge_Log_Stop();
	G_WallCut_Pillar = 0;
	Motor_Free();
	LED_Reset();
	HAL_Delay(500);
	Encorder_count_reset();
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
