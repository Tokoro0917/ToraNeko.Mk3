/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LEDF4_Pin GPIO_PIN_13
#define LEDF4_GPIO_Port GPIOC
#define LEDF3_Pin GPIO_PIN_14
#define LEDF3_GPIO_Port GPIOC
#define LEDF2_Pin GPIO_PIN_15
#define LEDF2_GPIO_Port GPIOC
#define WallLED_34_Pin GPIO_PIN_0
#define WallLED_34_GPIO_Port GPIOA
#define WallLED_12_Pin GPIO_PIN_1
#define WallLED_12_GPIO_Port GPIOA
#define LEDF1_Pin GPIO_PIN_3
#define LEDF1_GPIO_Port GPIOA
#define ICM_NSS_Pin GPIO_PIN_4
#define ICM_NSS_GPIO_Port GPIOA
#define ENCO_L_NSS_Pin GPIO_PIN_5
#define ENCO_L_NSS_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_12
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_13
#define LED2_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_14
#define LED3_GPIO_Port GPIOB
#define LED4_Pin GPIO_PIN_15
#define LED4_GPIO_Port GPIOB
#define ENCO_R_NSS_Pin GPIO_PIN_15
#define ENCO_R_NSS_GPIO_Port GPIOA
#define LED5_Pin GPIO_PIN_4
#define LED5_GPIO_Port GPIOB
#define LED6_Pin GPIO_PIN_5
#define LED6_GPIO_Port GPIOB
#define LED7_Pin GPIO_PIN_6
#define LED7_GPIO_Port GPIOB
#define LED8_Pin GPIO_PIN_7
#define LED8_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
