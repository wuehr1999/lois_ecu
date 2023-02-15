/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "rpmctrl.h"
#include "odometry.h"
#include "robotparameters.h"
#include "queue.h"
#include "interface.h"
#include "wire.h"
#include "nmea.h"
#include "filter.h"
#include "adcfilter.h"
#include "terminalmode.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum Recordmode
{
	RECORD_RPM,
	RECORD_HEADING,
	RECORD_CURRENT
}Recordmode;

typedef struct GPSData_t
{
	float latitude, longitude;
	int hours, minutes, seconds;
	int day, month, year;
}GPSData_t;

typedef struct EMERGENCY_t
{
	volatile bool isEvent;
	volatile bool isEmergency;
}EMERGENCY_t;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void Powertrain_Init();
void Powertrain_EnableMotors(int enableState);
void Powertrain_SetDutycycles(int left, int right);

void Servo_Set(int angle);
void Ballshitter_Init();
void Ballshitter_Drop();

void KVH_Init();
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define INH12_Pin GPIO_PIN_14
#define INH12_GPIO_Port GPIOC
#define INH34_Pin GPIO_PIN_15
#define INH34_GPIO_Port GPIOC
#define BEMF12_Pin GPIO_PIN_0
#define BEMF12_GPIO_Port GPIOA
#define BEMF34_Pin GPIO_PIN_1
#define BEMF34_GPIO_Port GPIOA
#define IS12_Pin GPIO_PIN_4
#define IS12_GPIO_Port GPIOA
#define IS34_Pin GPIO_PIN_5
#define IS34_GPIO_Port GPIOA
#define BATSENSE_Pin GPIO_PIN_6
#define BATSENSE_GPIO_Port GPIOA
#define CS_Pin GPIO_PIN_7
#define CS_GPIO_Port GPIOA
#define PWM2_Pin GPIO_PIN_0
#define PWM2_GPIO_Port GPIOB
#define PWM1_Pin GPIO_PIN_1
#define PWM1_GPIO_Port GPIOB
#define EMSTOP_Pin GPIO_PIN_12
#define EMSTOP_GPIO_Port GPIOB
#define EMSTOP_EXTI_IRQn EXTI15_10_IRQn
#define SERVO_Pin GPIO_PIN_8
#define SERVO_GPIO_Port GPIOA
#define ENCODER1_Pin GPIO_PIN_15
#define ENCODER1_GPIO_Port GPIOA
#define ENCODER1_EXTI_IRQn EXTI15_10_IRQn
#define ENCODER0_Pin GPIO_PIN_3
#define ENCODER0_GPIO_Port GPIOB
#define ENCODER0_EXTI_IRQn EXTI3_IRQn
#define PWM3_Pin GPIO_PIN_4
#define PWM3_GPIO_Port GPIOB
#define PWM4_Pin GPIO_PIN_5
#define PWM4_GPIO_Port GPIOB
#define SCL_Pin GPIO_PIN_6
#define SCL_GPIO_Port GPIOB
#define SDA_Pin GPIO_PIN_7
#define SDA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
