/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#define _USE_MATH_DEFINES
#include <math.h>
#include <stdlib.h>
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
/* USER CODE BEGIN Private defines */
#define MASTER_CAN_ID (1) // CAN ID OF MASTER DEVICE
#define MY_CAN_ID (15) // CAN ID OF THIS DEVICE
#define STEPPER_STEP_DENUMENATOR (0x02)	// 200/this_value = steps_per_circle
#define STEPPER_PWM_PRESCALER (24)		// clock speed (80000000) divided by (this + 1) = pwm timer speed
#define GEAR_RATIO (50.0) 				// reductor gear ratio
#define APB1_TIMER_CLOCK_FREQUENCY (80000000.0)// frequency of the APB1 clock
//parameters of the tim7
#define USER_TIMER_PRESCALER (49)
#define USER_TIMER_PERIOD (15999)
#define USER_TIMER_FREQUENCY (100)


#define READ 0x01
#define WRITE 0x00

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
