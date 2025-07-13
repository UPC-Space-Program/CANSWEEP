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
#include "stm32g4xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RESET_BUTTON_Pin GPIO_PIN_10
#define RESET_BUTTON_GPIO_Port GPIOG
#define LSWITCH_OUT_Pin GPIO_PIN_0
#define LSWITCH_OUT_GPIO_Port GPIOC
#define RESET_DRV_Pin GPIO_PIN_1
#define RESET_DRV_GPIO_Port GPIOC
#define MOTOR_LEFT_Pin GPIO_PIN_2
#define MOTOR_LEFT_GPIO_Port GPIOC
#define MOTOR_RIGHT_Pin GPIO_PIN_3
#define MOTOR_RIGHT_GPIO_Port GPIOC
#define nSLEEP_Pin GPIO_PIN_0
#define nSLEEP_GPIO_Port GPIOA
#define nFAULT_Pin GPIO_PIN_1
#define nFAULT_GPIO_Port GPIOA
#define nSTALL_Pin GPIO_PIN_2
#define nSTALL_GPIO_Port GPIOA
#define VBAT_I_OUT_Pin GPIO_PIN_4
#define VBAT_I_OUT_GPIO_Port GPIOC
#define _5V_f_Pin GPIO_PIN_1
#define _5V_f_GPIO_Port GPIOB
#define _3v3_f_Pin GPIO_PIN_2
#define _3v3_f_GPIO_Port GPIOB
#define PH_B_f_Pin GPIO_PIN_12
#define PH_B_f_GPIO_Port GPIOB
#define PH_A_f_Pin GPIO_PIN_13
#define PH_A_f_GPIO_Port GPIOB
#define VBUS_f_Pin GPIO_PIN_14
#define VBUS_f_GPIO_Port GPIOB
#define DIR_AIN2_Pin GPIO_PIN_15
#define DIR_AIN2_GPIO_Port GPIOB
#define STEP_AIN1_Pin GPIO_PIN_10
#define STEP_AIN1_GPIO_Port GPIOA
#define BIN1_Pin GPIO_PIN_15
#define BIN1_GPIO_Port GPIOA
#define BIN2_Pin GPIO_PIN_10
#define BIN2_GPIO_Port GPIOC
#define ENCODER_PULSE1_Pin GPIO_PIN_12
#define ENCODER_PULSE1_GPIO_Port GPIOC
#define ENCODER_PULSE2_Pin GPIO_PIN_3
#define ENCODER_PULSE2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
