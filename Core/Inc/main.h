/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

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
#define BAT_NTC_Pin GPIO_PIN_0
#define BAT_NTC_GPIO_Port GPIOA
#define EN_HEAT_Pin GPIO_PIN_1
#define EN_HEAT_GPIO_Port GPIOA
#define MCU_TX_Pin GPIO_PIN_2
#define MCU_TX_GPIO_Port GPIOA
#define MCU_RX_Pin GPIO_PIN_3
#define MCU_RX_GPIO_Port GPIOA
#define AD_DCIN_Pin GPIO_PIN_4
#define AD_DCIN_GPIO_Port GPIOA
#define AD_BAT_V_Pin GPIO_PIN_5
#define AD_BAT_V_GPIO_Port GPIOA
#define W_LEVEL_Pin GPIO_PIN_6
#define W_LEVEL_GPIO_Port GPIOA
#define AD_NTC_Pin GPIO_PIN_7
#define AD_NTC_GPIO_Port GPIOA
#define AD_PUMP_I_Pin GPIO_PIN_0
#define AD_PUMP_I_GPIO_Port GPIOB
#define AD_MOTOR_I_Pin GPIO_PIN_1
#define AD_MOTOR_I_GPIO_Port GPIOB
#define Log_TX_Pin GPIO_PIN_10
#define Log_TX_GPIO_Port GPIOB
#define Log_RX_Pin GPIO_PIN_11
#define Log_RX_GPIO_Port GPIOB
#define WHEEL_ON_Pin GPIO_PIN_13
#define WHEEL_ON_GPIO_Port GPIOB
#define EN_BAT_NTC_Pin GPIO_PIN_14
#define EN_BAT_NTC_GPIO_Port GPIOB
#define DCIN_ON_Pin GPIO_PIN_15
#define DCIN_ON_GPIO_Port GPIOB
#define Linux_TX_Pin GPIO_PIN_9
#define Linux_TX_GPIO_Port GPIOA
#define Linux_RX_Pin GPIO_PIN_10
#define Linux_RX_GPIO_Port GPIOA
#define EN_PUMP_Pin GPIO_PIN_15
#define EN_PUMP_GPIO_Port GPIOA
#define EN_NTC_Pin GPIO_PIN_3
#define EN_NTC_GPIO_Port GPIOB
#define EN_TWV_Pin GPIO_PIN_4
#define EN_TWV_GPIO_Port GPIOB
#define EN_UV_Pin GPIO_PIN_5
#define EN_UV_GPIO_Port GPIOB
#define LED_R_Pin GPIO_PIN_6
#define LED_R_GPIO_Port GPIOB
#define LED_G_Pin GPIO_PIN_7
#define LED_G_GPIO_Port GPIOB
#define LED_B_Pin GPIO_PIN_8
#define LED_B_GPIO_Port GPIOB
#define LED_W_Pin GPIO_PIN_9
#define LED_W_GPIO_Port GPIOB
#define BAT_ON_Pin GPIO_PIN_12
#define BAT_ON_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
