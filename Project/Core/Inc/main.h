/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#define Key_GND_Pin GPIO_PIN_12
#define Key_GND_GPIO_Port GPIOB
#define Key_4_Pin GPIO_PIN_13
#define Key_4_GPIO_Port GPIOB
#define Key_3_Pin GPIO_PIN_14
#define Key_3_GPIO_Port GPIOB
#define Key_2_Pin GPIO_PIN_15
#define Key_2_GPIO_Port GPIOB
#define Key_1_Pin GPIO_PIN_8
#define Key_1_GPIO_Port GPIOA
#define LED_IO_Pin GPIO_PIN_11
#define LED_IO_GPIO_Port GPIOA
#define LED_GND_Pin GPIO_PIN_12
#define LED_GND_GPIO_Port GPIOA
#define Relay_IN_Pin GPIO_PIN_3
#define Relay_IN_GPIO_Port GPIOB
#define Relay_GND_Pin GPIO_PIN_4
#define Relay_GND_GPIO_Port GPIOB
#define Relay_VCC_Pin GPIO_PIN_5
#define Relay_VCC_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
