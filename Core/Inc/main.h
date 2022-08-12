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
#include "stm32f3xx_hal.h"

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
#define LED_13_Pin GPIO_PIN_13
#define LED_13_GPIO_Port GPIOC
#define LED_14_Pin GPIO_PIN_14
#define LED_14_GPIO_Port GPIOC
#define LED_15_Pin GPIO_PIN_15
#define LED_15_GPIO_Port GPIOC
#define LED_01_Pin GPIO_PIN_0
#define LED_01_GPIO_Port GPIOF
#define LED_02_Pin GPIO_PIN_1
#define LED_02_GPIO_Port GPIOF
#define LED_03_Pin GPIO_PIN_0
#define LED_03_GPIO_Port GPIOC
#define LED_04_Pin GPIO_PIN_1
#define LED_04_GPIO_Port GPIOC
#define LED_05_Pin GPIO_PIN_2
#define LED_05_GPIO_Port GPIOC
#define SR_SI_Pin GPIO_PIN_3
#define SR_SI_GPIO_Port GPIOC
#define SR_SCK_Pin GPIO_PIN_4
#define SR_SCK_GPIO_Port GPIOC
#define SR_RCK_Pin GPIO_PIN_5
#define SR_RCK_GPIO_Port GPIOC
#define LED_06_Pin GPIO_PIN_0
#define LED_06_GPIO_Port GPIOB
#define LED_07_Pin GPIO_PIN_1
#define LED_07_GPIO_Port GPIOB
#define LED_08_Pin GPIO_PIN_2
#define LED_08_GPIO_Port GPIOB
#define SPI2_SS_Pin GPIO_PIN_9
#define SPI2_SS_GPIO_Port GPIOE
#define GPIO_16_Pin GPIO_PIN_8
#define GPIO_16_GPIO_Port GPIOC
#define GPIO_15_Pin GPIO_PIN_9
#define GPIO_15_GPIO_Port GPIOC
#define GPIO_14_Pin GPIO_PIN_9
#define GPIO_14_GPIO_Port GPIOA
#define GPIO_13_Pin GPIO_PIN_10
#define GPIO_13_GPIO_Port GPIOA
#define GPIO_12_Pin GPIO_PIN_12
#define GPIO_12_GPIO_Port GPIOA
#define GPIO_11_Pin GPIO_PIN_6
#define GPIO_11_GPIO_Port GPIOF
#define GPIO_10_Pin GPIO_PIN_7
#define GPIO_10_GPIO_Port GPIOF
#define GPIO_09_Pin GPIO_PIN_15
#define GPIO_09_GPIO_Port GPIOA
#define SPI1_SS_Pin GPIO_PIN_12
#define SPI1_SS_GPIO_Port GPIOC
#define LED_12_Pin GPIO_PIN_2
#define LED_12_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
