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
#include "usbd_cdc_if.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern SPI_HandleTypeDef hspi1;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void AD5206_SetResistance(uint8_t index, uint8_t channel, uint8_t resistance);


/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define CS1_Pin GPIO_PIN_0
#define CS1_GPIO_Port GPIOB
#define CS2_Pin GPIO_PIN_1
#define CS2_GPIO_Port GPIOB
#define IO11_Pin GPIO_PIN_6
#define IO11_GPIO_Port GPIOC
#define IO4_Pin GPIO_PIN_8
#define IO4_GPIO_Port GPIOA
#define IO3_Pin GPIO_PIN_9
#define IO3_GPIO_Port GPIOA
#define IO1_Pin GPIO_PIN_15
#define IO1_GPIO_Port GPIOA
#define IO2_Pin GPIO_PIN_10
#define IO2_GPIO_Port GPIOC
#define IO5_Pin GPIO_PIN_11
#define IO5_GPIO_Port GPIOC
#define IO7_Pin GPIO_PIN_4
#define IO7_GPIO_Port GPIOB
#define IO9_Pin GPIO_PIN_5
#define IO9_GPIO_Port GPIOB
#define IO10_Pin GPIO_PIN_6
#define IO10_GPIO_Port GPIOB
#define IO8_Pin GPIO_PIN_7
#define IO8_GPIO_Port GPIOB
#define IO6_Pin GPIO_PIN_9
#define IO6_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
