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
//extern ADC_HandleTypeDef hadc2;
extern SPI_HandleTypeDef hspi1;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
void AD5206_SetResistance(uint8_t index, uint8_t channel, uint8_t resistance);
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define SEL4_Pin GPIO_PIN_2
#define SEL4_GPIO_Port GPIOB
#define SEL3_Pin GPIO_PIN_10
#define SEL3_GPIO_Port GPIOB
#define SEL5_Pin GPIO_PIN_12
#define SEL5_GPIO_Port GPIOB
#define SEL6_Pin GPIO_PIN_13
#define SEL6_GPIO_Port GPIOB
#define SEL2_Pin GPIO_PIN_14
#define SEL2_GPIO_Port GPIOB
#define SEL1_Pin GPIO_PIN_15
#define SEL1_GPIO_Port GPIOB
#define SEL9_Pin GPIO_PIN_6
#define SEL9_GPIO_Port GPIOC
#define SEL10_Pin GPIO_PIN_8
#define SEL10_GPIO_Port GPIOA
#define SEL7_Pin GPIO_PIN_9
#define SEL7_GPIO_Port GPIOA
#define SEL8_Pin GPIO_PIN_10
#define SEL8_GPIO_Port GPIOA
#define CS1_Pin GPIO_PIN_7
#define CS1_GPIO_Port GPIOB
#define CS2_Pin GPIO_PIN_9
#define CS2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
