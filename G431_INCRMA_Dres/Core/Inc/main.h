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

#include "stm32g4xx_ll_tim.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_dma.h"

#include "stm32g4xx_ll_exti.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>          
#include <string.h>  
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

extern SPI_HandleTypeDef hspi1;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
typedef union  {
    uint16_t word16;         // 16bit
		uint8_t byte[2];       // 8bit   byte[0]:low_8bit   byte[1]:high_8bit     example:  word16=0x5678  byte[0]=0x78 byte[1]=0x56
}Word_union;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void select_switcher(uint8_t index);
void select_switcher_2(uint8_t index);
void AD5206_SetResistance(uint8_t index, uint8_t channel, uint8_t resistance);
void user_delaynus_tim(uint16_t nus);
void exchange_res_p(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CS1_Pin GPIO_PIN_4
#define CS1_GPIO_Port GPIOC
#define CS2_Pin GPIO_PIN_2
#define CS2_GPIO_Port GPIOB
#define OUT11_Pin GPIO_PIN_15
#define OUT11_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_6
#define LED_GPIO_Port GPIOC
#define OUT1_Pin GPIO_PIN_8
#define OUT1_GPIO_Port GPIOA
#define OUT10_Pin GPIO_PIN_15
#define OUT10_GPIO_Port GPIOA
#define OUT2_Pin GPIO_PIN_10
#define OUT2_GPIO_Port GPIOC
#define OUT9_Pin GPIO_PIN_11
#define OUT9_GPIO_Port GPIOC
#define OUT3_Pin GPIO_PIN_3
#define OUT3_GPIO_Port GPIOB
#define OUT8_Pin GPIO_PIN_4
#define OUT8_GPIO_Port GPIOB
#define OUT4_Pin GPIO_PIN_5
#define OUT4_GPIO_Port GPIOB
#define OUT7_Pin GPIO_PIN_6
#define OUT7_GPIO_Port GPIOB
#define OUT5_Pin GPIO_PIN_7
#define OUT5_GPIO_Port GPIOB
#define OUT6_Pin GPIO_PIN_9
#define OUT6_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
