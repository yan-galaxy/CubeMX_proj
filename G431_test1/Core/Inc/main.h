/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_exti.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_dma.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>          
#include <string.h>  
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim17;
extern SPI_HandleTypeDef hspi1;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
typedef struct
{
	uint32_t capture_Buf[2];   //存放计数值
	uint32_t OverflowCount;
	uint8_t capture_Cnt;    //状态标志位
	int32_t low_time;   //高电平时间
} Tim_CaptureTypeDef;

extern uint8_t res_flags[10];
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
#define KEY_Pin LL_GPIO_PIN_13
#define KEY_GPIO_Port GPIOC
#define CS1_Pin LL_GPIO_PIN_4
#define CS1_GPIO_Port GPIOC
#define CS2_Pin LL_GPIO_PIN_2
#define CS2_GPIO_Port GPIOB
#define SEL6_Pin LL_GPIO_PIN_10
#define SEL6_GPIO_Port GPIOB
#define wave_Pin LL_GPIO_PIN_11
#define wave_GPIO_Port GPIOB
#define SEL7_Pin LL_GPIO_PIN_12
#define SEL7_GPIO_Port GPIOB
#define SEL8_Pin LL_GPIO_PIN_13
#define SEL8_GPIO_Port GPIOB
#define LED_Pin LL_GPIO_PIN_6
#define LED_GPIO_Port GPIOC
#define SEL1_Pin LL_GPIO_PIN_8
#define SEL1_GPIO_Port GPIOA
#define SEL2_Pin LL_GPIO_PIN_9
#define SEL2_GPIO_Port GPIOA
#define SEL3_Pin LL_GPIO_PIN_10
#define SEL3_GPIO_Port GPIOA
#define SEL4_Pin LL_GPIO_PIN_15
#define SEL4_GPIO_Port GPIOA
#define SEL9_Pin LL_GPIO_PIN_10
#define SEL9_GPIO_Port GPIOC
#define SEL10_Pin LL_GPIO_PIN_11
#define SEL10_GPIO_Port GPIOC
#define SEL5_Pin LL_GPIO_PIN_3
#define SEL5_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define SEL1_pin_num 8
#define SEL2_pin_num 9
#define SEL3_pin_num 10
#define SEL4_pin_num 15
#define SEL5_pin_num 3
#define SEL6_pin_num 10
#define SEL7_pin_num 12
#define SEL8_pin_num 13
#define SEL9_pin_num 10
#define SEL10_pin_num 11

#define SEL1_pin_LOW  0x01000000	// (1 << SEL1_pin_num)<<16
#define SEL2_pin_LOW  0x02000000
#define SEL3_pin_LOW  0x04000000
#define SEL4_pin_LOW  0x80000000
#define SEL5_pin_LOW  0x00080000
#define SEL6_pin_LOW  0x04000000
#define SEL7_pin_LOW  0x10000000
#define SEL8_pin_LOW  0x20000000
#define SEL9_pin_LOW  0x04000000
#define SEL10_pin_LOW 0x08000000

#define SEL1_pin_HIGH  0x00000100	// 1 << SEL1_pin_num
#define SEL2_pin_HIGH  0x00000200
#define SEL3_pin_HIGH  0x00000400
#define SEL4_pin_HIGH  0x00008000
#define SEL5_pin_HIGH  0x00000008
#define SEL6_pin_HIGH  0x00000400
#define SEL7_pin_HIGH  0x00001000
#define SEL8_pin_HIGH  0x00002000
#define SEL9_pin_HIGH  0x00000400
#define SEL10_pin_HIGH 0x00000800
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
