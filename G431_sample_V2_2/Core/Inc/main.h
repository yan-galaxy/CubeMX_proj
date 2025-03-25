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
typedef struct
{
	uint32_t capture_Buf[2];   //存放计数值
	uint32_t OverflowCount;
	uint8_t capture_Cnt;    //状态标志位
	int64_t low_time;   //高电平时间
} Tim_CaptureTypeDef;
typedef union  {
    uint16_t word16;         // 16bit
		uint8_t byte[2];       // 8bit   byte[0]:low_8bit   byte[1]:high_8bit     example:  word16=0x5678  byte[0]=0x78 byte[1]=0x56
}Word_union;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern SPI_HandleTypeDef hspi1;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define KEY_Pin GPIO_PIN_13
#define KEY_GPIO_Port GPIOC
#define TEST_Pin GPIO_PIN_0
#define TEST_GPIO_Port GPIOA
#define CS1_Pin GPIO_PIN_4
#define CS1_GPIO_Port GPIOC
#define CS2_Pin GPIO_PIN_2
#define CS2_GPIO_Port GPIOB
#define SEL10_Pin GPIO_PIN_10
#define SEL10_GPIO_Port GPIOB
#define SEL4_Pin GPIO_PIN_12
#define SEL4_GPIO_Port GPIOB
#define SEL5_Pin GPIO_PIN_13
#define SEL5_GPIO_Port GPIOB
#define SEL3_Pin GPIO_PIN_14
#define SEL3_GPIO_Port GPIOB
#define SEL6_Pin GPIO_PIN_15
#define SEL6_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_6
#define LED_GPIO_Port GPIOC
#define SEL7_Pin GPIO_PIN_8
#define SEL7_GPIO_Port GPIOA
#define SEL2_Pin GPIO_PIN_15
#define SEL2_GPIO_Port GPIOA
#define SEL8_Pin GPIO_PIN_10
#define SEL8_GPIO_Port GPIOC
#define SEL1_Pin GPIO_PIN_11
#define SEL1_GPIO_Port GPIOC
#define SEL9_Pin GPIO_PIN_3
#define SEL9_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
// (1 << SEL1_pin_num)<<16
#define SEL1_pin_LOW  0x08000000	//PC11	
#define SEL2_pin_LOW  0x80000000	//PA15
#define SEL3_pin_LOW  0x40000000	//PB14
#define SEL4_pin_LOW  0x10000000	//PB12
#define SEL5_pin_LOW  0x20000000	//PB13
#define SEL6_pin_LOW  0x80000000	//PB15
#define SEL7_pin_LOW  0x01000000	//PA8
#define SEL8_pin_LOW  0x04000000	//PC10
#define SEL9_pin_LOW  0x00080000	//PB3
#define SEL10_pin_LOW 0x04000000	//PB10

// 1 << SEL1_pin_num
#define SEL1_pin_HIGH  0x00000800	//PC11
#define SEL2_pin_HIGH  0x00008000	//PA15
#define SEL3_pin_HIGH  0x00004000	//PB14
#define SEL4_pin_HIGH  0x00001000	//PB12
#define SEL5_pin_HIGH  0x00002000	//PB13
#define SEL6_pin_HIGH  0x00008000	//PB15
#define SEL7_pin_HIGH  0x00000100	//PA8
#define SEL8_pin_HIGH  0x00000400	//PC10
#define SEL9_pin_HIGH  0x00000008	//PB3
#define SEL10_pin_HIGH 0x00000400	//PB10
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
