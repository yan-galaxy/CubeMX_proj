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
#include "stm32f1xx_hal.h"

#include "stm32f1xx_ll_tim.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_exti.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_cortex.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_utils.h"
#include "stm32f1xx_ll_pwr.h"
#include "stm32f1xx_ll_dma.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef union  {
    uint16_t word16;         // 16bit
		uint8_t byte[2];       // 8bit   byte[0]:low_8bit   byte[1]:high_8bit     example:  word16=0x5678  byte[0]=0x78 byte[1]=0x56
}Word_union;

typedef struct
{//根据引脚定义修改
	uint16_t IN1;//adc1
	uint16_t IN2;
	uint16_t IN3;
	uint16_t IN4;
	uint16_t IN5;
	uint16_t IN6;
	uint16_t IN7;
	uint16_t IN8;
	uint16_t IN9;
	uint16_t IN10;
	
}Adc_IN_Struct;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern UART_HandleTypeDef huart1;
extern ADC_HandleTypeDef hadc1; 
extern DMA_HandleTypeDef hdma_usart1_tx;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void Select_switcher(uint8_t index);
void exchange_res_p(void);
void user_delaynus_tim(uint16_t nus);
void UART1_TX_DMA_Send(uint8_t *buffer, uint16_t length);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin LL_GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define EN0_Pin LL_GPIO_PIN_15
#define EN0_GPIO_Port GPIOC
#define SEL10_Pin LL_GPIO_PIN_15
#define SEL10_GPIO_Port GPIOB
#define SEL11_Pin LL_GPIO_PIN_8
#define SEL11_GPIO_Port GPIOA
#define SEL0_Pin LL_GPIO_PIN_11
#define SEL0_GPIO_Port GPIOA
#define SEL1_Pin LL_GPIO_PIN_12
#define SEL1_GPIO_Port GPIOA
#define SEL2_Pin LL_GPIO_PIN_15
#define SEL2_GPIO_Port GPIOA
#define SEL3_Pin LL_GPIO_PIN_3
#define SEL3_GPIO_Port GPIOB
#define SEL4_Pin LL_GPIO_PIN_4
#define SEL4_GPIO_Port GPIOB
#define SEL5_Pin LL_GPIO_PIN_5
#define SEL5_GPIO_Port GPIOB
#define SEL6_Pin LL_GPIO_PIN_6
#define SEL6_GPIO_Port GPIOB
#define SEL7_Pin LL_GPIO_PIN_7
#define SEL7_GPIO_Port GPIOB
#define SEL8_Pin LL_GPIO_PIN_8
#define SEL8_GPIO_Port GPIOB
#define SEL9_Pin LL_GPIO_PIN_9
#define SEL9_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
