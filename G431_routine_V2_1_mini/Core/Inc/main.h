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
#include "usbd_cdc_if.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef union  {
    uint16_t word16;         // 16bit
		uint8_t byte[2];       // 8bit   byte[0]:low_8bit   byte[1]:high_8bit     example:  word16=0x5678  byte[0]=0x78 byte[1]=0x56
}Word_union;

typedef struct
{//根据引脚定义修改
	uint16_t IN2;//adc1
	uint16_t IN4;
	uint16_t IN8;
	uint16_t IN7;
	uint16_t IN10;
	uint16_t IN1;//adc2
	uint16_t IN3;
	uint16_t IN6;
	uint16_t IN5;
	uint16_t IN9;
	
}Adc_IN_Struct;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
//extern SPI_HandleTypeDef hspi1;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
void AD5206_SetResistance(uint8_t index, uint8_t channel, uint8_t resistance);
void Select_switcher(uint8_t index);
void user_delaynus_tim(uint16_t nus);
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */


// (1 << SEL1_pin_num)<<16
#define SEL1_pin_LOW  0x04000000	//PA10
#define SEL2_pin_LOW  0x02000000	//PA9 
#define SEL3_pin_LOW  0x01000000	//PA8 
#define SEL4_pin_LOW  0x00400000	//PC6 
#define SEL5_pin_LOW  0x04000000	//PB10
#define SEL6_pin_LOW  0x00040000	//PB2 
#define SEL7_pin_LOW  0x80000000	//PB15
#define SEL8_pin_LOW  0x10000000	//PB12
#define SEL9_pin_LOW  0x40000000	//PB14
#define SEL10_pin_LOW 0x20000000	//PB13

// 1 << SEL1_pin_num
#define SEL1_pin_HIGH  0x0000400	//PA10
#define SEL2_pin_HIGH  0x0000200	//PA9 
#define SEL3_pin_HIGH  0x0000100	//PA8 
#define SEL4_pin_HIGH  0x0000040	//PC6 
#define SEL5_pin_HIGH  0x0000400	//PB10
#define SEL6_pin_HIGH  0x0000004	//PB2 
#define SEL7_pin_HIGH  0x0008000	//PB15
#define SEL8_pin_HIGH  0x0001000	//PB12
#define SEL9_pin_HIGH  0x0004000	//PB14
#define SEL10_pin_HIGH 0x0002000	//PB13


/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define SEL6_Pin GPIO_PIN_2
#define SEL6_GPIO_Port GPIOB
#define SEL5_Pin GPIO_PIN_10
#define SEL5_GPIO_Port GPIOB
#define SEL10_Pin GPIO_PIN_12
#define SEL10_GPIO_Port GPIOB
#define SEL9_Pin GPIO_PIN_13
#define SEL9_GPIO_Port GPIOB
#define SEL8_Pin GPIO_PIN_14
#define SEL8_GPIO_Port GPIOB
#define SEL7_Pin GPIO_PIN_15
#define SEL7_GPIO_Port GPIOB
#define SEL4_Pin GPIO_PIN_6
#define SEL4_GPIO_Port GPIOC
#define SEL3_Pin GPIO_PIN_8
#define SEL3_GPIO_Port GPIOA
#define SEL2_Pin GPIO_PIN_9
#define SEL2_GPIO_Port GPIOA
#define SEL1_Pin GPIO_PIN_10
#define SEL1_GPIO_Port GPIOA
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
