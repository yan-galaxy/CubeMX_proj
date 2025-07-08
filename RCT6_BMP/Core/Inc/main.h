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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BMP280.h"
#include "stdio.h"
#include "math.h"


/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
//extern UART_HandleTypeDef huart2;

extern SPI_HandleTypeDef hspi1;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
typedef union  {
    uint16_t word16;         // 16bit
		uint8_t byte[2];       // 8bit   byte[0]:low_8bit   byte[1]:high_8bit     example:  word16=0x5678  byte[0]=0x78 byte[1]=0x56
}Word_union;

typedef struct{
	uint8_t head[4];
	float value[4];
	uint8_t tail[4];
}Protocol_struct;


/**
 * ��ͨ�˲����ṹ��
 */
typedef struct {
    float fc;       // ��ֹƵ��(Hz)
    float fs;       // ����Ƶ��(Hz)
    float alpha;    // �˲�ϵ��
    float y_prev;   // ��һʱ�����ֵ
} LowPassFilter;

/**
 * ��ͨ�˲����ṹ��
 */
typedef struct {
    float fc;       // ��ֹƵ��(Hz)
    float fs;       // ����Ƶ��(Hz)
    float alpha;    // �˲�ϵ��
    float x_prev;   // ��һʱ������ֵ
    float y_prev;   // ��һʱ�����ֵ
} HighPassFilter;

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED1_Pin GPIO_PIN_0
#define LED1_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_1
#define LED2_GPIO_Port GPIOC
#define LED3_Pin GPIO_PIN_2
#define LED3_GPIO_Port GPIOC
#define LED4_Pin GPIO_PIN_3
#define LED4_GPIO_Port GPIOC
#define LED5_Pin GPIO_PIN_4
#define LED5_GPIO_Port GPIOC
#define LED6_Pin GPIO_PIN_5
#define LED6_GPIO_Port GPIOC
#define SPI_CS1_Pin GPIO_PIN_0
#define SPI_CS1_GPIO_Port GPIOB
#define SPI_CS2_Pin GPIO_PIN_1
#define SPI_CS2_GPIO_Port GPIOB
#define SPI_CS3_Pin GPIO_PIN_2
#define SPI_CS3_GPIO_Port GPIOB
#define LED7_Pin GPIO_PIN_6
#define LED7_GPIO_Port GPIOC
#define LED8_Pin GPIO_PIN_7
#define LED8_GPIO_Port GPIOC
#define SPI_CS4_Pin GPIO_PIN_3
#define SPI_CS4_GPIO_Port GPIOB
#define SPI_CS5_Pin GPIO_PIN_4
#define SPI_CS5_GPIO_Port GPIOB
#define SPI_CS6_Pin GPIO_PIN_5
#define SPI_CS6_GPIO_Port GPIOB
#define SPI_CS7_Pin GPIO_PIN_6
#define SPI_CS7_GPIO_Port GPIOB
#define SPI_CS8_Pin GPIO_PIN_7
#define SPI_CS8_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
