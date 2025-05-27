/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
typedef struct
{
	uint16_t IN2;//adc1
	uint16_t IN7;
	uint16_t IN8;
	uint16_t IN9;
	uint16_t IN10;
	uint16_t IN1;//adc2
	uint16_t IN3;
	uint16_t IN4;
	uint16_t IN5;
	uint16_t IN6;
	
}Adc_IN_Struct;
Adc_IN_Struct adc_value_struct;
uint16_t adc1_value[5];
//0:PB14 IN2
//1:PA0  IN7
//2:PA1  IN8
//3:PA2  IN9
//4:PA3  IN10
uint16_t adc2_value[5];
//0:PB15 IN1
//1:PB11 IN3
//2:PB2  IN4
//3:PA5  IN5
//4:PA4  IN6
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	char usb_buff[128]={0};
	HAL_GPIO_WritePin(IO1_GPIO_Port ,IO1_Pin ,1);
	HAL_GPIO_WritePin(IO2_GPIO_Port ,IO2_Pin ,1);
	HAL_GPIO_WritePin(IO3_GPIO_Port ,IO3_Pin ,1);
	HAL_GPIO_WritePin(IO4_GPIO_Port ,IO4_Pin ,1);
	HAL_GPIO_WritePin(IO5_GPIO_Port ,IO5_Pin ,1);
	HAL_GPIO_WritePin(IO6_GPIO_Port ,IO6_Pin ,1);
	HAL_GPIO_WritePin(IO7_GPIO_Port ,IO7_Pin ,1);
	HAL_GPIO_WritePin(IO8_GPIO_Port ,IO8_Pin ,1);
	HAL_GPIO_WritePin(IO9_GPIO_Port ,IO9_Pin ,1);
	HAL_GPIO_WritePin(IO10_GPIO_Port,IO10_Pin,1);
	HAL_GPIO_WritePin(IO11_GPIO_Port,IO11_Pin,1);
	
//	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc1_value, 5);
//	HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc2_value, 5);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)(&(adc_value_struct.IN2)), 5);
	HAL_ADC_Start_DMA(&hadc2, (uint32_t *)(&(adc_value_struct.IN1)), 5);
//	adc_value_struct.IN2=1;
  /* Infinite loop */
	for(;;)
	{
		HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
//		sprintf(usb_buff,"IN1:%d ,IN2:%d ,IN3:%d ,IN4:%d ,IN5:%d ,IN6:%d ,IN7:%d ,IN8:%d ,IN9:%d ,IN10:%d\r\n",
//			adc2_value[0],adc1_value[0],adc2_value[1],adc2_value[2],adc2_value[3],adc2_value[4],adc1_value[1],adc1_value[2],adc1_value[3],adc1_value[4]);
		sprintf(usb_buff,"IN1:%d ,IN2:%d ,IN3:%d ,IN4:%d ,IN5:%d ,IN6:%d ,IN7:%d ,IN8:%d ,IN9:%d ,IN10:%d\r\n",
			adc_value_struct.IN1,adc_value_struct.IN2,adc_value_struct.IN3,adc_value_struct.IN4,adc_value_struct.IN5,adc_value_struct.IN6,adc_value_struct.IN7,adc_value_struct.IN8,adc_value_struct.IN9,adc_value_struct.IN10);
//		sprintf(usb_buff,"adc_value_struct:%p,adc_value_struct.IN2:%p,adc_value_struct.IN7:%p,adc_value_struct.IN1:%p\r\n",&adc_value_struct,&(adc_value_struct.IN2),&(adc_value_struct.IN7),&(adc_value_struct.IN1));
		CDC_Transmit_FS((uint8_t *)usb_buff,strlen(usb_buff));
		
		
		
		osDelay(500);
	}
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

