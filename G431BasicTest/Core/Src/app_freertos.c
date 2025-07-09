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
osThreadId ledTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void LedTask(void const * argument);

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

  /* definition and creation of ledTask */
  osThreadDef(ledTask, LedTask, osPriorityIdle, 0, 128);
  ledTaskHandle = osThreadCreate(osThread(ledTask), NULL);

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
uint16_t adc_value[10];
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	char usb_buff[128]={0};
//	HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc_value, 10);
	AD5206_SetResistance(1,4,10);//Reef 50-1.95kΩ 100-3.85kΩ
	
	
	AD5206_SetResistance(1,2,100);//OUT3 50-1.95kΩ 100-3.85kΩ
  /* Infinite loop */
	for(;;)
	{
		HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
		sprintf(usb_buff,"PB2 adc_value:%d\r\n",adc_value[0]);
		CDC_Transmit_FS((uint8_t *)usb_buff,strlen(usb_buff));
		osDelay(500);
	}
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_LedTask */
/**
* @brief Function implementing the ledTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LedTask */
void LedTask(void const * argument)
{
  /* USER CODE BEGIN LedTask */
	TickType_t xLastWakeTime = xTaskGetTickCount();//HAL_GPIO_TogglePin
  /* Infinite loop */
	for(;;)
	{
		HAL_GPIO_WritePin(SEL1_GPIO_Port, SEL1_Pin, 0);
		HAL_GPIO_WritePin(SEL2_GPIO_Port, SEL2_Pin, 0);
		HAL_GPIO_WritePin(SEL3_GPIO_Port, SEL3_Pin, 0);
		HAL_GPIO_WritePin(SEL4_GPIO_Port, SEL4_Pin, 0);
		HAL_GPIO_WritePin(SEL5_GPIO_Port, SEL5_Pin, 0);
		HAL_GPIO_WritePin(SEL6_GPIO_Port, SEL6_Pin, 0);
		HAL_GPIO_WritePin(SEL7_GPIO_Port, SEL7_Pin, 0);
		HAL_GPIO_WritePin(SEL8_GPIO_Port, SEL8_Pin, 0);
		HAL_GPIO_WritePin(SEL9_GPIO_Port, SEL9_Pin, 0);
		HAL_GPIO_WritePin(SEL10_GPIO_Port,SEL10_Pin,0);
		
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
		
		
		
		HAL_GPIO_WritePin(SEL1_GPIO_Port, SEL1_Pin, 1);
		HAL_GPIO_WritePin(SEL2_GPIO_Port, SEL2_Pin, 1);
		HAL_GPIO_WritePin(SEL3_GPIO_Port, SEL3_Pin, 1);
		HAL_GPIO_WritePin(SEL4_GPIO_Port, SEL4_Pin, 1);
		HAL_GPIO_WritePin(SEL5_GPIO_Port, SEL5_Pin, 1);
		HAL_GPIO_WritePin(SEL6_GPIO_Port, SEL6_Pin, 1);
		HAL_GPIO_WritePin(SEL7_GPIO_Port, SEL7_Pin, 1);
		HAL_GPIO_WritePin(SEL8_GPIO_Port, SEL8_Pin, 1);
		HAL_GPIO_WritePin(SEL9_GPIO_Port, SEL9_Pin, 1);
		HAL_GPIO_WritePin(SEL10_GPIO_Port,SEL10_Pin,1);
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(3));
	}
  /* USER CODE END LedTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
// 设置AD5206的电阻值
// index: 0 1
// channel: 0~5
// resistance: 0~255
void AD5206_SetResistance(uint8_t index, uint8_t channel, uint8_t resistance) {
		uint8_t spi_data[2]={0x00,10};
		
    // 确保通道和电阻值在有效范围内
    if (channel > 5) return; // AD5206有6个通道，通道号从0到5
    if (resistance > 255) return; // 电阻值应在0到255之间
		if (index!=0 && index!=1) return;
	
		if(index==0)
			HAL_GPIO_WritePin(CS1_GPIO_Port,CS1_Pin,0);
		else if(index==1)
			HAL_GPIO_WritePin(CS2_GPIO_Port,CS2_Pin,0);
		
		spi_data[0]=channel;
		spi_data[1]=resistance;
		
		HAL_SPI_Transmit(&hspi1,spi_data,2,0xffff);
		osDelay(5);
    if(index==0)
		HAL_GPIO_WritePin(CS1_GPIO_Port,CS1_Pin,1);
	else if(index==1)
		HAL_GPIO_WritePin(CS2_GPIO_Port,CS2_Pin,1);
}
/* USER CODE END Application */

