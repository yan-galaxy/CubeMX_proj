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
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 2048 * 4
};
/* Definitions for bmpTask */
osThreadId_t bmpTaskHandle;
const osThreadAttr_t bmpTask_attributes = {
  .name = "bmpTask",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 128 * 4
};
/* Definitions for ledTask */
osThreadId_t ledTaskHandle;
const osThreadAttr_t ledTask_attributes = {
  .name = "ledTask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 64 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void BMPTask(void *argument);
void LEDTask(void *argument);

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of bmpTask */
  bmpTaskHandle = osThreadNew(BMPTask, NULL, &bmpTask_attributes);

  /* creation of ledTask */
  ledTaskHandle = osThreadNew(LEDTask, NULL, &ledTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
BMP280_Device bmp_dev[64];

uint16_t adc_value[10];
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	char usb_buff[4096]={0};
	uint8_t dev_index=0;
	
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	
	for(dev_index=0;dev_index<64;dev_index++)
	{
		bmp_dev[dev_index].hspi = &hspi1;
		bmp_dev[dev_index].cs_index = dev_index;
		
		while(BMP280_Init(&(bmp_dev[dev_index])) != HAL_OK)
		{
			sprintf(usb_buff,"dev[%d]初始化失败\r\n",dev_index);
			CDC_Transmit_FS((uint8_t *)usb_buff,strlen(usb_buff));
			break;
			
			vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(500));
//			osDelay(500);
		}
	}

//	// 发送任务通知给BMPTask
//    xTaskNotifyGive(bmpTaskHandle);
	
	
  /* Infinite loop */
	for(;;)
	{
		
		sprintf(usb_buff,"PB2 adc_value:%d\r\n",adc_value[0]);
//		CDC_Transmit_FS((uint8_t *)usb_buff,strlen(usb_buff));
		
		for(dev_index=0;dev_index<64;dev_index++)
		{
		/* 读取压力和温度 */
			if (BMP280_ReadPressureTemperature(&(bmp_dev[dev_index]),dev_index) == HAL_OK) {
				sprintf(usb_buff+strlen(usb_buff),"dev[%2d],温度:%.2f °C,压力:%.2fkPa,海拔:%.2fm\r\n",dev_index, bmp_dev[dev_index].temperature, bmp_dev[dev_index].pressure, bmp_dev[dev_index].altitude);
			} else {
				sprintf(usb_buff+strlen(usb_buff),"dev[%2d]读取数据失败\r\n",dev_index);
			}
		}
		sprintf(usb_buff+strlen(usb_buff),"------------------------\r\n");
		sprintf(usb_buff+strlen(usb_buff),"strlen(usb_buff):%d\r\n",strlen(usb_buff));
//		sprintf(usb_buff+strlen(usb_buff),"test:0x%llX\r\n",(0xFFFFFFFFFFFFFFFF & (~( (unsigned long long)(0x0000000000000001)<<(63-2) ))) );
		CDC_Transmit_FS((uint8_t *)usb_buff,strlen(usb_buff));
		
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
//		osDelay(50);
	}
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_BMPTask */
/**
* @brief Function implementing the bmpTask thread.
* @param argument: Not used
* @retval None
*/

/* USER CODE END Header_BMPTask */
void BMPTask(void *argument)
{
  /* USER CODE BEGIN BMPTask */
	
	
	
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	
	HAL_GPIO_WritePin(DS_GPIO_Port,DS_Pin,0);
	HAL_GPIO_WritePin(STCP_GPIO_Port,STCP_Pin,0);
	HAL_GPIO_WritePin(SHCP_GPIO_Port,SHCP_Pin,0);
	vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
	
//	// 等待StartDefaultTask的通知，永久阻塞直到收到通知
//    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
	
  /* Infinite loop */
	for(;;)
	{
		
		
//		Input_74HC595(0);
//		user_delaynus_tim(200);
//		Input_74HC595(1);
//		user_delaynus_tim(300);
		
		
//		Input_74HC595_CH8(0xAA);
//		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(2));
//		Input_74HC595_CH8(0x55);
//		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
		
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(500));
	}
  /* USER CODE END BMPTask */
}

/* USER CODE BEGIN Header_LEDTask */
/**
* @brief Function implementing the ledTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LEDTask */
void LEDTask(void *argument)
{
  /* USER CODE BEGIN LEDTask */
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
  /* Infinite loop */
	for(;;)
	{
		HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(500));
	}
  /* USER CODE END LEDTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void Input_74HC595(uint8_t DS)
{
	HAL_GPIO_WritePin(DS_GPIO_Port,DS_Pin,DS);//输入 DS
	user_delaynus_tim(1);
	HAL_GPIO_WritePin(SHCP_GPIO_Port,SHCP_Pin,1);//数据载入
	user_delaynus_tim(1);
	HAL_GPIO_WritePin(SHCP_GPIO_Port,SHCP_Pin,0);
	HAL_GPIO_WritePin(STCP_GPIO_Port,STCP_Pin,1);//数据输出 此时真实输出为DS指定的电平
	user_delaynus_tim(1);
	HAL_GPIO_WritePin(STCP_GPIO_Port,STCP_Pin,0);
}
void Input_74HC595_CH8(uint8_t DS_8)//DS_8从低位到高位为bit7~bit0
{
	for(uint8_t i=0;i<8;i++)
	{
		HAL_GPIO_WritePin(DS_GPIO_Port,DS_Pin, ((DS_8>>i) & 0x01) );//输入 DS
		user_delaynus_tim(1);
		HAL_GPIO_WritePin(SHCP_GPIO_Port,SHCP_Pin,1);//数据载入
		user_delaynus_tim(1);
		HAL_GPIO_WritePin(SHCP_GPIO_Port,SHCP_Pin,0);
	}
	HAL_GPIO_WritePin(STCP_GPIO_Port,STCP_Pin,1);//数据输出 此时真实输出为DS_8指定的8通道电平
	user_delaynus_tim(1);
	HAL_GPIO_WritePin(STCP_GPIO_Port,STCP_Pin,0);
}
void Input_74HC595_CH64(uint64_t DS_64)//DS_64从低位到高位为bit63~bit0
{
	for(uint8_t i=0;i<64;i++)
	{
		HAL_GPIO_WritePin(DS_GPIO_Port,DS_Pin, ((DS_64>>i) & 0x0000000000000001) );//输入 DS
		user_delaynus_tim(1);
		HAL_GPIO_WritePin(SHCP_GPIO_Port,SHCP_Pin,1);//数据载入
		user_delaynus_tim(1);
		HAL_GPIO_WritePin(SHCP_GPIO_Port,SHCP_Pin,0);
	}
	HAL_GPIO_WritePin(STCP_GPIO_Port,STCP_Pin,1);//数据输出 此时真实输出为DS_8指定的8通道电平
	user_delaynus_tim(1);
	HAL_GPIO_WritePin(STCP_GPIO_Port,STCP_Pin,0);
}
void user_delaynus_tim(uint16_t nus)
{
	
	LL_TIM_SetCounter(TIM7, 0);
	while (LL_TIM_GetCounter(TIM7) < nus)
	{
		;// Optionally, add a timeout condition here to avoid an infinite loop
	}
}	
/* USER CODE END Application */

