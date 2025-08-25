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
osThreadId bmpTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void BMPTask(void const * argument);

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
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 512);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of bmpTask */
  osThreadDef(bmpTask, BMPTask, osPriorityAboveNormal, 0, 128);
  bmpTaskHandle = osThreadCreate(osThread(bmpTask), NULL);

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
BMP280_Device bmp_dev1;
uint16_t adc_value[10];
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	char usb_buff[1024]={0};
	bmp_dev1.hspi = &hspi1;
    bmp_dev1.cs_index = 0;
	
	while(BMP280_Init(&bmp_dev1) != HAL_OK)
	{
		sprintf(usb_buff,"dev1初始化失败\r\n");
		CDC_Transmit_FS((uint8_t *)usb_buff,strlen(usb_buff));
		osDelay(500);
	}
  /* Infinite loop */
	for(;;)
	{
		HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
		sprintf(usb_buff,"PB2 adc_value:%d\r\n",adc_value[0]);
//		CDC_Transmit_FS((uint8_t *)usb_buff,strlen(usb_buff));
		
		/* 读取压力和温度 */
			if (BMP280_ReadPressureTemperature(&bmp_dev1) == HAL_OK) {
				sprintf(usb_buff+strlen(usb_buff),"dev1\r\n");
				sprintf(usb_buff+strlen(usb_buff),"温度: %.2f °C\r\n", bmp_dev1.temperature);
				sprintf(usb_buff+strlen(usb_buff),"压力: %.2f kPa\r\n", bmp_dev1.pressure);
				sprintf(usb_buff+strlen(usb_buff),"海拔: %.2f m\r\n", bmp_dev1.altitude);
				sprintf(usb_buff+strlen(usb_buff),"------------------------\r\n");
			} else {
				sprintf(usb_buff,"dev1读取数据失败\r\n");
			}
		CDC_Transmit_FS((uint8_t *)usb_buff,strlen(usb_buff));
		
		osDelay(500);
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
void BMPTask(void const * argument)
{
  /* USER CODE BEGIN BMPTask */
	
	
	
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	
	HAL_GPIO_WritePin(DS_GPIO_Port,DS_Pin,0);
	HAL_GPIO_WritePin(STCP_GPIO_Port,STCP_Pin,0);
	HAL_GPIO_WritePin(SHCP_GPIO_Port,SHCP_Pin,0);
	vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
	
	
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
		
		
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
	}
  /* USER CODE END BMPTask */
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
void Input_74HC595_CH8(uint8_t DS_8)//DS_8低位为bit7，高位为bit0
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
void user_delaynus_tim(uint16_t nus)
{
	
	LL_TIM_SetCounter(TIM7, 0);
	while (LL_TIM_GetCounter(TIM7) < nus)
	{
		;// Optionally, add a timeout condition here to avoid an infinite loop
	}
}	
/* USER CODE END Application */

