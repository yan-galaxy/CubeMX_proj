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
#include "BMP280.h"
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
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = 128 * 4
};
/* Definitions for ledTask */
osThreadId_t ledTaskHandle;
const osThreadAttr_t ledTask_attributes = {
  .name = "ledTask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 64 * 4
};
/* Definitions for bmpEvent */
osEventFlagsId_t bmpEventHandle;
const osEventFlagsAttr_t bmpEvent_attributes = {
  .name = "bmpEvent"
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

  /* Create the event(s) */
  /* creation of bmpEvent */
  bmpEventHandle = osEventFlagsNew(&bmpEvent_attributes);

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
Float32_union bmp_press_f_1[66];//64+2 帧头4字节 帧尾4字节
Float32_union bmp_press_f_2[66];//64+2 帧头4字节 帧尾4字节
volatile Float32_union* bmp_press_send_p=bmp_press_f_1;
volatile Float32_union* bmp_press_store_p=bmp_press_f_2;

//采集完成之后需要进行缓存切换
void exchange_res_p(void)
{
	static uint8_t state=1;//第一次运行该函数前 采集的数据存到bmp_press_f_2   第一次运行该函数后状态变为0，发送的指针指向bmp_press_f_1  之后循环往复
	state=!state;//状态翻转
	
	if(state)
	{
		bmp_press_send_p=bmp_press_f_1;
		bmp_press_store_p=bmp_press_f_2;
	}
	else
	{
		bmp_press_send_p=bmp_press_f_2;
		bmp_press_store_p=bmp_press_f_1;
	}
}

uint16_t adc_value[10];
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	char usb_buff[4096]={0};
	uint8_t dev_index=0;
	uint8_t init_state=0;
	
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	
	bmp_press_f_1[0].byte[0]=0x55;//帧头
	bmp_press_f_1[0].byte[1]=0xAA;
	bmp_press_f_1[0].byte[2]=0xBB;
	bmp_press_f_1[0].byte[3]=0xCC;
	for(uint16_t i=0;i<64;i++)
	{
		bmp_press_f_1[1+i].f32=i*1.5f;
	}
	bmp_press_f_1[65].byte[0]=0xAA;//帧尾
	bmp_press_f_1[65].byte[1]=0x55;
	bmp_press_f_1[65].byte[2]=0x66;
	bmp_press_f_1[65].byte[3]=0x77;
	
	bmp_press_f_2[0].byte[0]=0x55;//帧头
	bmp_press_f_2[0].byte[1]=0xAA;
	bmp_press_f_2[0].byte[2]=0xBB;
	bmp_press_f_2[0].byte[3]=0xCC;
	for(uint16_t i=0;i<64;i++)
	{
		bmp_press_f_2[1+i].f32=i*1.5f+1.0f;
	}
	bmp_press_f_2[65].byte[0]=0xAA;//帧尾
	bmp_press_f_2[65].byte[1]=0x55;
	bmp_press_f_2[65].byte[2]=0x66;
	bmp_press_f_2[65].byte[3]=0x77;
	
	
	for(dev_index=0;dev_index<2;dev_index++)
	{
		bmp_dev[dev_index].hspi = &hspi1;
		bmp_dev[dev_index].cs_index = dev_index;
		
		do
		{
			init_state = BMP280_Init(&(bmp_dev[dev_index]));
			if(init_state != HAL_OK)
			{
				sprintf(usb_buff,"dev[%d]初始化失败,init_state:%d\r\n",dev_index,init_state);
				CDC_Transmit_FS((uint8_t *)usb_buff,strlen(usb_buff));
//				break;
				vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(500));
			}
		}
		while(init_state != HAL_OK);
	}

//	// 发送任务通知给BMPTask
//    xTaskNotifyGive(bmpTaskHandle);
	osEventFlagsSet(bmpEventHandle, 1<<0);
	
	
  /* Infinite loop */
	for(;;)
	{
		
//		sprintf(usb_buff,"PB2 adc_value:%d\r\n",adc_value[0]);		
//		for(dev_index=0;dev_index<3;dev_index++)
//		{
//		/* 读取压力和温度 */
//			if (bmp_dev[dev_index].receive_error_flag == HAL_OK) {
//				sprintf(usb_buff+strlen(usb_buff),"dev[%2d],温度:%.2f °C,压力:%.5fkPa,海拔:%.2fm\r\n",
//				dev_index, 
//				bmp_dev[dev_index].temperature, 
////				bmp_press_store_p[1+dev_index].f32,
//				bmp_dev[dev_index].pressure, 
//				bmp_dev[dev_index].altitude);
//			} else {
//				sprintf(usb_buff+strlen(usb_buff),"dev[%2d]读取数据失败,receive_error_flag:%d\r\n",dev_index,bmp_dev[dev_index].receive_error_flag);
//			}
//		}
//		sprintf(usb_buff+strlen(usb_buff),"------------------------\r\n");
//		sprintf(usb_buff+strlen(usb_buff), "SPI:%u us Temp:%u us Press:%u us\r\n", 
//           t_spi, t_temp, t_press);
//		sprintf(usb_buff+strlen(usb_buff),"strlen(usb_buff):%d\r\n",strlen(usb_buff));
////		sprintf(usb_buff+strlen(usb_buff),"test:0x%llX\r\n",(0xFFFFFFFFFFFFFFFF & (~( (unsigned long long)(0x0000000000000001)<<(63-2) ))) );
//		CDC_Transmit_FS((uint8_t *)usb_buff,strlen(usb_buff));
		
		
		
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(500));
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
	uint8_t dev_index=0;
	HAL_StatusTypeDef error_flag=0;
	
	
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	
	HAL_GPIO_WritePin(DS_GPIO_Port,DS_Pin,0);
	HAL_GPIO_WritePin(STCP_GPIO_Port,STCP_Pin,0);
	HAL_GPIO_WritePin(SHCP_GPIO_Port,SHCP_Pin,0);
	vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
	
//	// 等待StartDefaultTask的通知，永久阻塞直到收到通知
//    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
	osEventFlagsWait(bmpEventHandle, 1<<0, osFlagsWaitAll, portMAX_DELAY);
	
	xLastWakeTime = xTaskGetTickCount();
  /* Infinite loop */
	for(;;)
	{
		
		for(dev_index=0;dev_index<64;dev_index++)
		{
			
			/* 读取压力和温度 */
			bmp_dev[dev_index].receive_error_flag = BMP280_ReadPressureTemperature(&(bmp_dev[dev_index]),dev_index,bmp_press_store_p);
		
		}
		exchange_res_p();
		CDC_Transmit_FS((uint8_t *)bmp_press_send_p, 264);
		
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(15));
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
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(200));
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

