/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
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
osThreadId capTaskHandle;
osThreadId usbTaskHandle;
osMessageQId tim3ch1QueueHandle;
osMessageQId tim2ch1QueueHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void CaptureTask(void const * argument);
void UsbTask(void const * argument);

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

  /* Create the queue(s) */
  /* definition and creation of tim3ch1Queue */
  osMessageQDef(tim3ch1Queue, 16, uint16_t);
  tim3ch1QueueHandle = osMessageCreate(osMessageQ(tim3ch1Queue), NULL);

  /* definition and creation of tim2ch1Queue */
  osMessageQDef(tim2ch1Queue, 16, uint16_t);
  tim2ch1QueueHandle = osMessageCreate(osMessageQ(tim2ch1Queue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 256);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of capTask */
  osThreadDef(capTask, CaptureTask, osPriorityLow, 0, 256);
  capTaskHandle = osThreadCreate(osThread(capTask), NULL);

  /* definition and creation of usbTask */
  osThreadDef(usbTask, UsbTask, osPriorityBelowNormal, 0, 256);
  usbTaskHandle = osThreadCreate(osThread(usbTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
//  	osTimerStart (tickTimerHandle, 1000);
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */

Tim_CaptureTypeDef Timer2_ch1_Cap;
Tim_CaptureTypeDef Timer3_ch1_Cap;
uint64_t senddata_count;
uint64_t senddata_speed;
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
	
	TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(100);
    
	
	long long usb_time=0;
	uint8_t my_TxBuf[512];//小心不要超过任务栈大小
	uint32_t my_RxLength=0;
	uint8_t USBD_Result=0;
	
	uint8_t print_txBuf[128];
	
	sprintf((char *)my_TxBuf,"123456789,times:%lld\r\n",usb_time);//+16
	sprintf((char *)my_TxBuf,"123456789,123456789,123456789,123456789,123456789,123456789,123456789,123456789,123456789,123456789,123456789,123456789,123456789,123456789,123456789,123456789,123456789,123456789,123456789,123456789,");
	
	xLastWakeTime = xTaskGetTickCount();
	while(usb_cdc_flag==0)
	{
		HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_6);
//		vTaskDelayUntil(&xLastWakeTime, xFrequency);
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
//		osDelay(100);
	}
	
	for(;;)
	{
		sprintf((char *)(my_TxBuf+200),"%lld,senddata_speed:%lldbps \r\n",usb_time,senddata_speed * 8);
		my_RxLength=strlen((char *)my_TxBuf);
		USBD_Result=CDC_Transmit_FS(my_TxBuf, my_RxLength);
		if(USBD_Result==0)
		{
			usb_time++;
			senddata_count+=my_RxLength;
		}
		
		
//		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
//		osDelay(20);
//		if(GPIOC->IDR & 0x2000)break;//HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13)
	}
	
	UBaseType_t freeStackSpace = uxTaskGetStackHighWaterMark(xTaskGetCurrentTaskHandle());
	sprintf((char *)print_txBuf,"Free Stack Space: %lu bytes\n", freeStackSpace * sizeof(StackType_t));//栈空间的单位是字  32位系统里一个字是32bit(4字节)
	CDC_Transmit_FS(print_txBuf, strlen((char *)print_txBuf));
	while (1)
	{
		HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_6);
		osDelay(500);

	}
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_CaptureTask */
/**
* @brief Function implementing the capTask thread.
* @param argument: Not used
* @retval None
*/

/* USER CODE END Header_CaptureTask */
void CaptureTask(void const * argument)
{
  /* USER CODE BEGIN CaptureTask */
	long long usb_time=0;
	uint8_t my_TxBuf[512];//小心不要超过任务栈大小
	uint32_t my_RxLength=0;
	uint8_t USBD_Result=0;
	
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start_IT(&htim6);//count
	
	
	__HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_1, TIM_ICPOLARITY_FALLING);
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);	//启动输入捕获       或者: __HAL_TIM_ENABLE(&htim5);
		
  /* Infinite loop */
  for(;;)
  {
  
    switch (Timer2_ch1_Cap.capture_Cnt){
	case 0:
		Timer2_ch1_Cap.capture_Cnt++;
		__HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_1, TIM_ICPOLARITY_FALLING);
		HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);	//启动输入捕获       或者: __HAL_TIM_ENABLE(&htim5);
		
		break;
	case 3:
		Timer2_ch1_Cap.low_time = Timer2_ch1_Cap.capture_Buf[1] - Timer2_ch1_Cap.capture_Buf[0] + (Timer2_ch1_Cap.OverflowCount << 16 );    //高电平时间
			sprintf((char *)my_TxBuf,"Timer2_ch1_Cap.low_time:%lld,%lld\r\ncapture_Buf[0]:%lu,capture_Buf[1]:%lu\r\nOverflowCount:%d\r\n\r\n",
				Timer2_ch1_Cap.low_time,usb_time,Timer2_ch1_Cap.capture_Buf[0],Timer2_ch1_Cap.capture_Buf[1],Timer2_ch1_Cap.OverflowCount);
			my_RxLength=strlen((char *)my_TxBuf);
			USBD_Result=CDC_Transmit_FS(my_TxBuf, my_RxLength);
			if(USBD_Result==0)usb_time++;
		Timer2_ch1_Cap.capture_Cnt = 0;  //清空标志位
		break;
	}
//    osDelay(1);
  }
  /* USER CODE END CaptureTask */
}

/* USER CODE BEGIN Header_UsbTask */
/**
* @brief Function implementing the usbTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UsbTask */
void UsbTask(void const * argument)
{
  /* USER CODE BEGIN UsbTask */
	long long usb_time=0;
	uint8_t my_TxBuf[512];//小心不要超过任务栈大小
	uint32_t my_RxLength=0;
	uint8_t USBD_Result=0;
	long long usb_wrong_time=0;
	
	UBaseType_t Queue_data_num=0;
	osEvent event;
  /* Infinite loop */
  for(;;)
  {
	Queue_data_num = uxQueueMessagesWaiting(tim3ch1QueueHandle);
//	if(Queue_data_num)
	{
		event=osMessageGet(tim3ch1QueueHandle,1);//osWaitForever
		if(osEventMessage==event.status)
		{
			sprintf((char *)my_TxBuf,"Timer3_ch1_Cap.low_time:%d,usb_time:%lld,usb_wrong_time:%lld,Queue_data_num:%d\n",
				event.value.v,usb_time,usb_wrong_time,Queue_data_num);
			my_RxLength=strlen((char *)my_TxBuf);
			USBD_Result=CDC_Transmit_FS(my_TxBuf, my_RxLength);
			if(USBD_Result==0)
			{
				usb_time++;
				senddata_count+=my_RxLength;
			}
			else
			{
				usb_wrong_time++;
			}
		}
	}
//	else
//	{
//		sprintf((char *)my_TxBuf,"Timer3_ch1_Cap.low_time error:0x%x,usb_time:%lld,usb_wrong_time:%lld\n",event.status,usb_time,usb_wrong_time);
//		my_RxLength=strlen((char *)my_TxBuf);
//		USBD_Result=CDC_Transmit_FS(my_TxBuf, my_RxLength);
//		if(USBD_Result==0)
//		{
//			usb_time++;
//			senddata_count+=my_RxLength;
//		}
//		else
//		{
//			usb_wrong_time++;
//		}
//	}
	
	
//    osDelay(1);
  }
  /* USER CODE END UsbTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	osEvent xReturn[10];
	if(TIM3 == htim->Instance)
	{
		switch(Timer3_ch1_Cap.capture_Cnt){
			case 0:
				Timer3_ch1_Cap.capture_Buf[0] = HAL_TIM_ReadCapturedValue(&htim3,TIM_CHANNEL_1);//获取当前的捕获值.
				Timer3_ch1_Cap.OverflowCount=0;
				__HAL_TIM_SET_CAPTUREPOLARITY(&htim3,TIM_CHANNEL_1,TIM_INPUTCHANNELPOLARITY_RISING);  //设置为下降沿捕获
				Timer3_ch1_Cap.capture_Cnt=1;
				break;
			case 1:
				Timer3_ch1_Cap.capture_Buf[1] = HAL_TIM_ReadCapturedValue(&htim3,TIM_CHANNEL_1);//获取当前的捕获值.
				Timer3_ch1_Cap.low_time = Timer3_ch1_Cap.capture_Buf[1] - Timer3_ch1_Cap.capture_Buf[0] + (Timer3_ch1_Cap.OverflowCount << 16 );    //高电平时间
				__HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_1, TIM_ICPOLARITY_FALLING);
				Timer3_ch1_Cap.capture_Cnt=0;
				xReturn[0].status=osMessagePut(tim3ch1QueueHandle,(uint16_t)Timer3_ch1_Cap.low_time,0);
				//	if(osOK!=xReturn.status)
				//	{
				//		while(1);
				////		printf("send fail\n");
				//	}
		}
	}
	else if(TIM2 == htim->Instance)
	{
		switch(Timer2_ch1_Cap.capture_Cnt){
			case 1:
				
				Timer2_ch1_Cap.capture_Buf[0] = HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_1);//获取当前的捕获值.
				Timer2_ch1_Cap.OverflowCount=0;
				__HAL_TIM_SET_CAPTUREPOLARITY(&htim2,TIM_CHANNEL_1,TIM_INPUTCHANNELPOLARITY_RISING);  //设置为下降沿捕获
				Timer2_ch1_Cap.capture_Cnt++;
				break;
			case 2:
				Timer2_ch1_Cap.capture_Buf[1] = HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_1);//获取当前的捕获值.
				HAL_TIM_IC_Stop_IT(&htim2,TIM_CHANNEL_1); //停止捕获   或者: __HAL_TIM_DISABLE(&htim5);
				Timer2_ch1_Cap.capture_Cnt++;    
		}
	
	}
	
}


/* USER CODE END Application */

