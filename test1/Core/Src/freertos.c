/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
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
#include <string.h>
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
osThreadId usbTaskHandle;
osMessageQId tim3ch1QueueHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void AD5206_SetResistance1(uint8_t index, uint8_t channel, uint8_t resistance);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void UsbTask(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

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

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 1024);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of usbTask */
  osThreadDef(usbTask, UsbTask, osPriorityBelowNormal, 0, 1024);
  usbTaskHandle = osThreadCreate(osThread(usbTask), NULL);

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
Tim_CaptureTypeDef Timer3_ch1_Cap;
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
	HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_1);
	htim12.Instance->CCR1 = 2;//Vramp2 
	
	//Prescaler-Period-CCR-wave_time-resistance    100pf
	//base resistance :  3k3   2.4us latency
	//167-19-8-6us-255(10.4kOm)    
	//33-99-24-2.6us-128(5.2kOm)   
	//33-99-18-1.4us-64(2.6kOm)
	//2-999-168-700ns-32(1.3kOm)   release time~=100ns    
	//base resistance :  820 Om   1.6us latency
	//base resistance :  390 Om   1.35us latency
	//2-999-112-700ns-32(1.3kOm)   release time~=40ns 
	
	//change the NPN transistor  :2SC3356   
	//Prescaler-Period-CCR-wave_time-resistance    100pf
	//base resistance :  390 Om   50ns latency
	//2-999-45-700ns-32(1.3kOm)   release time~=40ns 
	//2-999-100-1.7us-70(2843.75Om)   release time=ns 
	//2-999-100-1.7us-200(8125Om)   release time=ns
	htim9.Instance->CCR1 = 260;//Vramp2 
	
	uint8_t i=0;
	for(i=0;i<6;i++)
	{
		AD5206_SetResistance1(0, i, 200);
	}
	
	AD5206_SetResistance1(0, 4, 9);//Vramp1  full range:10.4kOm  step:40.625Om       80(3.25kOm)
	AD5206_SetResistance1(0, 5, 200);//Vramp2
	
	osDelay(100);
	
	HAL_TIM_Base_Start_IT(&htim3);
	__HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_1, TIM_ICPOLARITY_FALLING);
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);	//启动输入捕获       或者: __HAL_TIM_ENABLE(&htim5);
	
	
  /* Infinite loop */
	for(;;)
	{
		//led
		GPIOA->BSRR = GPIO_BSRR_BS8;//high
		osDelay(100);
		GPIOA->BSRR = GPIO_BSRR_BR8;//low   charge
		osDelay(100);
		
		
	  
//		//Vramp1
//		GPIOC->BSRR = GPIO_BSRR_BS1;//high
//		osDelay(100);
//		GPIOC->BSRR = GPIO_BSRR_BR1;//low   charge
//		osDelay(100);//GPIO  71us-80(3.25kOm)   20us-20(812.5Om)    10us-9(365.625Om)
	}
  /* USER CODE END StartDefaultTask */
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
	char my_TxBuf[512];//小心不要超过任务栈大小
	uint32_t my_RxLength=0;
	uint8_t USBD_Result=0;
	
	UBaseType_t Queue_data_num=0;
	osEvent event;
  /* Infinite loop */
	for(;;)
	{
		Queue_data_num = uxQueueMessagesWaiting(tim3ch1QueueHandle);
		event=osMessageGet(tim3ch1QueueHandle,1);//osWaitForever
		if(osEventMessage==event.status)
		{
		sprintf(my_TxBuf,"event.value:%d\r\n",event.value);
//		my_RxLength=strlen(my_TxBuf);
//		USBD_Result=CDC_Transmit_FS(my_TxBuf, my_RxLength);
//		if(USBD_Result==0)usb_time++;
		}
		osDelay(100);
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
	
}
// 设置AD5206的电阻值
void AD5206_SetResistance1(uint8_t index, uint8_t channel, uint8_t resistance) {
		uint8_t spi_data[2]={0x00,10};
		
    // 确保通道和电阻值在有效范围内
    if (channel > 5) return; // AD5206有6个通道，通道号从0到5
    if (resistance > 255) return; // 电阻值应在0到255之间
		if (index!=0 && index!=1) return;
	
		if(index==0)
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,0);
		else if(index==1)
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,0);
		
		spi_data[0]=channel;
		spi_data[1]=resistance;
		
		HAL_SPI_Transmit(&hspi1,spi_data,2,0xffff);
    if(index==0)
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,1);
		else if(index==1)
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,1);
}
/* USER CODE END Application */
