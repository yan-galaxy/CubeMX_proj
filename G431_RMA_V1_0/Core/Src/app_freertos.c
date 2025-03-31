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
osThreadId sampleTaskHandle;
osSemaphoreId dataBinarySemHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void select_switcher(uint8_t index);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void SampleTask(void const * argument);

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

  /* Create the semaphores(s) */
  /* definition and creation of dataBinarySem */
  osSemaphoreDef(dataBinarySem);
  dataBinarySemHandle = osSemaphoreCreate(osSemaphore(dataBinarySem), 1);

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
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 1024);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of sampleTask */
  osThreadDef(sampleTask, SampleTask, osPriorityHigh, 0, 128);
  sampleTaskHandle = osThreadCreate(osThread(sampleTask), NULL);

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

/*
R1 :v1 5           adc1_value[5]
R2 :v1 4           adc1_value[4]
R3 :v2 0           adc2_value[0]
R4 :v1 3           adc1_value[3]
R5 :v1 1           adc1_value[1]
R7 :v1 0           adc1_value[0]
R8 :v1 2           adc1_value[2]
R9 :v2 1           adc2_value[1]
R10:v1 6           adc1_value[6]
R11:v1 7           adc1_value[7]
*/
uint16_t samp_data[11][10];
uint16_t adc1_value[8];
uint16_t adc2_value[2];
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	uint8_t usb_TxBuf[2048];//小心不要超过任务栈大小
	uint32_t usb_RxLength=0;
	uint8_t USBD_Result=0;
	
	
	static uint32_t index = 0;          // 数据帧索引
  /* Infinite loop */
	for(;;)
	{
		HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
		
//		osSemaphoreWait(dataBinarySemHandle, osWaitForever);
		if(osSemaphoreWait(dataBinarySemHandle, 0) == osOK)
		{
			// 格式化数据帧（示例：每行11通道，共10组）
			sprintf((char *)usb_TxBuf, 
					"Frame %02d: \r\n", 
					index++);

			for (uint8_t i = 0; i < 11; i++) {
				sprintf((char *)usb_TxBuf + strlen((char *)usb_TxBuf), 
					"line:%2d:%4d %4d %4d %4d %4d %4d %4d %4d %4d %4d \r\n", 
						i+1,
						samp_data[i][0], samp_data[i][1], samp_data[i][2], 
						samp_data[i][3], samp_data[i][4], samp_data[i][5], 
						samp_data[i][6], samp_data[i][7], samp_data[i][8], 
						samp_data[i][9]);
			}
			sprintf((char *)usb_TxBuf + strlen((char *)usb_TxBuf), 
					"value: \r\n");
			for (uint8_t i = 0; i < 11; i++) {
				sprintf((char *)usb_TxBuf + strlen((char *)usb_TxBuf), 
					"line:%2d:%05.2f %05.2f %05.2f %05.2f %05.2f %05.2f %05.2f %05.2f %05.2f %05.2f \r\n", 
						i+1,
						(4095-samp_data[0][0])/(float)(4095-samp_data[i][0])*2000.0, (4095-samp_data[0][1])/(float)(4095-samp_data[i][1])*2000.0, (4095-samp_data[0][2])/(float)(4095-samp_data[i][2])*2000.0, 
						(4095-samp_data[0][3])/(float)(4095-samp_data[i][3])*2000.0, (4095-samp_data[0][4])/(float)(4095-samp_data[i][4])*2000.0, (4095-samp_data[0][5])/(float)(4095-samp_data[i][5])*2000.0, 
						(4095-samp_data[0][6])/(float)(4095-samp_data[i][6])*2000.0, (4095-samp_data[0][7])/(float)(4095-samp_data[i][7])*2000.0, (4095-samp_data[0][8])/(float)(4095-samp_data[i][8])*2000.0, 
						(4095-samp_data[0][9])/(float)(4095-samp_data[i][9])*2000.0);
			}
			
			
			
			usb_RxLength=strlen((char *)usb_TxBuf);
			CDC_Transmit_FS(usb_TxBuf, usb_RxLength);//USB CDC测试
			
			osSemaphoreRelease(dataBinarySemHandle);
		}
		
		
		
////		OUT11_GPIO_Port->BSRR = (OUT11_Pin << 16);//低电平
//		OUT11_GPIO_Port->BSRR = OUT11_Pin;//高电平
//		
//		user_delaynus_tim(25);//  1/42.5MHz*105cycles *8ch = 19.76us
//		
//		sprintf((char *)usb_TxBuf,"adc1_value 1: %4d %4d %4d %4d %4d %4d %4d %4d\r\n",adc1_value[0],adc1_value[1],adc1_value[2],adc1_value[3],adc1_value[4],adc1_value[5],adc1_value[6],adc1_value[7]);
//		usb_RxLength=strlen((char *)usb_TxBuf);
//		CDC_Transmit_FS(usb_TxBuf, usb_RxLength);//USB CDC测试
//		
//		
//		OUT11_GPIO_Port->BSRR = (OUT11_Pin << 16);//低电平
////		OUT11_GPIO_Port->BSRR = OUT11_Pin;//高电平
//		
//		user_delaynus_tim(25);
//		
//		sprintf((char *)usb_TxBuf,"adc1_value 2: %4d %4d %4d %4d %4d %4d %4d %4d\r\n",adc1_value[0],adc1_value[1],adc1_value[2],adc1_value[3],adc1_value[4],adc1_value[5],adc1_value[6],adc1_value[7]);
//		usb_RxLength=strlen((char *)usb_TxBuf);
//		
//		user_delaynus_tim(100);
//		CDC_Transmit_FS(usb_TxBuf, usb_RxLength);//USB CDC测试	
//		
//		
//		OUT11_GPIO_Port->BSRR = OUT11_Pin;//高电平
//		user_delaynus_tim(20);//  1/42.5MHz*260cycles *2ch = 12.24us
//		sprintf((char *)usb_TxBuf,"adc2_value  : %4d %4d\r\n",adc2_value[0],adc2_value[1]);
//		usb_RxLength=strlen((char *)usb_TxBuf);
//		
//		user_delaynus_tim(500);
//		CDC_Transmit_FS(usb_TxBuf, usb_RxLength);//USB CDC测试	
		
		
		
		
		
//		for(uint16_t i = 0;i<1000;i++)
//		user_delaynus_tim(1000);
		
//		user_delaynus_tim(100);
		osDelay(100);
	}
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_SampleTask */
/**
* @brief Function implementing the sampleTask thread.
* @param argument: Not used
* @retval None
*/

/* USER CODE END Header_SampleTask */
void SampleTask(void const * argument)
{
  /* USER CODE BEGIN SampleTask */
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc1_value, 8);
	HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc2_value, 2);
	
	TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(1);  // 1秒周期[6](@ref)
    
    xLastWakeTime = xTaskGetTickCount();  // 初始化基准时间[2](@ref)
  /* Infinite loop */
	for(;;)
	{
//		osSemaphoreWait(dataBinarySemHandle, osWaitForever);
		if(osSemaphoreWait(dataBinarySemHandle, 0) == osOK)
		{
			for(uint8_t i=0;i<11;i++)
			{
				select_switcher(i);
				user_delaynus_tim(25);
				samp_data[i][9]=adc1_value[5];//IN10对应R1
				samp_data[i][8]=adc1_value[4];//IN9对应R2
				samp_data[i][7]=adc2_value[0];//IN8对应R3
				samp_data[i][6]=adc1_value[3];//IN7对应R4
				samp_data[i][5]=adc1_value[1];//IN6对应R5
				samp_data[i][4]=adc1_value[0];//IN5对应R7
				samp_data[i][3]=adc1_value[2];//IN4对应R8
				samp_data[i][2]=adc2_value[1];//IN3对应R9
				samp_data[i][1]=adc1_value[6];//IN2对应R10
				samp_data[i][0]=adc1_value[7];//IN1对应R11
			}
			osSemaphoreRelease(dataBinarySemHandle);
		}
		
		
		
//		vTaskSuspend(NULL);          // 挂起当前任务（自身）
//		osDelay(1);
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
  /* USER CODE END SampleTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void select_switcher(uint8_t index)
{
	switch(index)
	{
		case 0:
			OUT10_GPIO_Port->BSRR=OUT10_Pin;//高电平
			OUT11_GPIO_Port->BSRR=(OUT11_Pin << 16);//低电平
			break;
		case 1:
			OUT11_GPIO_Port->BSRR=OUT11_Pin;//高电平
			OUT1_GPIO_Port->BSRR =(OUT1_Pin << 16);//低电平
			break;
		case 2:
			OUT1_GPIO_Port->BSRR=OUT1_Pin;//高电平
			OUT2_GPIO_Port->BSRR=(OUT2_Pin << 16);//低电平
			break;
		case 3:
			OUT2_GPIO_Port->BSRR=OUT2_Pin;//高电平
			OUT3_GPIO_Port->BSRR=(OUT3_Pin << 16);//低电平
			break;
		case 4:
			OUT3_GPIO_Port->BSRR=OUT3_Pin;//高电平
			OUT4_GPIO_Port->BSRR=(OUT4_Pin << 16);//低电平
			break;
		case 5:
			OUT4_GPIO_Port->BSRR=OUT4_Pin;//高电平
			OUT5_GPIO_Port->BSRR=(OUT5_Pin << 16);//低电平
			break;
		case 6:
			OUT5_GPIO_Port->BSRR=OUT5_Pin;//高电平
			OUT6_GPIO_Port->BSRR=(OUT6_Pin << 16);//低电平
			break;
		case 7:
			OUT6_GPIO_Port->BSRR=OUT6_Pin;//高电平
			OUT7_GPIO_Port->BSRR=(OUT7_Pin << 16);//低电平
			break;
		case 8:
			OUT7_GPIO_Port->BSRR=OUT7_Pin;//高电平
			OUT8_GPIO_Port->BSRR=(OUT8_Pin << 16);//低电平
			break;
		case 9:
			OUT8_GPIO_Port->BSRR=OUT8_Pin;//高电平
			OUT9_GPIO_Port->BSRR=(OUT9_Pin << 16);//低电平
			break;
		case 10:
			OUT9_GPIO_Port->BSRR =OUT9_Pin;//高电平
			OUT10_GPIO_Port->BSRR=(OUT10_Pin << 16);//低电平
			break;
	}
}
/* USER CODE END Application */

