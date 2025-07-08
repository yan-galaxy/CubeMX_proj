/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
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
#include "usbd_cdc_if.h"
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
void LED_blink_1_4(void);
float LowPassFilter(float current, float fc, float fs, float* prev_lowpass_output, float* highpass_output);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void LedTask(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

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
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 1024);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of ledTask */
  osThreadDef(ledTask, LedTask, osPriorityLow, 0, 128);
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
#define PRINT_MESSAGE 0

float LowPass_value1=0.0,Prev_value1=0.0,HighPass_value1=0.0;
float LowPass_value2=0.0,Prev_value2=0.0,HighPass_value2=0.0;
float LowPass_value3=0.0,Prev_value3=0.0,HighPass_value3=0.0;
float LowPass_value4=0.0,Prev_value4=0.0,HighPass_value4=0.0;
BMP280_Device bmp_dev1;
BMP280_Device bmp_dev2;
BMP280_Device bmp_dev3;
BMP280_Device bmp_dev4;
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
	char usb_TxBuf[1024]={0};
	
	bmp_dev1.hspi = &hspi1;
    bmp_dev1.cs_port = SPI_CS1_GPIO_Port;
    bmp_dev1.cs_pin = SPI_CS1_Pin;
	
	bmp_dev2.hspi = &hspi1;
    bmp_dev2.cs_port = SPI_CS2_GPIO_Port;
    bmp_dev2.cs_pin = SPI_CS2_Pin;
	
	bmp_dev3.hspi = &hspi1;
    bmp_dev3.cs_port = SPI_CS3_GPIO_Port;
    bmp_dev3.cs_pin = SPI_CS3_Pin;
	
	bmp_dev4.hspi = &hspi1;
//    bmp_dev4.cs_port = SPI_CS4_GPIO_Port;
//    bmp_dev4.cs_pin = SPI_CS4_Pin;
	bmp_dev4.cs_port = GPIOA;
	bmp_dev4.cs_pin = GPIO_PIN_2;
	
	while(BMP280_Init(&bmp_dev1) != HAL_OK)
	{
		sprintf(usb_TxBuf,"dev1初始化失败\r\n");
		CDC_Transmit_FS((uint8_t *)usb_TxBuf,strlen(usb_TxBuf));
		osDelay(500);
	}
	while(BMP280_Init(&bmp_dev2) != HAL_OK)
	{
		sprintf(usb_TxBuf,"dev2初始化失败\r\n");
		CDC_Transmit_FS((uint8_t *)usb_TxBuf,strlen(usb_TxBuf));
		osDelay(500);
	}
	while(BMP280_Init(&bmp_dev3) != HAL_OK)
	{
		sprintf(usb_TxBuf,"dev3初始化失败\r\n");
		CDC_Transmit_FS((uint8_t *)usb_TxBuf,strlen(usb_TxBuf));
		osDelay(500);
	}
	while(BMP280_Init(&bmp_dev4) != HAL_OK)
	{
		sprintf(usb_TxBuf,"dev4初始化失败\r\n");
		CDC_Transmit_FS((uint8_t *)usb_TxBuf,strlen(usb_TxBuf));
		osDelay(500);
	}
	
	TickType_t xLastWakeTime = xTaskGetTickCount();
  /* Infinite loop */
	for(;;)
	{
		#if PRINT_MESSAGE == 1
		  sprintf(usb_TxBuf,"\r\n\r\n");
		  /* 读取压力和温度 */
			if (BMP280_ReadPressureTemperature(&bmp_dev1) == HAL_OK) {
				sprintf(usb_TxBuf+strlen(usb_TxBuf),"dev1\r\n");
				sprintf(usb_TxBuf+strlen(usb_TxBuf),"温度: %.2f °C\r\n", bmp_dev1.temperature);
				sprintf(usb_TxBuf+strlen(usb_TxBuf),"压力: %.2f kPa\r\n", bmp_dev1.pressure);
				sprintf(usb_TxBuf+strlen(usb_TxBuf),"海拔: %.2f m\r\n", bmp_dev1.altitude);
				sprintf(usb_TxBuf+strlen(usb_TxBuf),"------------------------\r\n");
			} else {
				sprintf(usb_TxBuf,"dev1读取数据失败\r\n");
			}
			if (BMP280_ReadPressureTemperature(&bmp_dev2) == HAL_OK) {
				sprintf(usb_TxBuf+strlen(usb_TxBuf),"dev2\r\n");
				sprintf(usb_TxBuf+strlen(usb_TxBuf),"温度: %.2f °C\r\n", bmp_dev2.temperature);
				sprintf(usb_TxBuf+strlen(usb_TxBuf),"压力: %.2f kPa\r\n", bmp_dev2.pressure);
				sprintf(usb_TxBuf+strlen(usb_TxBuf),"海拔: %.2f m\r\n", bmp_dev2.altitude);
				sprintf(usb_TxBuf+strlen(usb_TxBuf),"------------------------\r\n");
			} else {
				sprintf(usb_TxBuf,"dev2读取数据失败\r\n");
			}
			if (BMP280_ReadPressureTemperature(&bmp_dev3) == HAL_OK) {
				sprintf(usb_TxBuf+strlen(usb_TxBuf),"dev3\r\n");
				sprintf(usb_TxBuf+strlen(usb_TxBuf),"温度: %.2f °C\r\n", bmp_dev3.temperature);
				sprintf(usb_TxBuf+strlen(usb_TxBuf),"压力: %.2f kPa\r\n", bmp_dev3.pressure);
				sprintf(usb_TxBuf+strlen(usb_TxBuf),"海拔: %.2f m\r\n", bmp_dev3.altitude);
				sprintf(usb_TxBuf+strlen(usb_TxBuf),"------------------------\r\n");
			} else {
				sprintf(usb_TxBuf,"dev3读取数据失败\r\n");
			}
			if (BMP280_ReadPressureTemperature(&bmp_dev4) == HAL_OK) {
				sprintf(usb_TxBuf+strlen(usb_TxBuf),"dev4\r\n");
				sprintf(usb_TxBuf+strlen(usb_TxBuf),"温度: %.2f °C\r\n", bmp_dev4.temperature);
				sprintf(usb_TxBuf+strlen(usb_TxBuf),"压力: %.2f kPa\r\n", bmp_dev4.pressure);
				sprintf(usb_TxBuf+strlen(usb_TxBuf),"海拔: %.2f m\r\n", bmp_dev4.altitude);
				sprintf(usb_TxBuf+strlen(usb_TxBuf),"------------------------\r\n");
			} else {
				sprintf(usb_TxBuf,"dev4读取数据失败\r\n");
			}
		#endif
			
		#if PRINT_MESSAGE == 0
			BMP280_ReadPressureTemperature(&bmp_dev1);
			BMP280_ReadPressureTemperature(&bmp_dev2);
			BMP280_ReadPressureTemperature(&bmp_dev3);
			BMP280_ReadPressureTemperature(&bmp_dev4);
			
			LowPass_value1=LowPassFilter(bmp_dev1.altitude,40,100,&Prev_value1,&HighPass_value1);
			LowPass_value2=LowPassFilter(bmp_dev2.altitude,40,100,&Prev_value2,&HighPass_value2);
			LowPass_value3=LowPassFilter(bmp_dev3.altitude,40,100,&Prev_value3,&HighPass_value3);
			LowPass_value4=LowPassFilter(bmp_dev4.altitude,40,100,&Prev_value4,&HighPass_value4);
			
//			sprintf(usb_TxBuf,"%f,%f\r\n",bmp_dev1.pressure,LowPass_value1);
			sprintf(usb_TxBuf,"%f,%f,%f,%f\r\n",LowPass_value1,LowPass_value2,LowPass_value3,LowPass_value4);
		#endif
		
//		HAL_GPIO_TogglePin(SPI_CS1_GPIO_Port,SPI_CS1_Pin);
//		HAL_GPIO_TogglePin(SPI_CS2_GPIO_Port,SPI_CS2_Pin);
//		HAL_GPIO_TogglePin(SPI_CS3_GPIO_Port,SPI_CS3_Pin);
//		HAL_GPIO_TogglePin(SPI_CS4_GPIO_Port,SPI_CS4_Pin);
//		
//		HAL_GPIO_TogglePin(SPI_CS5_GPIO_Port,SPI_CS5_Pin);
//		HAL_GPIO_TogglePin(SPI_CS6_GPIO_Port,SPI_CS6_Pin);
//		HAL_GPIO_TogglePin(SPI_CS7_GPIO_Port,SPI_CS7_Pin);
//		HAL_GPIO_TogglePin(SPI_CS8_GPIO_Port,SPI_CS8_Pin);
//		
//		HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_2);
		
//		LED_blink_1_4();
//		HAL_GPIO_TogglePin(LED5_GPIO_Port,LED5_Pin);
//		HAL_GPIO_TogglePin(LED6_GPIO_Port,LED6_Pin);
//		HAL_GPIO_TogglePin(LED7_GPIO_Port,LED7_Pin);
//		HAL_GPIO_TogglePin(LED8_GPIO_Port,LED8_Pin);
		
		CDC_Transmit_FS((uint8_t *)usb_TxBuf,strlen(usb_TxBuf));
//		osDelay(20);
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
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
	TickType_t xLastWakeTime = xTaskGetTickCount();
  /* Infinite loop */
	for(;;)
	{
		if(HighPass_value1>=0.5)	HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,0); else	HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,1);
		if(HighPass_value2>=0.5)	HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,0); else	HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,1);
		if(HighPass_value3>=0.5)	HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,0); else	HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,1);
		if(HighPass_value4>=0.5)	HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin,0); else	HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin,1);
		
//		osDelay(10);
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
	}
  /* USER CODE END LedTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void LED_blink_1_4(void)
{
	static uint8_t cnt=0;
	switch(cnt)
	{
		case 0:
		{
			HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,0);
			HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,1);
			HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,1);
			HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin,1);
			break;
		}
		case 1:
		{
			HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,1);
			HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,0);
			HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,1);
			HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin,1);
			break;
		}
		case 2:
		{
			HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,1);
			HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,1);
			HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,0);
			HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin,1);
			break;
		}
		case 3:
		{
			HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,1);
			HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,1);
			HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,1);
			HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin,0);
			break;
		}
		default:
			HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,1);	
			HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,1);
			HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,1);
			HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin,1);
	}
	if(cnt<3)cnt++;
	else cnt=0;
	
}



/**
 * @brief 一阶IIR低通滤波函数（浮点数版）
 * @param current 当前传感器采样值（已转换为物理量，如电压）
 * @param fc 截止频率（单位：Hz，需小于fs/2）
 * @param fs 采样频率（单位：Hz）
 * @return 滤波后的值
 */
float LowPassFilter(float current, float fc, float fs, float* prev_lowpass_output, float* highpass_output) {
//	// 全局变量：保存前一次滤波输出
//	static float prev_lowpass_output = 0.0f;
	
    // 计算滤波系数α（精确计算）
    float alpha = expf(-2.0f * 3.1416f * fc / fs);  // 精确公式
    // 或近似计算（当fc << fs时，如fc < fs/10）：
    // float alpha = (2.0f * 3.1416f * fc) / fs;

    // 滤波计算：当前输出 = α*当前输入 + (1-α)*前一次输出
    float output = alpha * current + (1.0f - alpha) * (*prev_lowpass_output);
	
	//同时输出高通滤波后的成分 取绝对值
	if(current - output >= 0)
		*highpass_output = current - output;
	else
		*highpass_output = output - current;

    // 更新前一次输出
    (*prev_lowpass_output) = output;

    return output;
}
/* USER CODE END Application */

