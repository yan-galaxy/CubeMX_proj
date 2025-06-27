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
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 512);
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
uint16_t adc_value[10];
BMP280_Device bmp_dev;
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	char usb_buff[1024]={0};
	HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc_value, 10);
	/* 初始化BMP280 */
    bmp_dev.hspi = &hspi1;
    bmp_dev.cs_port = BMP280_SPI_CS_GPIO_Port;
    bmp_dev.cs_pin = BMP280_SPI_CS_Pin;
	
	while(BMP280_Init(&bmp_dev) != HAL_OK)
	{
		sprintf(usb_buff,"初始化失败\r\n");
		CDC_Transmit_FS((uint8_t *)usb_buff,strlen(usb_buff));
		osDelay(500);
	}
  /* Infinite loop */
	for(;;)
	{
		HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
		
		/* 读取压力和温度 */
        if (BMP280_ReadPressureTemperature(&bmp_dev) == HAL_OK) {
            sprintf(usb_buff,"温度: %.2f °C\r\n", bmp_dev.temperature);
            sprintf(usb_buff+strlen(usb_buff),"压力: %.2f kPa\r\n", bmp_dev.pressure);
            sprintf(usb_buff+strlen(usb_buff),"海拔: %.2f m\r\n", bmp_dev.altitude);
            sprintf(usb_buff+strlen(usb_buff),"------------------------\r\n");
        } else {
            sprintf(usb_buff,"读取数据失败\r\n");
        }
		
		
//		sprintf(usb_buff,"PB2 adc_value:%d\r\n",adc_value[0]);
//		sprintf(usb_buff,"BMP id:0x%X\r\n",bmp_dev.chip_id);
		CDC_Transmit_FS((uint8_t *)usb_buff,strlen(usb_buff));
		osDelay(500);
	}
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

