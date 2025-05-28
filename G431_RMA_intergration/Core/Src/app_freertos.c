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
	AD5206_SetResistance(1,3,5);//IN1-OUT11  2082Ω
	AD5206_SetResistance(1,1,5);//IN2-OUT11  2093Ω
	AD5206_SetResistance(1,0,5);//IN3-OUT11  2095Ω
	AD5206_SetResistance(1,2,5);//IN4-OUT11  2088Ω
	AD5206_SetResistance(1,5,5);//IN5-OUT11  2082Ω
	AD5206_SetResistance(0,5,5);//IN6-OUT11  1985Ω
	AD5206_SetResistance(0,3,5);//IN7-OUT11  1987Ω
	AD5206_SetResistance(0,1,5);//IN8-OUT11  1987Ω
	AD5206_SetResistance(0,0,5);//IN9-OUT11  1988Ω
	AD5206_SetResistance(0,2,5);//IN10-OUT11 1990Ω
	
	
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
void select_switcher(uint8_t index)
{
	switch(index)
	{
		case 0:
			IO10_GPIO_Port->BSRR=IO10_Pin;//高电平
			IO11_GPIO_Port->BSRR=(IO11_Pin << 16);//低电平
			break;
		case 1:
			IO11_GPIO_Port->BSRR=IO11_Pin;//高电平
			IO1_GPIO_Port->BSRR =(IO1_Pin << 16);//低电平
			break;
		case 2:
			IO1_GPIO_Port->BSRR=IO1_Pin;//高电平
			IO2_GPIO_Port->BSRR=(IO2_Pin << 16);//低电平
			break;
		case 3:
			IO2_GPIO_Port->BSRR=IO2_Pin;//高电平
			IO3_GPIO_Port->BSRR=(IO3_Pin << 16);//低电平
			break;
		case 4:
			IO3_GPIO_Port->BSRR=IO3_Pin;//高电平
			IO4_GPIO_Port->BSRR=(IO4_Pin << 16);//低电平
			break;
		case 5:
			IO4_GPIO_Port->BSRR=IO4_Pin;//高电平
			IO5_GPIO_Port->BSRR=(IO5_Pin << 16);//低电平
			break;
		case 6:
			IO5_GPIO_Port->BSRR=IO5_Pin;//高电平
			IO6_GPIO_Port->BSRR=(IO6_Pin << 16);//低电平
			break;
		case 7:
			IO6_GPIO_Port->BSRR=IO6_Pin;//高电平
			IO7_GPIO_Port->BSRR=(IO7_Pin << 16);//低电平
			break;
		case 8:
			IO7_GPIO_Port->BSRR=IO7_Pin;//高电平
			IO8_GPIO_Port->BSRR=(IO8_Pin << 16);//低电平
			break;
		case 9:
			IO8_GPIO_Port->BSRR=IO8_Pin;//高电平
			IO9_GPIO_Port->BSRR=(IO9_Pin << 16);//低电平
			break;
		case 10:
			IO9_GPIO_Port->BSRR =IO9_Pin;//高电平
			IO10_GPIO_Port->BSRR=(IO10_Pin << 16);//低电平
			break;
	}
}

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
    if(index==0)
		HAL_GPIO_WritePin(CS1_GPIO_Port,CS1_Pin,1);
	else if(index==1)
		HAL_GPIO_WritePin(CS2_GPIO_Port,CS2_Pin,1);
}
/* USER CODE END Application */

