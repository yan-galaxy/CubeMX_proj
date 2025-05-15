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
osThreadId keyTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void LedTask(void const * argument);
void KeyTask(void const * argument);

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
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 256);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of ledTask */
  osThreadDef(ledTask, LedTask, osPriorityLow, 0, 128);
  ledTaskHandle = osThreadCreate(osThread(ledTask), NULL);

  /* definition and creation of keyTask */
  osThreadDef(keyTask, KeyTask, osPriorityRealtime, 0, 128);
  keyTaskHandle = osThreadCreate(osThread(keyTask), NULL);

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
#define KEY_ON 0
uint8_t key_flag;
uint16_t adc_value[3];
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	char usb_buff[256]={0};
	HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc_value, 3);
	
	uint16_t record_OUT1_IN1;
	uint16_t record_OUT1_IN2;
	uint16_t record_OUT1_IN3;
	uint16_t record_OUT2_IN1;
	uint16_t record_OUT2_IN2;
	uint16_t record_OUT2_IN3;
	uint16_t record_OUT3_IN1;
	uint16_t record_OUT3_IN2;
	uint16_t record_OUT3_IN3;
	uint16_t record_OUT4_IN1;
	uint16_t record_OUT4_IN2;
	uint16_t record_OUT4_IN3;
	
	uint16_t record_2_OUT1_IN1;
	uint16_t record_2_OUT1_IN2;
	uint16_t record_2_OUT1_IN3;
	uint16_t record_2_OUT2_IN1;
	uint16_t record_2_OUT2_IN2;
	uint16_t record_2_OUT2_IN3;
	uint16_t record_2_OUT3_IN1;
	uint16_t record_2_OUT3_IN2;
	uint16_t record_2_OUT3_IN3;
	
	double Rref1 = 1016;
	double Rref2 = 1005;
	double Rref3 = 1002;
	double R11,R12,R13,R21,R22,R23,R31,R32,R33;
	double R11_2,R12_2,R13_2,R21_2,R22_2,R23_2,R31_2,R32_2,R33_2;
	
	double voltage_IN1=0.0;
	double voltage_IN2=0.0;
	double voltage_IN3=0.0;
	osDelay(3000);
  /* Infinite loop */
	for(;;)
	{
		
		
		HAL_GPIO_WritePin(OUT1_GPIO_Port,OUT1_Pin,0);
		HAL_GPIO_WritePin(OUT2_GPIO_Port,OUT2_Pin,1);
		HAL_GPIO_WritePin(OUT3_GPIO_Port,OUT3_Pin,1);
		HAL_GPIO_WritePin(OUT4_GPIO_Port,OUT4_Pin,1);
		osDelay(1);
		for(uint16_t i=0;i<200;i++)
		{
			voltage_IN1+=adc_value[0];
			voltage_IN2+=adc_value[1];
		    voltage_IN3+=adc_value[2];
			osDelay(1);
		}
		voltage_IN1/=200.0;
		voltage_IN2/=200.0;
		voltage_IN3/=200.0;
//		sprintf(usb_buff,"OUT1=0\r\nPA0 adc_value:%4.3lf , PB11 adc_value:%4.3lf , PA4 adc_value:%4.3lf key_flag:%d\r\n",voltage_IN1,voltage_IN2,voltage_IN3,key_flag);
//		CDC_Transmit_FS((uint8_t *)usb_buff,strlen(usb_buff));
		record_OUT1_IN1 = voltage_IN1;
		record_OUT1_IN2 = voltage_IN2;
		record_OUT1_IN3 = voltage_IN3;
		voltage_IN1=0.0;
		voltage_IN2=0.0;
		voltage_IN3=0.0;
		if(KEY_ON)
			while(key_flag==0)
				osDelay(1);
		
		
		HAL_GPIO_WritePin(OUT1_GPIO_Port,OUT1_Pin,1);
		HAL_GPIO_WritePin(OUT2_GPIO_Port,OUT2_Pin,0);
		HAL_GPIO_WritePin(OUT3_GPIO_Port,OUT3_Pin,1);
		HAL_GPIO_WritePin(OUT4_GPIO_Port,OUT4_Pin,1);
		osDelay(1);
		for(uint16_t i=0;i<200;i++)
		{
			voltage_IN1+=adc_value[0];
			voltage_IN2+=adc_value[1];
		    voltage_IN3+=adc_value[2];
			osDelay(1);
		}
		voltage_IN1/=200.0;
		voltage_IN2/=200.0;
		voltage_IN3/=200.0;
//		sprintf(usb_buff,"OUT2=0\r\nPA0 adc_value:%4.3lf , PB11 adc_value:%4.3lf , PA4 adc_value:%4.3lf key_flag:%d\r\n",voltage_IN1,voltage_IN2,voltage_IN3,key_flag);
//		CDC_Transmit_FS((uint8_t *)usb_buff,strlen(usb_buff));
		record_OUT2_IN1 = voltage_IN1;
		record_OUT2_IN2 = voltage_IN2;
		record_OUT2_IN3 = voltage_IN3;
		voltage_IN1=0.0;
		voltage_IN2=0.0;
		voltage_IN3=0.0;
		if(KEY_ON)
			while(key_flag==1)
				osDelay(1);
		
		HAL_GPIO_WritePin(OUT1_GPIO_Port,OUT1_Pin,1);
		HAL_GPIO_WritePin(OUT2_GPIO_Port,OUT2_Pin,1);
		HAL_GPIO_WritePin(OUT3_GPIO_Port,OUT3_Pin,0);
		HAL_GPIO_WritePin(OUT4_GPIO_Port,OUT4_Pin,1);
		osDelay(1);
		for(uint16_t i=0;i<200;i++)
		{
			voltage_IN1+=adc_value[0];
			voltage_IN2+=adc_value[1];
		    voltage_IN3+=adc_value[2];
			osDelay(1);
		}
		voltage_IN1/=200.0;
		voltage_IN2/=200.0;
		voltage_IN3/=200.0;
//		sprintf(usb_buff,"OUT3=0\r\nPA0 adc_value:%4.3lf , PB11 adc_value:%4.3lf , PA4 adc_value:%4.3lf key_flag:%d\r\n",voltage_IN1,voltage_IN2,voltage_IN3,key_flag);
//		CDC_Transmit_FS((uint8_t *)usb_buff,strlen(usb_buff));
		record_OUT3_IN1 = voltage_IN1;
		record_OUT3_IN2 = voltage_IN2;
		record_OUT3_IN3 = voltage_IN3;
		voltage_IN1=0.0;
		voltage_IN2=0.0;
		voltage_IN3=0.0;
		if(KEY_ON)
			while(key_flag==2)
				osDelay(1);
		
		HAL_GPIO_WritePin(OUT1_GPIO_Port,OUT1_Pin,1);
		HAL_GPIO_WritePin(OUT2_GPIO_Port,OUT2_Pin,1);
		HAL_GPIO_WritePin(OUT3_GPIO_Port,OUT3_Pin,1);
		HAL_GPIO_WritePin(OUT4_GPIO_Port,OUT4_Pin,0);
		osDelay(1);
		for(uint16_t i=0;i<200;i++)
		{
			voltage_IN1+=adc_value[0];
			voltage_IN2+=adc_value[1];
		    voltage_IN3+=adc_value[2];
			osDelay(1);
		}
		voltage_IN1/=200.0;
		voltage_IN2/=200.0;
		voltage_IN3/=200.0;
//		sprintf(usb_buff,"OUT4=0\r\nPA0 adc_value:%4.3lf , PB11 adc_value:%4.3lf , PA4 adc_value:%4.3lf key_flag:%d\r\n",voltage_IN1,voltage_IN2,voltage_IN3,key_flag);
//		CDC_Transmit_FS((uint8_t *)usb_buff,strlen(usb_buff));
		record_OUT4_IN1 = voltage_IN1;
		record_OUT4_IN2 = voltage_IN2;
		record_OUT4_IN3 = voltage_IN3;
		voltage_IN1=0.0;
		voltage_IN2=0.0;
		voltage_IN3=0.0;
		if(KEY_ON)
			while(key_flag==3)
				osDelay(1);
			
		
		HAL_GPIO_WritePin(OUT1_GPIO_Port,OUT1_Pin,0);
		HAL_GPIO_WritePin(OUT2_GPIO_Port,OUT2_Pin,1);
		HAL_GPIO_WritePin(OUT3_GPIO_Port,OUT3_Pin,1);
		HAL_GPIO_WritePin(OUT4_GPIO_Port,OUT4_Pin,0);
		osDelay(1);
		for(uint16_t i=0;i<200;i++)
		{
			voltage_IN1+=adc_value[0];
			voltage_IN2+=adc_value[1];
		    voltage_IN3+=adc_value[2];
			osDelay(1);
		}
		voltage_IN1/=200.0;
		voltage_IN2/=200.0;
		voltage_IN3/=200.0;
		record_2_OUT1_IN1 = voltage_IN1;
		record_2_OUT1_IN2 = voltage_IN2;
		record_2_OUT1_IN3 = voltage_IN3;
		voltage_IN1=0.0;
		voltage_IN2=0.0;
		voltage_IN3=0.0;
		if(KEY_ON)
			while(key_flag==0)
				osDelay(1);
			
		HAL_GPIO_WritePin(OUT1_GPIO_Port,OUT1_Pin,1);
		HAL_GPIO_WritePin(OUT2_GPIO_Port,OUT2_Pin,0);
		HAL_GPIO_WritePin(OUT3_GPIO_Port,OUT3_Pin,1);
		HAL_GPIO_WritePin(OUT4_GPIO_Port,OUT4_Pin,0);
		osDelay(1);
		for(uint16_t i=0;i<200;i++)
		{
			voltage_IN1+=adc_value[0];
			voltage_IN2+=adc_value[1];
		    voltage_IN3+=adc_value[2];
			osDelay(1);
		}
		voltage_IN1/=200.0;
		voltage_IN2/=200.0;
		voltage_IN3/=200.0;
		record_2_OUT2_IN1 = voltage_IN1;
		record_2_OUT2_IN2 = voltage_IN2;
		record_2_OUT2_IN3 = voltage_IN3;
		voltage_IN1=0.0;
		voltage_IN2=0.0;
		voltage_IN3=0.0;
		if(KEY_ON)
			while(key_flag==1)
				osDelay(1);
			
			
		HAL_GPIO_WritePin(OUT1_GPIO_Port,OUT1_Pin,1);
		HAL_GPIO_WritePin(OUT2_GPIO_Port,OUT2_Pin,1);
		HAL_GPIO_WritePin(OUT3_GPIO_Port,OUT3_Pin,0);
		HAL_GPIO_WritePin(OUT4_GPIO_Port,OUT4_Pin,0);
		osDelay(1);
		for(uint16_t i=0;i<200;i++)
		{
			voltage_IN1+=adc_value[0];
			voltage_IN2+=adc_value[1];
		    voltage_IN3+=adc_value[2];
			osDelay(1);
		}
		voltage_IN1/=200.0;
		voltage_IN2/=200.0;
		voltage_IN3/=200.0;
		record_2_OUT3_IN1 = voltage_IN1;
		record_2_OUT3_IN2 = voltage_IN2;
		record_2_OUT3_IN3 = voltage_IN3;
		voltage_IN1=0.0;
		voltage_IN2=0.0;
		voltage_IN3=0.0;
		if(KEY_ON)
			while(key_flag==2)
				osDelay(1);
		
		key_flag=0;
		
		
		R31 = (double)( 4095 - record_OUT4_IN1 ) / (double)( 4095 - record_OUT3_IN1 ) * Rref1;
		R32 = (double)( 4095 - record_OUT4_IN2 ) / (double)( 4095 - record_OUT3_IN2 ) * Rref2;
		R33 = (double)( 4095 - record_OUT4_IN3 ) / (double)( 4095 - record_OUT3_IN3 ) * Rref3;
		
		R21 = (double)( 4095 - record_OUT4_IN1 ) / (double)( 4095 - record_OUT2_IN1 ) * Rref1;
		R22 = (double)( 4095 - record_OUT4_IN2 ) / (double)( 4095 - record_OUT2_IN2 ) * Rref2;
		R23 = (double)( 4095 - record_OUT4_IN3 ) / (double)( 4095 - record_OUT2_IN3 ) * Rref3;
		
		R11 = (double)( 4095 - record_OUT4_IN1 ) / (double)( 4095 - record_OUT1_IN1 ) * Rref1;
		R12 = (double)( 4095 - record_OUT4_IN2 ) / (double)( 4095 - record_OUT1_IN2 ) * Rref2;
		R13 = (double)( 4095 - record_OUT4_IN3 ) / (double)( 4095 - record_OUT1_IN3 ) * Rref3;
		
		sprintf(usb_buff,"\r\n\r\nR31:%4.3lf, R32:%4.3lf, R33:%4.3lf\r\nR21:%4.3lf, R22:%4.3lf, R23:%4.3lf\r\nR11:%4.3lf, R12:%4.3lf, R13:%4.3lf\r\n",R31,R32,R33,R21,R22,R23,R11,R12,R13);
		CDC_Transmit_FS((uint8_t *)usb_buff,strlen(usb_buff));
			
		osDelay(5);
		//INCRMA·½·¨
		R31_2 = (double)( record_OUT3_IN1 - record_2_OUT3_IN1 ) / (double)( record_OUT4_IN1 - record_2_OUT3_IN1 ) * Rref1;
		R32_2 = (double)( record_OUT3_IN2 - record_2_OUT3_IN2 ) / (double)( record_OUT4_IN2 - record_2_OUT3_IN2 ) * Rref2;
		R33_2 = (double)( record_OUT3_IN3 - record_2_OUT3_IN3 ) / (double)( record_OUT4_IN3 - record_2_OUT3_IN3 ) * Rref3;
												   
		R21_2 = (double)( record_OUT2_IN1 - record_2_OUT2_IN1 ) / (double)( record_OUT4_IN1 - record_2_OUT2_IN1 ) * Rref1;
		R22_2 = (double)( record_OUT2_IN2 - record_2_OUT2_IN2 ) / (double)( record_OUT4_IN2 - record_2_OUT2_IN2 ) * Rref2;
		R23_2 = (double)( record_OUT2_IN3 - record_2_OUT2_IN3 ) / (double)( record_OUT4_IN3 - record_2_OUT2_IN3 ) * Rref3;
												   
		R11_2 = (double)( record_OUT1_IN1 - record_2_OUT1_IN1 ) / (double)( record_OUT4_IN1 - record_2_OUT1_IN1 ) * Rref1;
		R12_2 = (double)( record_OUT1_IN2 - record_2_OUT1_IN2 ) / (double)( record_OUT4_IN2 - record_2_OUT1_IN2 ) * Rref2;
		R13_2 = (double)( record_OUT1_IN3 - record_2_OUT1_IN3 ) / (double)( record_OUT4_IN3 - record_2_OUT1_IN3 ) * Rref3;
		
		sprintf(usb_buff,"\r\n\r\nR31_2:%4.3lf, R32_2:%4.3lf, R33_2:%4.3lf\r\nR21_2:%4.3lf, R22_2:%4.3lf, R23_2:%4.3lf\r\nR11_2:%4.3lf, R12_2:%4.3lf, R13_2:%4.3lf\r\n",R31_2,R32_2,R33_2,R21_2,R22_2,R23_2,R11_2,R12_2,R13_2);
		CDC_Transmit_FS((uint8_t *)usb_buff,strlen(usb_buff));
//		while(1)
//		{
//			osDelay(500);
//		}
		
		
//		sprintf(usb_buff,"PA0 adc_value:%4d , PB11 adc_value:%d , PA4 adc_value:%d\r\n",adc_value[0],adc_value[1],adc_value[2]);

//		for(uint16_t i=0;i<200;i++)
//		{
//			voltage_IN1+=adc_value[0];
//			voltage_IN2+=adc_value[1];
//		    voltage_IN3+=adc_value[2];
//			osDelay(1);
//		}
//		voltage_IN1/=200.0;
//		voltage_IN2/=200.0;
//		voltage_IN3/=200.0;
//		sprintf(usb_buff,"PA0 adc_value:%4.3lf , PB11 adc_value:%4.3lf , PA4 adc_value:%4.3lf key_flag:%d\r\n",voltage_IN1,voltage_IN2,voltage_IN3,key_flag);
//		CDC_Transmit_FS((uint8_t *)usb_buff,strlen(usb_buff));
//		
//		voltage_IN1=0.0;
//		voltage_IN2=0.0;
//		voltage_IN3=0.0;
		
		osDelay(100);
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
  /* Infinite loop */
	for(;;)
	{
		HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
		osDelay(100);
	}
  /* USER CODE END LedTask */
}

/* USER CODE BEGIN Header_KeyTask */
/**
* @brief Function implementing the keyTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_KeyTask */
void KeyTask(void const * argument)
{
  /* USER CODE BEGIN KeyTask */
  /* Infinite loop */
	for(;;)
	{
		if(HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin))
		{
			osDelay(20);
			while(HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin))
			{
				osDelay(1);
			}
			osDelay(20);
			
//			HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
			
			if(key_flag<3)
				key_flag++;
			else
				key_flag=0;

		}
		osDelay(1);
	}
  /* USER CODE END KeyTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

