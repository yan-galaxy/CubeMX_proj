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

uint16_t samp_data[11][10];
uint16_t samp_data_INC[11][10];
volatile Word_union res_data_1[2104];
volatile Word_union res_data_2[2104];
volatile Word_union* res_send_p=res_data_1;
volatile Word_union* res_store_p=res_data_2;

#define SHOW_USB_DATA 0U
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	char usb_TxBuf[3048]={0};
	
	

	
	static unsigned long long index = 1;          // 数据帧索引
	static double res_ref_kOm1  = 2.082;  //2082Ω 2.082   51.1
	static double res_ref_kOm2  = 2.093;  //2093Ω 2.093   51.2
	static double res_ref_kOm3  = 2.095;  //2095Ω 2.095   51.2
	static double res_ref_kOm4  = 2.088;  //2088Ω 2.088   51.1
	static double res_ref_kOm5  = 2.082;  //2082Ω 2.082   51.1
	static double res_ref_kOm6  = 1.985;  //1985Ω 1.985   49.1
	static double res_ref_kOm7  = 1.987;  //1987Ω 1.987   48.7
	static double res_ref_kOm8  = 1.987;  //1987Ω 1.987   49.0
	static double res_ref_kOm9  = 1.988;  //1988Ω 1.988   48.8
	static double res_ref_kOm10 = 1.990;  //1990Ω 1.990   48.9
	
  /* Infinite loop */
	for(;;)
	{
		HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
		
//		sprintf(usb_TxBuf,"IN1:%d ,IN2:%d ,IN3:%d ,IN4:%d ,IN5:%d ,IN6:%d ,IN7:%d ,IN8:%d ,IN9:%d ,IN10:%d\r\n",
//			adc_value_struct.IN1,adc_value_struct.IN2,adc_value_struct.IN3,adc_value_struct.IN4,adc_value_struct.IN5,
//			adc_value_struct.IN6,adc_value_struct.IN7,adc_value_struct.IN8,adc_value_struct.IN9,adc_value_struct.IN10);
//		CDC_Transmit_FS((uint8_t *)usb_TxBuf,strlen(usb_TxBuf));
		
#if SHOW_USB_DATA == 1U
		if(osSemaphoreWait(dataBinarySemHandle, 0) == osOK)
		{
			// 格式化数据帧（示例：每行11通道，共10组）
			sprintf((char *)usb_TxBuf, 
					"Frame %02llu: \r\n", 
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
					"value: (kOm)\r\nRMA\r\n");
			/* 普通RMA */
			for (uint8_t i = 0; i < 11; i++) {
				sprintf((char *)usb_TxBuf + strlen((char *)usb_TxBuf), 
					"line:%2d:%2.3f  %2.3f  %2.3f  %2.3f  %2.3f  %2.3f  %2.3f  %2.3f  %2.3f  %2.3f \r\n", //ADC多通道串扰体现在行串扰(单个line中)，RMA电压注入不理想体现在列串扰(不同line之间)
						i+1,
						(4095-samp_data[0][0])/(double)(4095-samp_data[i][0])*res_ref_kOm1, (4095-samp_data[0][1])/(double)(4095-samp_data[i][1])*res_ref_kOm2, (4095-samp_data[0][2])/(double)(4095-samp_data[i][2])*res_ref_kOm3, 
						(4095-samp_data[0][3])/(double)(4095-samp_data[i][3])*res_ref_kOm4, (4095-samp_data[0][4])/(double)(4095-samp_data[i][4])*res_ref_kOm5, (4095-samp_data[0][5])/(double)(4095-samp_data[i][5])*res_ref_kOm6, 
						(4095-samp_data[0][6])/(double)(4095-samp_data[i][6])*res_ref_kOm7, (4095-samp_data[0][7])/(double)(4095-samp_data[i][7])*res_ref_kOm8, (4095-samp_data[0][8])/(double)(4095-samp_data[i][8])*res_ref_kOm9, 
						(4095-samp_data[0][9])/(double)(4095-samp_data[i][9])*res_ref_kOm10);
			}
			/* INCRMA */
			sprintf((char *)usb_TxBuf + strlen((char *)usb_TxBuf),"INCRMA\r\n");
			for (uint8_t i = 1; i < 11; i++) {
				sprintf((char *)usb_TxBuf + strlen((char *)usb_TxBuf), 
					"line:%2d:%02.3f  %2.3f  %2.3f  %2.3f  %2.3f  %2.3f  %2.3f  %2.3f  %2.3f  %2.3f \r\n", 
						i,
						(samp_data[i][0]-samp_data_INC[i][0])/(double)(samp_data[0][0]-samp_data_INC[i][0])*res_ref_kOm1,
						(samp_data[i][1]-samp_data_INC[i][1])/(double)(samp_data[0][1]-samp_data_INC[i][1])*res_ref_kOm2,
						(samp_data[i][2]-samp_data_INC[i][2])/(double)(samp_data[0][2]-samp_data_INC[i][2])*res_ref_kOm3,
						(samp_data[i][3]-samp_data_INC[i][3])/(double)(samp_data[0][3]-samp_data_INC[i][3])*res_ref_kOm4,
						(samp_data[i][4]-samp_data_INC[i][4])/(double)(samp_data[0][4]-samp_data_INC[i][4])*res_ref_kOm5,
						(samp_data[i][5]-samp_data_INC[i][5])/(double)(samp_data[0][5]-samp_data_INC[i][5])*res_ref_kOm6,
						(samp_data[i][6]-samp_data_INC[i][6])/(double)(samp_data[0][6]-samp_data_INC[i][6])*res_ref_kOm7,
						(samp_data[i][7]-samp_data_INC[i][7])/(double)(samp_data[0][7]-samp_data_INC[i][7])*res_ref_kOm8,
						(samp_data[i][8]-samp_data_INC[i][8])/(double)(samp_data[0][8]-samp_data_INC[i][8])*res_ref_kOm9,
						(samp_data[i][9]-samp_data_INC[i][9])/(double)(samp_data[0][9]-samp_data_INC[i][9])*res_ref_kOm10
						);
			}
			
			
			
			CDC_Transmit_FS((uint8_t *)usb_TxBuf, strlen((char *)usb_TxBuf));//USB CDC
			
			osSemaphoreRelease(dataBinarySemHandle);
		}
#endif
		osDelay(500);
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
	TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(1);  // 1秒周期[6](@ref)
	
	osSemaphoreWait(dataBinarySemHandle, osWaitForever);
		
	select_switcher_all_high();
	AD5206_SetResistance(1,3,5);//IN1-OUT11  5:2082Ω  12:4.92kΩ  25:10.20kΩ  125:51.1kΩ
	AD5206_SetResistance(1,1,5);//IN2-OUT11  5:2093Ω  12:4.95kΩ  25:10.29kΩ  125:51.2kΩ
	AD5206_SetResistance(1,0,5);//IN3-OUT11  5:2095Ω  12:4.95kΩ  25:10.32kΩ  125:51.2kΩ
	AD5206_SetResistance(1,2,5);//IN4-OUT11  5:2088Ω  12:4.94kΩ  25:10.35kΩ  125:51.1kΩ
	AD5206_SetResistance(1,5,5);//IN5-OUT11  5:2082Ω  12:4.94kΩ  25:10.25kΩ  125:51.1kΩ
	AD5206_SetResistance(0,5,5);//IN6-OUT11  5:1985Ω  12:4.73kΩ  25:9.82kΩ   125:49.1kΩ
	AD5206_SetResistance(0,3,5);//IN7-OUT11  5:1987Ω  12:4.74kΩ  25:9.74kΩ   125:48.7kΩ
	AD5206_SetResistance(0,1,5);//IN8-OUT11  5:1987Ω  12:4.72kΩ  25:9.81kΩ   125:49.0kΩ
	AD5206_SetResistance(0,0,5);//IN9-OUT11  5:1988Ω  12:4.73kΩ  25:9.77kΩ   125:48.8kΩ
	AD5206_SetResistance(0,2,5);//IN10-OUT11 5:1990Ω  12:4.75kΩ  25:9.78kΩ   125:48.9kΩ
	
	//  1/42.5MHz*(640.5+12.5=653)cycles *5ch = 76.8235us    ADC转换时配置为 640.5cycles
	//  1/42.5MHz*(247.5+12.5=260)cycles *5ch = 30.5882us    ADC转换时配置为 247.5cycles
	//  1/42.5MHz*(92.5+12.5=105 )cycles *5ch = 12.3530us    ADC转换时配置为 92.5cycles
	
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)(&(adc_value_struct.IN2)), 5);
	HAL_ADC_Start_DMA(&hadc2, (uint32_t *)(&(adc_value_struct.IN1)), 5);
	
	osSemaphoreRelease(dataBinarySemHandle);
	
	
	res_data_1[0].byte[0]=0x55;//帧头
	res_data_1[0].byte[1]=0xAA;
	res_data_1[1].byte[0]=0xBB;
	res_data_1[1].byte[1]=0xCC;
	for(uint16_t i=0;i<2100;i++)
	{
//		res_send_p[2+i].word16=i*1+1;
		res_data_1[2+i].word16=i*1+1;
	}
	res_data_1[2102].byte[0]=0xAA;//帧尾
	res_data_1[2102].byte[1]=0x55;
	res_data_1[2103].byte[0]=0x66;
	res_data_1[2103].byte[1]=0x77;
	
	res_data_2[0].byte[0]=0x55;//帧头
	res_data_2[0].byte[1]=0xAA;
	res_data_2[1].byte[0]=0xBB;
	res_data_2[1].byte[1]=0xCC;
	for(uint16_t i=0;i<2100;i++)
	{
		res_data_2[2+i].word16=i*2+2;
	}
	res_data_2[2102].byte[0]=0xAA;//帧尾
	res_data_2[2102].byte[1]=0x55;
	res_data_2[2103].byte[0]=0x66;
	res_data_2[2103].byte[1]=0x77;
	
	
	const uint16_t delay_time_us = 30;//采样等待时间
	xLastWakeTime = xTaskGetTickCount();  // 初始化基准时间[2](@ref)
  /* Infinite loop */
	for(;;)
	{
		for(uint8_t j=0;j<10;j++)
		{
			if(osSemaphoreWait(dataBinarySemHandle, 0) == osOK)//		osSemaphoreWait(dataBinarySemHandle, osWaitForever);
			{
				for(uint8_t i=0;i<21;i++)
				{
					select_switcher2(i);
					user_delaynus_tim(delay_time_us);
					if(i==0)//普通RMA部分
					{
						//+2是为了跳过帧头
						res_store_p[j*210+i*10+0+2].word16=adc_value_struct.IN1;
						res_store_p[j*210+i*10+1+2].word16=adc_value_struct.IN2;
						res_store_p[j*210+i*10+2+2].word16=adc_value_struct.IN3;
						res_store_p[j*210+i*10+3+2].word16=adc_value_struct.IN4;
						res_store_p[j*210+i*10+4+2].word16=adc_value_struct.IN5;
						res_store_p[j*210+i*10+5+2].word16=adc_value_struct.IN6;
						res_store_p[j*210+i*10+6+2].word16=adc_value_struct.IN7;
						res_store_p[j*210+i*10+7+2].word16=adc_value_struct.IN8;
						res_store_p[j*210+i*10+8+2].word16=adc_value_struct.IN9;
						res_store_p[j*210+i*10+9+2].word16=adc_value_struct.IN10;
						#if SHOW_USB_DATA == 1U
							samp_data[i][0]=res_store_p[j*210+i*10+0+2].word16;
							samp_data[i][1]=res_store_p[j*210+i*10+1+2].word16;
							samp_data[i][2]=res_store_p[j*210+i*10+2+2].word16;
							samp_data[i][3]=res_store_p[j*210+i*10+3+2].word16;
							samp_data[i][4]=res_store_p[j*210+i*10+4+2].word16;
							samp_data[i][5]=res_store_p[j*210+i*10+5+2].word16;
							samp_data[i][6]=res_store_p[j*210+i*10+6+2].word16;
							samp_data[i][7]=res_store_p[j*210+i*10+7+2].word16;
							samp_data[i][8]=res_store_p[j*210+i*10+8+2].word16;
							samp_data[i][9]=res_store_p[j*210+i*10+9+2].word16;
						#endif
					}
					else if(i%2==1)//普通RMA部分
					{
						//+2是为了跳过帧头
						res_store_p[j*210+((i+1)/2)*10+0+2].word16=adc_value_struct.IN1;
						res_store_p[j*210+((i+1)/2)*10+1+2].word16=adc_value_struct.IN2;
						res_store_p[j*210+((i+1)/2)*10+2+2].word16=adc_value_struct.IN3;
						res_store_p[j*210+((i+1)/2)*10+3+2].word16=adc_value_struct.IN4;
						res_store_p[j*210+((i+1)/2)*10+4+2].word16=adc_value_struct.IN5;
						res_store_p[j*210+((i+1)/2)*10+5+2].word16=adc_value_struct.IN6;
						res_store_p[j*210+((i+1)/2)*10+6+2].word16=adc_value_struct.IN7;
						res_store_p[j*210+((i+1)/2)*10+7+2].word16=adc_value_struct.IN8;
						res_store_p[j*210+((i+1)/2)*10+8+2].word16=adc_value_struct.IN9;
						res_store_p[j*210+((i+1)/2)*10+9+2].word16=adc_value_struct.IN10;
						#if SHOW_USB_DATA == 1U
							samp_data[((i+1)/2)][0]=res_store_p[j*210+((i+1)/2)*10+0+2].word16;
							samp_data[((i+1)/2)][1]=res_store_p[j*210+((i+1)/2)*10+1+2].word16;
							samp_data[((i+1)/2)][2]=res_store_p[j*210+((i+1)/2)*10+2+2].word16;
							samp_data[((i+1)/2)][3]=res_store_p[j*210+((i+1)/2)*10+3+2].word16;
							samp_data[((i+1)/2)][4]=res_store_p[j*210+((i+1)/2)*10+4+2].word16;
							samp_data[((i+1)/2)][5]=res_store_p[j*210+((i+1)/2)*10+5+2].word16;
							samp_data[((i+1)/2)][6]=res_store_p[j*210+((i+1)/2)*10+6+2].word16;
							samp_data[((i+1)/2)][7]=res_store_p[j*210+((i+1)/2)*10+7+2].word16;
							samp_data[((i+1)/2)][8]=res_store_p[j*210+((i+1)/2)*10+8+2].word16;
							samp_data[((i+1)/2)][9]=res_store_p[j*210+((i+1)/2)*10+9+2].word16;
						#endif
					}
					else//INCRMA部分
					{
						//+2是为了跳过帧头
						res_store_p[j*210+(i/2+10)*10+0+2].word16=adc_value_struct.IN1;
						res_store_p[j*210+(i/2+10)*10+1+2].word16=adc_value_struct.IN2;
						res_store_p[j*210+(i/2+10)*10+2+2].word16=adc_value_struct.IN3;
						res_store_p[j*210+(i/2+10)*10+3+2].word16=adc_value_struct.IN4;
						res_store_p[j*210+(i/2+10)*10+4+2].word16=adc_value_struct.IN5;
						res_store_p[j*210+(i/2+10)*10+5+2].word16=adc_value_struct.IN6;
						res_store_p[j*210+(i/2+10)*10+6+2].word16=adc_value_struct.IN7;
						res_store_p[j*210+(i/2+10)*10+7+2].word16=adc_value_struct.IN8;
						res_store_p[j*210+(i/2+10)*10+8+2].word16=adc_value_struct.IN9;
						res_store_p[j*210+(i/2+10)*10+9+2].word16=adc_value_struct.IN10;
						#if SHOW_USB_DATA == 1U
							samp_data_INC[i/2][0]=res_store_p[j*210+(i/2+10)*10+0+2].word16;
							samp_data_INC[i/2][1]=res_store_p[j*210+(i/2+10)*10+1+2].word16;
							samp_data_INC[i/2][2]=res_store_p[j*210+(i/2+10)*10+2+2].word16;
							samp_data_INC[i/2][3]=res_store_p[j*210+(i/2+10)*10+3+2].word16;
							samp_data_INC[i/2][4]=res_store_p[j*210+(i/2+10)*10+4+2].word16;
							samp_data_INC[i/2][5]=res_store_p[j*210+(i/2+10)*10+5+2].word16;
							samp_data_INC[i/2][6]=res_store_p[j*210+(i/2+10)*10+6+2].word16;
							samp_data_INC[i/2][7]=res_store_p[j*210+(i/2+10)*10+7+2].word16;
							samp_data_INC[i/2][8]=res_store_p[j*210+(i/2+10)*10+8+2].word16;
							samp_data_INC[i/2][9]=res_store_p[j*210+(i/2+10)*10+9+2].word16;
						#endif
					}
				}


//				/* 普通RMA */
//				for(uint8_t i=0;i<11;i++)
//				{
//					select_switcher3(i);
//					user_delaynus_tim(delay_time_us);					
//					//+2是为了跳过帧头
//					res_store_p[j*210+i*10+0+2].word16=adc_value_struct.IN1;
//					res_store_p[j*210+i*10+1+2].word16=adc_value_struct.IN2;
//					res_store_p[j*210+i*10+2+2].word16=adc_value_struct.IN3;
//					res_store_p[j*210+i*10+3+2].word16=adc_value_struct.IN4;
//					res_store_p[j*210+i*10+4+2].word16=adc_value_struct.IN5;
//					res_store_p[j*210+i*10+5+2].word16=adc_value_struct.IN6;
//					res_store_p[j*210+i*10+6+2].word16=adc_value_struct.IN7;
//					res_store_p[j*210+i*10+7+2].word16=adc_value_struct.IN8;
//					res_store_p[j*210+i*10+8+2].word16=adc_value_struct.IN9;
//					res_store_p[j*210+i*10+9+2].word16=adc_value_struct.IN10;
//					#if SHOW_USB_DATA == 1U
//						samp_data[i][0]=res_store_p[j*210+i*10+0+2].word16;
//						samp_data[i][1]=res_store_p[j*210+i*10+1+2].word16;
//						samp_data[i][2]=res_store_p[j*210+i*10+2+2].word16;
//						samp_data[i][3]=res_store_p[j*210+i*10+3+2].word16;
//						samp_data[i][4]=res_store_p[j*210+i*10+4+2].word16;
//						samp_data[i][5]=res_store_p[j*210+i*10+5+2].word16;
//						samp_data[i][6]=res_store_p[j*210+i*10+6+2].word16;
//						samp_data[i][7]=res_store_p[j*210+i*10+7+2].word16;
//						samp_data[i][8]=res_store_p[j*210+i*10+8+2].word16;
//						samp_data[i][9]=res_store_p[j*210+i*10+9+2].word16;
//					#endif
//				}
//				/* INCRMA */
//				for(uint8_t i=11;i<21;i++)
//				{
//					select_switcher3(i);
//					user_delaynus_tim(delay_time_us);
//					//+2是为了跳过帧头
//					res_store_p[j*210+i*10+0+2].word16=adc_value_struct.IN1;
//					res_store_p[j*210+i*10+1+2].word16=adc_value_struct.IN2;
//					res_store_p[j*210+i*10+2+2].word16=adc_value_struct.IN3;
//					res_store_p[j*210+i*10+3+2].word16=adc_value_struct.IN4;
//					res_store_p[j*210+i*10+4+2].word16=adc_value_struct.IN5;
//					res_store_p[j*210+i*10+5+2].word16=adc_value_struct.IN6;
//					res_store_p[j*210+i*10+6+2].word16=adc_value_struct.IN7;
//					res_store_p[j*210+i*10+7+2].word16=adc_value_struct.IN8;
//					res_store_p[j*210+i*10+8+2].word16=adc_value_struct.IN9;
//					res_store_p[j*210+i*10+9+2].word16=adc_value_struct.IN10;
//					#if SHOW_USB_DATA == 1U
//						samp_data_INC[i-10][0]=res_store_p[j*210+i*10+0+2].word16;
//						samp_data_INC[i-10][1]=res_store_p[j*210+i*10+1+2].word16;
//						samp_data_INC[i-10][2]=res_store_p[j*210+i*10+2+2].word16;
//						samp_data_INC[i-10][3]=res_store_p[j*210+i*10+3+2].word16;
//						samp_data_INC[i-10][4]=res_store_p[j*210+i*10+4+2].word16;
//						samp_data_INC[i-10][5]=res_store_p[j*210+i*10+5+2].word16;
//						samp_data_INC[i-10][6]=res_store_p[j*210+i*10+6+2].word16;
//						samp_data_INC[i-10][7]=res_store_p[j*210+i*10+7+2].word16;
//						samp_data_INC[i-10][8]=res_store_p[j*210+i*10+8+2].word16;
//						samp_data_INC[i-10][9]=res_store_p[j*210+i*10+9+2].word16;
//					#endif
//				}
				IO11_GPIO_Port->BSRR=IO11_Pin;//高电平
				IO10_GPIO_Port->BSRR=IO10_Pin;//高电平
				osSemaphoreRelease(dataBinarySemHandle);
			}
			vTaskDelayUntil(&xLastWakeTime, xFrequency);//		osDelay(1);  vTaskSuspend(NULL);          // 挂起当前任务（自身）
		}
		#if SHOW_USB_DATA == 0U
			exchange_res_p();//切换缓存区
			CDC_Transmit_FS(res_send_p->byte, 4208);//2008
		#endif
		
		
//		osDelay(1000);
	}
  /* USER CODE END SampleTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
//电压采集完成之后需要进行缓存切换
void exchange_res_p(void)
{
	static uint8_t state=1;//第一次运行该函数前 采集的数据存到res_data_2   第一次运行该函数后状态变为0，发送的指针指向res_data_1  之后循环往复
	state=!state;
	
	if(state)
	{
		res_send_p=res_data_1;
		res_store_p=res_data_2;
	}
	else
	{
		res_send_p=res_data_2;
		res_store_p=res_data_1;
	}
}

void user_delaynus_tim(uint16_t nus)
{
	
	LL_TIM_SetCounter(TIM7, 0);
	while (LL_TIM_GetCounter(TIM7) < nus)
	{
		;// Optionally, add a timeout condition here to avoid an infinite loop
	}
}
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
void select_switcher2(uint8_t index)
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
		case 3:
			IO11_GPIO_Port->BSRR=IO11_Pin;//高电平
			IO1_GPIO_Port->BSRR=IO1_Pin;//高电平
			IO2_GPIO_Port->BSRR=(IO2_Pin << 16);//低电平
			break;
		case 5:
			IO11_GPIO_Port->BSRR=IO11_Pin;//高电平
			IO2_GPIO_Port->BSRR=IO2_Pin;//高电平
			IO3_GPIO_Port->BSRR=(IO3_Pin << 16);//低电平
			break;
		case 7:
			IO11_GPIO_Port->BSRR=IO11_Pin;//高电平
			IO3_GPIO_Port->BSRR=IO3_Pin;//高电平
			IO4_GPIO_Port->BSRR=(IO4_Pin << 16);//低电平
			break;
		case 9:
			IO11_GPIO_Port->BSRR=IO11_Pin;//高电平
			IO4_GPIO_Port->BSRR=IO4_Pin;//高电平
			IO5_GPIO_Port->BSRR=(IO5_Pin << 16);//低电平
			break;
		case 11:
			IO11_GPIO_Port->BSRR=IO11_Pin;//高电平
			IO5_GPIO_Port->BSRR=IO5_Pin;//高电平
			IO6_GPIO_Port->BSRR=(IO6_Pin << 16);//低电平
			break;
		case 13:
			IO11_GPIO_Port->BSRR=IO11_Pin;//高电平
			IO6_GPIO_Port->BSRR=IO6_Pin;//高电平
			IO7_GPIO_Port->BSRR=(IO7_Pin << 16);//低电平
			break;
		case 15:
			IO11_GPIO_Port->BSRR=IO11_Pin;//高电平
			IO7_GPIO_Port->BSRR=IO7_Pin;//高电平
			IO8_GPIO_Port->BSRR=(IO8_Pin << 16);//低电平
			break;
		case 17:
			IO11_GPIO_Port->BSRR=IO11_Pin;//高电平
			IO8_GPIO_Port->BSRR=IO8_Pin;//高电平
			IO9_GPIO_Port->BSRR=(IO9_Pin << 16);//低电平
			break;
		case 19:
			IO11_GPIO_Port->BSRR=IO11_Pin;//高电平
			IO9_GPIO_Port->BSRR =IO9_Pin;//高电平
			IO10_GPIO_Port->BSRR=(IO10_Pin << 16);//低电平
			break;
		case 2:case 4:case 6:case 8:case 10:case 12:case 14:case 16:case 18:case 20:
			IO11_GPIO_Port->BSRR =(IO11_Pin << 16);//低电平
			break;
	}
}
void select_switcher3(uint8_t index)
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
		
		
		//INCRMA 额外采样
		case 11:
			IO11_GPIO_Port->BSRR =(IO11_Pin << 16);//低电平
		
			IO10_GPIO_Port->BSRR =IO10_Pin;//高电平
			IO1_GPIO_Port->BSRR =(IO1_Pin << 16);//低电平
			break;
		case 12:
			IO1_GPIO_Port->BSRR=IO1_Pin;//高电平
			IO2_GPIO_Port->BSRR=(IO2_Pin << 16);//低电平
			break;
		case 13:
			IO2_GPIO_Port->BSRR=IO2_Pin;//高电平
			IO3_GPIO_Port->BSRR=(IO3_Pin << 16);//低电平
			break;
		case 14:
			IO3_GPIO_Port->BSRR=IO3_Pin;//高电平
			IO4_GPIO_Port->BSRR=(IO4_Pin << 16);//低电平
			break;
		case 15:
			IO4_GPIO_Port->BSRR=IO4_Pin;//高电平
			IO5_GPIO_Port->BSRR=(IO5_Pin << 16);//低电平
			break;
		case 16:
			IO5_GPIO_Port->BSRR=IO5_Pin;//高电平
			IO6_GPIO_Port->BSRR=(IO6_Pin << 16);//低电平
			break;
		case 17:
			IO6_GPIO_Port->BSRR=IO6_Pin;//高电平
			IO7_GPIO_Port->BSRR=(IO7_Pin << 16);//低电平
			break;
		case 18:
			IO7_GPIO_Port->BSRR=IO7_Pin;//高电平
			IO8_GPIO_Port->BSRR=(IO8_Pin << 16);//低电平
			break;
		case 19:
			IO8_GPIO_Port->BSRR=IO8_Pin;//高电平
			IO9_GPIO_Port->BSRR=(IO9_Pin << 16);//低电平
			break;
		case 20:
			IO9_GPIO_Port->BSRR =IO9_Pin;//高电平
			IO10_GPIO_Port->BSRR=(IO10_Pin << 16);//低电平
			break;
	}
}
void select_switcher_all_high(void)
{
	IO1_GPIO_Port->BSRR =IO1_Pin; //高电平
	IO2_GPIO_Port->BSRR =IO2_Pin; //高电平
	IO3_GPIO_Port->BSRR =IO3_Pin; //高电平
	IO4_GPIO_Port->BSRR =IO4_Pin; //高电平
	IO5_GPIO_Port->BSRR =IO5_Pin; //高电平
	IO6_GPIO_Port->BSRR =IO6_Pin; //高电平
	IO7_GPIO_Port->BSRR =IO7_Pin; //高电平
	IO8_GPIO_Port->BSRR =IO8_Pin; //高电平
	IO9_GPIO_Port->BSRR =IO9_Pin; //高电平
	IO10_GPIO_Port->BSRR=IO10_Pin;//高电平
	IO11_GPIO_Port->BSRR=IO11_Pin;//高电平
}
void select_switcher_all_low(void)
{
	IO1_GPIO_Port->BSRR =(IO1_Pin << 16);//低电平
	IO2_GPIO_Port->BSRR =(IO2_Pin << 16);//低电平
	IO3_GPIO_Port->BSRR =(IO3_Pin << 16);//低电平
	IO4_GPIO_Port->BSRR =(IO4_Pin << 16);//低电平
	IO5_GPIO_Port->BSRR =(IO5_Pin << 16);//低电平
	IO6_GPIO_Port->BSRR =(IO6_Pin << 16);//低电平
	IO7_GPIO_Port->BSRR =(IO7_Pin << 16);//低电平
	IO8_GPIO_Port->BSRR =(IO8_Pin << 16);//低电平
	IO9_GPIO_Port->BSRR =(IO9_Pin << 16);//低电平
	IO10_GPIO_Port->BSRR=(IO10_Pin<< 16);//低电平
	IO11_GPIO_Port->BSRR=(IO11_Pin<< 16);//低电平
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
		osDelay(5);
    if(index==0)
		HAL_GPIO_WritePin(CS1_GPIO_Port,CS1_Pin,1);
	else if(index==1)
		HAL_GPIO_WritePin(CS2_GPIO_Port,CS2_Pin,1);
}
/* USER CODE END Application */

