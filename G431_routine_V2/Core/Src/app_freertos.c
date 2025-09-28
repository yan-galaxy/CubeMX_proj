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
osThreadId acquisitionTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void LedTask(void const * argument);
void AcquisitionTask(void const * argument);

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

  /* definition and creation of ledTask */
  osThreadDef(ledTask, LedTask, osPriorityIdle, 0, 128);
  ledTaskHandle = osThreadCreate(osThread(ledTask), NULL);

  /* definition and creation of acquisitionTask */
  osThreadDef(acquisitionTask, AcquisitionTask, osPriorityRealtime, 0, 256);
  acquisitionTaskHandle = osThreadCreate(osThread(acquisitionTask), NULL);

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
Adc_IN_Struct adc_value_struct;
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	char usb_buff[1024]={0};
	unsigned long long index=0;
	
	HAL_GPIO_WritePin(SEL1_GPIO_Port, SEL1_Pin, 1);
	HAL_GPIO_WritePin(SEL2_GPIO_Port, SEL2_Pin, 1);
	HAL_GPIO_WritePin(SEL3_GPIO_Port, SEL3_Pin, 1);
	HAL_GPIO_WritePin(SEL4_GPIO_Port, SEL4_Pin, 1);
	HAL_GPIO_WritePin(SEL5_GPIO_Port, SEL5_Pin, 1);
	HAL_GPIO_WritePin(SEL6_GPIO_Port, SEL6_Pin, 1);
	HAL_GPIO_WritePin(SEL7_GPIO_Port, SEL7_Pin, 1);
	HAL_GPIO_WritePin(SEL8_GPIO_Port, SEL8_Pin, 1);
	HAL_GPIO_WritePin(SEL9_GPIO_Port, SEL9_Pin, 1);
	HAL_GPIO_WritePin(SEL10_GPIO_Port,SEL10_Pin,1);
	
	//灵敏配置：35 200 
	//灵敏极限：50 255 （测到最小电阻为9.8kΩ）
	//量程大配置：5 50
	
	AD5206_SetResistance(1,4,5);//Rref  5-0.360V   10-0.576V   25-1.097V    35-1.357V   45-1.552V   50-1.633   55-1.710V   65-1.845V   75-1.957V
	
	//50 200
	AD5206_SetResistance(1,5,50);//OUT1
	AD5206_SetResistance(0,4,50);//OUT2
	AD5206_SetResistance(1,2,50);//OUT3 50-1.95kΩ 100-3.85kΩ
	AD5206_SetResistance(1,0,50);//OUT4
	AD5206_SetResistance(1,3,50);//OUT5
	AD5206_SetResistance(1,1,50);//OUT6
	AD5206_SetResistance(0,0,50);//OUT7
	AD5206_SetResistance(0,2,50);//OUT8
	AD5206_SetResistance(0,1,50);//OUT9
	AD5206_SetResistance(0,3,50);//OUT10
	
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)(&(adc_value_struct.IN2)), 5);
	HAL_ADC_Start_DMA(&hadc2, (uint32_t *)(&(adc_value_struct.IN1)), 5);
	
  /* Infinite loop */
	for(;;)
	{
		
//		sprintf(usb_buff,"IN1:%4d ,IN2:%4d ,IN3:%4d ,IN4:%4d ,IN5:%4d ,IN6:%4d ,IN7:%4d ,IN8:%4d ,IN9:%4d ,IN10:%4d\r\n",
//			adc_value_struct.IN1,adc_value_struct.IN2,adc_value_struct.IN3,adc_value_struct.IN4,adc_value_struct.IN5,
//			adc_value_struct.IN6,adc_value_struct.IN7,adc_value_struct.IN8,adc_value_struct.IN9,adc_value_struct.IN10);
		
//		sprintf((char *)usb_buff, 
//					"Frame %02llu: \r\n", 
//					index++);
//		
//		for (uint8_t i = 1; i < 11; i++) {
//				Select_switcher(i);
//				user_delaynus_tim(100);	
//				// 1/42.5MHz*(640.5+12.5=653)cycles *5ch = 76.8235us    ADC转换时配置为 640.5cycles
//				// 1/42.5MHz*(247.5+12.5=260)cycles *5ch = 30.5882us    ADC转换时配置为 247.5cycles
//				// 1/42.5MHz*(92.5+12.5=105 )cycles *5ch = 12.3530us    ADC转换时配置为 92.5cycles
//				sprintf((char *)usb_buff + strlen((char *)usb_buff), // 
//					"line:%2d:%4d %4d %4d %4d %4d %4d %4d %4d %4d %4d \r\n", 
//						i,
//						adc_value_struct.IN1,adc_value_struct.IN2,adc_value_struct.IN3,adc_value_struct.IN4,adc_value_struct.IN5, 
//						adc_value_struct.IN6,adc_value_struct.IN7,adc_value_struct.IN8,adc_value_struct.IN9,adc_value_struct.IN10);
//			}
//		
//		CDC_Transmit_FS((uint8_t *)usb_buff,strlen(usb_buff));
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
	
	TickType_t xLastWakeTime = xTaskGetTickCount();//HAL_GPIO_TogglePin
  /* Infinite loop */
	for(;;)
	{
		HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
//		HAL_GPIO_WritePin(SEL1_GPIO_Port, SEL1_Pin, 0);
//		HAL_GPIO_WritePin(SEL2_GPIO_Port, SEL2_Pin, 0);
//		HAL_GPIO_WritePin(SEL3_GPIO_Port, SEL3_Pin, 0);
//		HAL_GPIO_WritePin(SEL4_GPIO_Port, SEL4_Pin, 0);
//		HAL_GPIO_WritePin(SEL5_GPIO_Port, SEL5_Pin, 0);
//		HAL_GPIO_WritePin(SEL6_GPIO_Port, SEL6_Pin, 0);
//		HAL_GPIO_WritePin(SEL7_GPIO_Port, SEL7_Pin, 0);
//		HAL_GPIO_WritePin(SEL8_GPIO_Port, SEL8_Pin, 0);
//		HAL_GPIO_WritePin(SEL9_GPIO_Port, SEL9_Pin, 0);
//		HAL_GPIO_WritePin(SEL10_GPIO_Port,SEL10_Pin,0);
//		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
//		
//		
//		
//		HAL_GPIO_WritePin(SEL1_GPIO_Port, SEL1_Pin, 1);
//		HAL_GPIO_WritePin(SEL2_GPIO_Port, SEL2_Pin, 1);
//		HAL_GPIO_WritePin(SEL3_GPIO_Port, SEL3_Pin, 1);
//		HAL_GPIO_WritePin(SEL4_GPIO_Port, SEL4_Pin, 1);
//		HAL_GPIO_WritePin(SEL5_GPIO_Port, SEL5_Pin, 1);
//		HAL_GPIO_WritePin(SEL6_GPIO_Port, SEL6_Pin, 1);
//		HAL_GPIO_WritePin(SEL7_GPIO_Port, SEL7_Pin, 1);
//		HAL_GPIO_WritePin(SEL8_GPIO_Port, SEL8_Pin, 1);
//		HAL_GPIO_WritePin(SEL9_GPIO_Port, SEL9_Pin, 1);
//		HAL_GPIO_WritePin(SEL10_GPIO_Port,SEL10_Pin,1);
//		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(3));
		
	}
  /* USER CODE END LedTask */
}

/* USER CODE BEGIN Header_AcquisitionTask */
/**
* @brief Function implementing the acquisitionTask thread.
* @param argument: Not used
* @retval None
*/
volatile Word_union print_data[1004];
volatile Word_union res_data_1[1004];
volatile Word_union res_data_2[1004];
volatile Word_union* res_send_p=res_data_1;
volatile Word_union* res_store_p=res_data_2;
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
/* USER CODE END Header_AcquisitionTask */
void AcquisitionTask(void const * argument)
{
  /* USER CODE BEGIN AcquisitionTask */
	osDelay(500);
	TickType_t xLastWakeTime;
	
	res_data_1[0].byte[0]=0x55;//帧头
	res_data_1[0].byte[1]=0xAA;
	res_data_1[1].byte[0]=0xBB;
	res_data_1[1].byte[1]=0xCC;
	for(uint16_t i=0;i<1000;i++)
	{
//		res_send_p[2+i].word16=i*1+1;
		res_data_1[2+i].word16=i*1+1;
	}
	res_data_1[1002].byte[0]=0xAA;//帧尾
	res_data_1[1002].byte[1]=0x55;
	res_data_1[1003].byte[0]=0x66;
	res_data_1[1003].byte[1]=0x77;
	
	res_data_2[0].byte[0]=0x55;//帧头
	res_data_2[0].byte[1]=0xAA;
	res_data_2[1].byte[0]=0xBB;
	res_data_2[1].byte[1]=0xCC;
	for(uint16_t i=0;i<1000;i++)
	{
		res_data_2[2+i].word16=i*2+2;
	}
	res_data_2[1002].byte[0]=0xAA;//帧尾
	res_data_2[1002].byte[1]=0x55;
	res_data_2[1003].byte[0]=0x66;
	res_data_2[1003].byte[1]=0x77;
	
	xLastWakeTime = xTaskGetTickCount();
	// 1/42.5MHz*(640.5+12.5=653)cycles *5ch = 76.8235us    ADC转换时配置为 640.5cycles
	// 1/42.5MHz*(247.5+12.5=260)cycles *5ch = 30.5882us    ADC转换时配置为 247.5cycles
	// 1/42.5MHz*(92.5+12.5=105 )cycles *5ch = 12.3530us    ADC转换时配置为 92.5cycles
  /* Infinite loop */
	for(;;)
	{
		for(uint8_t j=0;j<10;j++)
		{
			for(uint8_t i=1;i<11;i++)
			{
				Select_switcher(i);//i
				user_delaynus_tim(50);
				res_store_p[j*100+(i-1)*10+0+2].word16=adc_value_struct.IN1;
				res_store_p[j*100+(i-1)*10+1+2].word16=adc_value_struct.IN2;
				res_store_p[j*100+(i-1)*10+2+2].word16=adc_value_struct.IN3;
				res_store_p[j*100+(i-1)*10+3+2].word16=adc_value_struct.IN4;
				res_store_p[j*100+(i-1)*10+4+2].word16=adc_value_struct.IN5;
				res_store_p[j*100+(i-1)*10+5+2].word16=adc_value_struct.IN6;
				res_store_p[j*100+(i-1)*10+6+2].word16=adc_value_struct.IN7;
				res_store_p[j*100+(i-1)*10+7+2].word16=adc_value_struct.IN8;
				res_store_p[j*100+(i-1)*10+8+2].word16=adc_value_struct.IN9;
				res_store_p[j*100+(i-1)*10+9+2].word16=adc_value_struct.IN10;
			}
				vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));//osDelay(1);
		}
//		osDelay(100);
		exchange_res_p();//切换缓存区
		CDC_Transmit_FS(res_send_p->byte, 2008);
	}
  /* USER CODE END AcquisitionTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
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

//切换列选  把上一个列的引脚置高电平 当前列置低电平
/*
index : 1~10
*/
void Select_switcher(uint8_t index)
{
	switch(index)
	{
		case 1:
			SEL10_GPIO_Port->BSRR=SEL10_pin_HIGH;//高电平
			SEL1_GPIO_Port->BSRR=SEL1_pin_LOW;//低电平
			break;
		case 2:
			SEL1_GPIO_Port->BSRR=SEL1_pin_HIGH;//高电平
			SEL2_GPIO_Port->BSRR=SEL2_pin_LOW;//低电平
			break;
		case 3:
			SEL2_GPIO_Port->BSRR=SEL2_pin_HIGH;//高电平
			SEL3_GPIO_Port->BSRR=SEL3_pin_LOW;//低电平
			break;
		case 4:
			SEL3_GPIO_Port->BSRR=SEL3_pin_HIGH;//高电平
			SEL4_GPIO_Port->BSRR=SEL4_pin_LOW;//低电平
			break;
		case 5:
			SEL4_GPIO_Port->BSRR=SEL4_pin_HIGH;//高电平
			SEL5_GPIO_Port->BSRR=SEL5_pin_LOW;//低电平
			break;
		case 6:
			SEL5_GPIO_Port->BSRR=SEL5_pin_HIGH;//高电平
			SEL6_GPIO_Port->BSRR=SEL6_pin_LOW;//低电平
			break;
		case 7:
			SEL6_GPIO_Port->BSRR=SEL6_pin_HIGH;//高电平
			SEL7_GPIO_Port->BSRR=SEL7_pin_LOW;//低电平
			break;
		case 8:
			SEL7_GPIO_Port->BSRR=SEL7_pin_HIGH;//高电平
			SEL8_GPIO_Port->BSRR=SEL8_pin_LOW;//低电平
			break;
		case 9:
			SEL8_GPIO_Port->BSRR=SEL8_pin_HIGH;//高电平
			SEL9_GPIO_Port->BSRR=SEL9_pin_LOW;//低电平
			break;
		case 10:
			SEL9_GPIO_Port->BSRR=SEL9_pin_HIGH;//高电平
			SEL10_GPIO_Port->BSRR=SEL10_pin_LOW;//低电平
			break;
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
/* USER CODE END Application */

