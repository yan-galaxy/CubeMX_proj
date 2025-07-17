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
osThreadId uartTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void LedTask(void const * argument);
void UartTask(void const * argument);

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
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of ledTask */
  osThreadDef(ledTask, LedTask, osPriorityIdle, 0, 64);
  ledTaskHandle = osThreadCreate(osThread(ledTask), NULL);

  /* definition and creation of uartTask */
  osThreadDef(uartTask, UartTask, osPriorityLow, 0, 64);
  uartTaskHandle = osThreadCreate(osThread(uartTask), NULL);

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
volatile Word_union res_data_1[1004];
volatile Word_union res_data_2[1004];
volatile Word_union* res_send_p=res_data_1;
volatile Word_union* res_store_p=res_data_2;
Adc_IN_Struct adc_value_struct;

char tx_buff[128];

/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	uint16_t i=0,j=0;
	
	HAL_ADC_Start_DMA(&hadc1,(uint32_t *)(&(adc_value_struct.IN1)),10);
	// 1/12MHz*(239.5+12.5=252)cycles *10ch = 210us    ADC转换时配置为 239.5cycles
	// 1/12MHz*(55.5+12.5=68)cycles *10ch = 56.667us    ADC转换时配置为 55.5cycles
	
	TickType_t xLastWakeTime;
	
	res_data_1[0].byte[0]=0x55;//帧头
	res_data_1[0].byte[1]=0xAA;
	res_data_1[1].byte[0]=0xBB;
	res_data_1[1].byte[1]=0xCC;
	for(uint16_t i=0;i<1000;i++)
	{
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
  /* Infinite loop */
	for(;;)
	{
		for(j=0;j<10;j++)
		{
			for(i=1;i<11;i++)
			{
				Select_switcher(i-1);
				user_delaynus_tim(70);
				
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
			vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5));//osDelay(1);
		}
		
		
		
		
//		for(i=0;i<10;i++)
//		{
//			Select_switcher(i);
//			user_delaynus_tim(500);
//			sprintf(tx_buff,"line:%2d:  ",i);
//			sprintf(tx_buff+strlen(tx_buff),"%4d;%4d;%4d;%4d;%4d;%4d;%4d;%4d;%4d;%4d;\r\n",
//			adc_value_struct.IN1,adc_value_struct.IN2,adc_value_struct.IN3,adc_value_struct.IN4,adc_value_struct.IN5,
//			adc_value_struct.IN6,adc_value_struct.IN7,adc_value_struct.IN8,adc_value_struct.IN9,adc_value_struct.IN10);
//			HAL_UART_Transmit(&huart1,(uint8_t *)tx_buff,strlen(tx_buff),0xffff);
//		}
//		sprintf(tx_buff,"  \r\n");
//		HAL_UART_Transmit(&huart1,(uint8_t *)tx_buff,strlen(tx_buff),0xffff);
//		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(300));//osDelay(1);
		
		
//		HAL_UART_Transmit(&huart1,(uint8_t *)res_store_p,2008,0xffff);//阻塞发送
//		while(HAL_DMA_GetState(&hdma_usart1_tx) != HAL_DMA_STATE_READY);
		exchange_res_p();//切换缓存区
		HAL_UART_Transmit_DMA(&huart1,(uint8_t *)res_store_p,2008);
//		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));//osDelay(1);
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
		LL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
//		LL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);//LL_GPIO_SetOutputPin  LL_GPIO_ResetOutputPin
		osDelay(200);
	}
  /* USER CODE END LedTask */
}

/* USER CODE BEGIN Header_UartTask */
/**
* @brief Function implementing the uartTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UartTask */
void UartTask(void const * argument)
{
  /* USER CODE BEGIN UartTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END UartTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void Select_switcher(uint8_t index)
{
	switch(index)
	{
		case 0:
			LL_GPIO_SetOutputPin(SEL9_GPIO_Port,SEL9_Pin);//高电平
			LL_GPIO_ResetOutputPin(SEL0_GPIO_Port,SEL0_Pin);//低电平
			break;
		case 1:
			LL_GPIO_SetOutputPin(SEL0_GPIO_Port,SEL0_Pin);//高电平
			LL_GPIO_ResetOutputPin(SEL1_GPIO_Port,SEL1_Pin);//低电平
			break;
		case 2:
			LL_GPIO_SetOutputPin(SEL1_GPIO_Port,SEL1_Pin);//高电平
			LL_GPIO_ResetOutputPin(SEL2_GPIO_Port,SEL2_Pin);//低电平
			break;
		case 3:
			LL_GPIO_SetOutputPin(SEL2_GPIO_Port,SEL2_Pin);//高电平
			LL_GPIO_ResetOutputPin(SEL3_GPIO_Port,SEL3_Pin);//低电平
			break;
		case 4:
			LL_GPIO_SetOutputPin(SEL3_GPIO_Port,SEL3_Pin);//高电平
			LL_GPIO_ResetOutputPin(SEL4_GPIO_Port,SEL4_Pin);//低电平
			break;
		case 5:
			LL_GPIO_SetOutputPin(SEL4_GPIO_Port,SEL4_Pin);//高电平
			LL_GPIO_ResetOutputPin(SEL5_GPIO_Port,SEL5_Pin);//低电平
			break;
		case 6:
			LL_GPIO_SetOutputPin(SEL5_GPIO_Port,SEL5_Pin);//高电平
			LL_GPIO_ResetOutputPin(SEL6_GPIO_Port,SEL6_Pin);//低电平
			break;
		case 7:
			LL_GPIO_SetOutputPin(SEL6_GPIO_Port,SEL6_Pin);//高电平
			LL_GPIO_ResetOutputPin(SEL7_GPIO_Port,SEL7_Pin);//低电平
			break;
		case 8:
			LL_GPIO_SetOutputPin(SEL7_GPIO_Port,SEL7_Pin);//高电平
			LL_GPIO_ResetOutputPin(SEL8_GPIO_Port,SEL8_Pin);//低电平
			break;
		case 9:
			LL_GPIO_SetOutputPin(SEL8_GPIO_Port,SEL8_Pin);//高电平
			LL_GPIO_ResetOutputPin(SEL9_GPIO_Port,SEL9_Pin);//低电平
			break;
	}
}

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
	
	LL_TIM_SetCounter(TIM3, 0);
	while (LL_TIM_GetCounter(TIM3) < nus)
	{
		;// Optionally, add a timeout condition here to avoid an infinite loop
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1) //如果是串口1
	{
		
		// 在F7系列是可以不写的，F1必须写
//		__HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx, DMA_FLAG_TC4); //清除DMA2_Steam7传输完成标志
		HAL_UART_DMAStop(&huart1);		//传输完成以后关闭串口DMA,缺了这一句会死机
	}
}

void UART1_TX_DMA_Send(uint8_t *buffer, uint16_t length)
{
    //等待上一次的数据发送完毕
	while(HAL_DMA_GetState(&hdma_usart1_tx) != HAL_DMA_STATE_READY);
    //while(__HAL_DMA_GET_COUNTER(&hdma_usart1_tx));
	
    //关闭DMA
    __HAL_DMA_DISABLE(&hdma_usart1_tx);

    //开始发送数据
    HAL_UART_Transmit_DMA(&huart1, buffer, length);
}
/* USER CODE END Application */

