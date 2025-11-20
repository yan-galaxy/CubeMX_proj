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
#include "event_groups.h"
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
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 512 * 4
};
/* Definitions for ledTask */
osThreadId_t ledTaskHandle;
const osThreadAttr_t ledTask_attributes = {
  .name = "ledTask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for resTask */
osThreadId_t resTaskHandle;
const osThreadAttr_t resTask_attributes = {
  .name = "resTask",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 128 * 4
};
/* Definitions for micTask */
osThreadId_t micTaskHandle;
const osThreadAttr_t micTask_attributes = {
  .name = "micTask",
  .priority = (osPriority_t) osPriorityAboveNormal1,
  .stack_size = 128 * 4
};
/* Definitions for usb_tx_BinarySem */
osSemaphoreId_t usb_tx_BinarySemHandle;
const osSemaphoreAttr_t usb_tx_BinarySem_attributes = {
  .name = "usb_tx_BinarySem"
};
/* Definitions for adc_dma_Event */
osEventFlagsId_t adc_dma_EventHandle;
const osEventFlagsAttr_t adc_dma_Event_attributes = {
  .name = "adc_dma_Event"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void LEDTask(void *argument);
void ResTask(void *argument);
void MicTask(void *argument);

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
  /* creation of usb_tx_BinarySem */
  usb_tx_BinarySemHandle = osSemaphoreNew(1, 1, &usb_tx_BinarySem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	if (usb_tx_BinarySemHandle != NULL) {
    osSemaphoreAcquire(usb_tx_BinarySemHandle, 0);  // 超时0，立即尝试获取
  }
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of ledTask */
  ledTaskHandle = osThreadNew(LEDTask, NULL, &ledTask_attributes);

  /* creation of resTask */
  resTaskHandle = osThreadNew(ResTask, NULL, &resTask_attributes);

  /* creation of micTask */
  micTaskHandle = osThreadNew(MicTask, NULL, &micTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the event(s) */
  /* creation of adc_dma_Event */
  adc_dma_EventHandle = osEventFlagsNew(&adc_dma_Event_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
// 事件标志宏定义（5个ADC，每个有半满和全满事件）
#define EVENT_MIC_DMA_HALF  (0x0001UL)  // ADC2半满
#define EVENT_MIC_DMA_FULL  (0x0002UL)  // ADC2全满
#define EVENT_RES_DMA_HALF  (0x0004UL)  // ADC1半满
#define EVENT_RES_DMA_FULL  (0x0008UL)  // ADC1全满
						
						
#define FRAME_HEAD_CNT 2    // 帧头数量
#define FRAME_TAIL_CNT 2    // 帧尾数量



Sample_total_Data_Union sample_buf[2];

//#define PACK_FRAME 100
uint16_t mic_value[1000];//500 * 2

volatile Word_union* mic_send_p= sample_buf[0].mic_data;
volatile Word_union* mic_store_p= sample_buf[1].mic_data;

uint16_t res_value_dma[800];//360 * 2 -> 400 * 2
uint16_t res_value[36];
volatile Word_union* res_send_p= sample_buf[0].res_data;
volatile Word_union* res_store_p= sample_buf[1].res_data;

uint8_t frame_head[4]={0x55,0xAA,0xBB,0xCC};
uint8_t frame_tail[4]={0xAA,0x55,0x66,0x77};

// 模拟开关控制值表（对应index 1~9，最后补0xFFFB循环触发）
const uint16_t switch_odr_table[] = {
  0xFFFB, // index 1：PB2=0
  0xFFFD, // index 2：PB1=0
  0xFFFE, // index 3：PB0=0
  0xFFDF, // index 4：PB5=0
  0xFFEF, // index 5：PB4=0
  0xFFF7, // index 6：PB3=0
  0xFDFF, // index 7：PB9=0
  0xFF7F, // index 8：PB7=0
  0xFFBF, // index 9：PB6=0
  0xFFFF  // index 10: None
};
#define SWITCH_CNT 10 // 模拟开关数量 真实为9次，但是为了跟mic同步 选择为10
#define SWITCH_TABLE_LEN (SWITCH_CNT)


//电压采集完成之后需要进行缓存切换
void exchange_res_p(void)
{
	static uint8_t state=1;//第一次运行该函数前 采集的数据存到res_data_2   第一次运行该函数后状态变为0，发送的指针指向res_data_1  之后循环往复
	state=!state;
	
	if(state)
	{
		res_send_p=sample_buf[0].res_data;
		res_store_p=sample_buf[1].res_data;
	}
	else
	{
		res_send_p=sample_buf[1].res_data;
		res_store_p=sample_buf[0].res_data;
	}
}
char usb_buff[128]={0};
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */


	
	
	
  /* Infinite loop */
	for(;;)
	{
//		HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
//		sprintf(usb_buff,"MIC1:%d\r\nMIC2:%d\r\nMIC3:%d\r\nMIC4:%d\r\nMIC5:%d\r\n\r\n",mic_value[0],mic_value[1],mic_value[2],mic_value[3],mic_value[4]);
//		CDC_Transmit_FS((uint8_t *)usb_buff,strlen(usb_buff));
		osDelay(100);
		

	}
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_LEDTask */
/**
* @brief Function implementing the ledTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LEDTask */
void LEDTask(void *argument)
{
  /* USER CODE BEGIN LEDTask */
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
  /* Infinite loop */
	for(;;)
	{
		HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
	}
  /* USER CODE END LEDTask */
}

/* USER CODE BEGIN Header_ResTask */
/**
* @brief Function implementing the resTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ResTask */
void ResTask(void *argument)
{
  /* USER CODE BEGIN ResTask */
	AD5206_SetResistance(0, 50);
	AD5206_SetResistance(1, 50);
	AD5206_SetResistance(2, 50);
	AD5206_SetResistance(4, 50);//参考电压
	AD5206_SetResistance(5, 50);
	
	
	HAL_ADCEx_Calibration_Start(&hadc2,ADC_SINGLE_ENDED);
	HAL_ADC_Start_DMA(&hadc2, (uint32_t *)mic_value, 1000);
	HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)res_value_dma, 800);//adc触发源需要选择下降沿
	// 第一步：初始化DMA传输（指定源地址、目标地址、传输数量，仅CH1需要DMA）
	// 源：开关控制数组（switch_odr_table），目标：GPIOB->ODR，传输次数：SWITCH_CNT（9次）
	HAL_DMA_Start(
	  &hdma_tim1_ch1,                // TIM1_CH1对应的DMA通道句柄
	  (uint32_t)switch_odr_table,    // DMA源地址：开关控制值数组
	  (uint32_t)&GPIOB->ODR,         // DMA目标地址：GPIO输出寄存器（控制模拟开关）
	  SWITCH_CNT                     // 传输数量：9个开关，循环传输
	);

	// 第二步：启用TIM1_CH1的DMA请求（关键！必须在通道启用前开启，确保DMA就绪）
	__HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_CC1);  // 使能CH1的比较事件触发DMA请求
	
	
	// 确认TIM1_CH3和计数器已启动（防止初始化时遗漏）
	TIM_CCxChannelCmd(htim1.Instance, TIM_CHANNEL_1, TIM_CCx_ENABLE);
	TIM_CCxChannelCmd(htim1.Instance, TIM_CHANNEL_2, TIM_CCx_ENABLE);
	TIM_CCxChannelCmd(htim1.Instance, TIM_CHANNEL_3, TIM_CCx_ENABLE);
	
	__HAL_TIM_MOE_ENABLE(&htim1);
	__HAL_TIM_ENABLE(&htim1);
	
	
	
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	uint32_t ulReceivedEvents;          // 接收的事件标志
	uint16_t* send_res_p=res_value_dma;
	uint16_t* send_mic_p=mic_value;
	
	uint8_t USB_result = USBD_OK;
  /* Infinite loop */
	for(;;)
	{
		// 等待：ADC半满 或 ADC全满（任意一组满足即响应）
		ulReceivedEvents = osEventFlagsWait(
		  adc_dma_EventHandle,            // 事件组句柄（你的原有定义）
		  EVENT_RES_DMA_HALF | EVENT_RES_DMA_FULL, // 等待的事件组合      EVENT_ALL_HALF | EVENT_ALL_FULL  EVENT_ADC5_DMA_HALF | EVENT_ADC5_DMA_FULL
		  osFlagsWaitAny,                 // 任意事件满足即返回
		  osWaitForever                    // 永久等待（直到事件触发）
		);
		
		if (ulReceivedEvents & EVENT_RES_DMA_HALF)//EVENT_ADC5_DMA_HALF  EVENT_ALL_HALF
		{
			  send_res_p = res_value_dma;
			  // 清除所有半满事件标志（避免重复处理）
			osEventFlagsClear(adc_dma_EventHandle, EVENT_RES_DMA_HALF);
		}
		else if (ulReceivedEvents & EVENT_RES_DMA_FULL)//EVENT_ADC5_DMA_FULL  EVENT_ALL_FULL
		{
			send_res_p = res_value_dma+400;
			  // 清除所有全满事件标志
			osEventFlagsClear(adc_dma_EventHandle, EVENT_RES_DMA_FULL);
		}
		
		
		
		if (CDC_Transmit_FS(frame_head, 4) == USBD_OK)
		{
			// 等待发送完成（超时时间可根据需求设置）
			osSemaphoreAcquire(usb_tx_BinarySemHandle, osWaitForever);  // 阻塞等待，CPU可调度其他任务
		}
		if (CDC_Transmit_FS((uint8_t *)send_res_p, 800) == USBD_OK)
		{
			// 等待发送完成（超时时间可根据需求设置）
			osSemaphoreAcquire(usb_tx_BinarySemHandle, osWaitForever);  // 阻塞等待，CPU可调度其他任务
		}
		if (CDC_Transmit_FS(frame_tail, 4) == USBD_OK)
		{
			// 等待发送完成（超时时间可根据需求设置）
			osSemaphoreAcquire(usb_tx_BinarySemHandle, osWaitForever);  // 阻塞等待，CPU可调度其他任务
		}
		
		
		
		
	}
  /* USER CODE END ResTask */
}

/* USER CODE BEGIN Header_MicTask */
/**
* @brief Function implementing the micTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MicTask */
void MicTask(void *argument)
{
  /* USER CODE BEGIN MicTask */
	HAL_TIM_Base_Start(&htim6);
//	HAL_ADCEx_Calibration_Start(&hadc2,ADC_SINGLE_ENDED);
//	HAL_ADC_Start_DMA(&hadc2, (uint32_t *)mic_value, 1000);
	
////	// 第一步：初始化DMA传输（指定源地址、目标地址、传输数量，仅CH1需要DMA）
////	// 源：开关控制数组（switch_odr_table），目标：GPIOB->ODR，传输次数：SWITCH_CNT（9次）
////	HAL_DMA_Start(
////	  &hdma_tim1_ch1,                // TIM1_CH1对应的DMA通道句柄
////	  (uint32_t)switch_odr_table,    // DMA源地址：开关控制值数组
////	  (uint32_t)&GPIOB->ODR,         // DMA目标地址：GPIO输出寄存器（控制模拟开关）
////	  SWITCH_CNT                     // 传输数量：9个开关，循环传输
////	);

////	// 第二步：启用TIM1_CH1的DMA请求（关键！必须在通道启用前开启，确保DMA就绪）
////	__HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_CC1);  // 使能CH1的比较事件触发DMA请求
//	
//	
////	// 确认TIM1_CH3和计数器已启动（防止初始化时遗漏）
//	TIM_CCxChannelCmd(htim1.Instance, TIM_CHANNEL_3, TIM_CCx_ENABLE);
//	
//	__HAL_TIM_MOE_ENABLE(&htim1);
//	__HAL_TIM_ENABLE(&htim1);
	
	
	
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
//	uint32_t ulReceivedEvents;          // 接收的事件标志
//	
//	uint8_t USB_result = USBD_OK;
//	uint16_t* send_mic_p=mic_value;
  /* Infinite loop */
	for(;;)
	{
//		// 等待：ADC半满 或 ADC全满（任意一组满足即响应）
//		ulReceivedEvents = osEventFlagsWait(
//		  adc_dma_EventHandle,            // 事件组句柄（你的原有定义）
//		  EVENT_MIC_DMA_HALF | EVENT_MIC_DMA_FULL, // 等待的事件组合      EVENT_ALL_HALF | EVENT_ALL_FULL  EVENT_ADC5_DMA_HALF | EVENT_ADC5_DMA_FULL
//		  osFlagsWaitAny,                 // 任意事件满足即返回
//		  osWaitForever                    // 永久等待（直到事件触发）
//		);
//		
//		if (ulReceivedEvents & EVENT_MIC_DMA_HALF)//EVENT_ADC5_DMA_HALF  EVENT_ALL_HALF
//		{
//			  send_mic_p = mic_value;
//			  // 清除所有半满事件标志（避免重复处理）
//			osEventFlagsClear(adc_dma_EventHandle, EVENT_MIC_DMA_HALF);
//		}
//		else if (ulReceivedEvents & EVENT_MIC_DMA_FULL)//EVENT_ADC5_DMA_FULL  EVENT_ALL_FULL
//		{
//			send_mic_p = mic_value+500;
//			  // 清除所有全满事件标志
//			osEventFlagsClear(adc_dma_EventHandle, EVENT_MIC_DMA_FULL);
//		}
		
		
		
//		if (CDC_Transmit_FS(frame_head, 4) == USBD_OK)
//		{
//			// 等待发送完成（超时时间可根据需求设置）
//			osSemaphoreAcquire(usb_tx_BinarySemHandle, osWaitForever);  // 阻塞等待，CPU可调度其他任务
//		}
//		if (CDC_Transmit_FS((uint8_t *)send_mic_p, 1000) == USBD_OK)
//		{
//			// 等待发送完成（超时时间可根据需求设置）
//			osSemaphoreAcquire(usb_tx_BinarySemHandle, osWaitForever);  // 阻塞等待，CPU可调度其他任务
//		}
//		if (CDC_Transmit_FS(frame_tail, 4) == USBD_OK)
//		{
//			// 等待发送完成（超时时间可根据需求设置）
//			osSemaphoreAcquire(usb_tx_BinarySemHandle, osWaitForever);  // 阻塞等待，CPU可调度其他任务
//		}
		
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
		
	}
  /* USER CODE END MicTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void user_delaynus_tim(uint16_t nus)
{
	
	LL_TIM_SetCounter(TIM7, 0);
	while (LL_TIM_GetCounter(TIM7) < nus)
	{
		;// Optionally, add a timeout condition here to avoid an infinite loop
	}
}

/* USER CODE BEGIN Application */
// 设置AD5206的电阻值
// channel: 0~5
// resistance: 0~255
void AD5206_SetResistance(uint8_t channel, uint8_t resistance) {
		uint8_t spi_data[2]={0x00,10};
		
    // 确保通道和电阻值在有效范围内
    if (channel > 5) return; // AD5206有6个通道，通道号从0到5
    if (resistance > 255) return; // 电阻值应在0到255之间
	
	HAL_GPIO_WritePin(CS_GPIO_Port,CS_Pin,0);
		
	spi_data[0]=channel;
	spi_data[1]=resistance;
	
	HAL_SPI_Transmit(&hspi2,spi_data,2,0xffff);
	osDelay(5);
	HAL_GPIO_WritePin(CS_GPIO_Port,CS_Pin,1);
}
void Select_switcher(uint8_t index)
{
    // 2. 根据index设置目标引脚为低，其他控制引脚为高
    switch(index)
    {
        case 1:  // 选通PB2（bit2=0）0xFFFB
			GPIOB->ODR = 0xFFFB;
            break;
        case 2:  // 选通PB1（bit1=0）0xFFFD
			GPIOB->ODR = 0xFFFD;
            break;
        case 3:  // 选通PB0（bit0=0）0xFFFE
			GPIOB->ODR = 0xFFFE;
            break;
        case 4:  // 选通PB5（bit5=0）0xFFDF
			GPIOB->ODR = 0xFFDF;
            break;
        case 5:  // 选通PB4（bit4=0）0xFFEF
			GPIOB->ODR = 0xFFEF;
            break;
        case 6:  // 选通PB3（bit3=0）0xFFF7
			GPIOB->ODR = 0xFFF7;
            break;
        case 7:  // 选通PB9（bit9=0）0xFDFF
			GPIOB->ODR = 0xFDFF;//1111 1101 1111 1111
            break;
        case 8:  // 选通PB7（bit7=0）0xFF7F
			GPIOB->ODR = 0xFF7F;//1111 1111 0111 1111
            break;
        case 9:  // 选通PB6（bit6=0）0xFFBF
			GPIOB->ODR = 0xFFBF;//1111 1111 1011 1111
            break;
        default: // 默认：所有控制引脚为高（不选通任何路）
            GPIOB->ODR = 0xFFFF; // 0x3FF（1111111111）
            break;
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	EventGroupHandle_t xEventGroup = (EventGroupHandle_t)adc_dma_EventHandle; // 转换句柄类型
	if(hadc == & hadc1)
	{
		xEventGroupSetBitsFromISR(
		  xEventGroup,          // 事件组句柄（转换后）
		  EVENT_RES_DMA_FULL,  // 要设置的事件标志
		  &xHigherPriorityTaskWoken  // 输出参数：是否需要切换任务
		);
	}
	else if(hadc == & hadc2)
	{
		xEventGroupSetBitsFromISR(
		  xEventGroup,          // 事件组句柄（转换后）
		  EVENT_MIC_DMA_FULL,  // 要设置的事件标志
		  &xHigherPriorityTaskWoken  // 输出参数：是否需要切换任务
		);
	}
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	EventGroupHandle_t xEventGroup = (EventGroupHandle_t)adc_dma_EventHandle; // 转换句柄类型
	if(hadc == & hadc1)
	{
		xEventGroupSetBitsFromISR(
		  xEventGroup,          // 事件组句柄（转换后）
		  EVENT_RES_DMA_HALF,  // 要设置的事件标志
		  &xHigherPriorityTaskWoken  // 输出参数：是否需要切换任务
		);
	}
	else if(hadc == & hadc2)
	{
		xEventGroupSetBitsFromISR(
		  xEventGroup,          // 事件组句柄（转换后）
		  EVENT_MIC_DMA_HALF,  // 要设置的事件标志
		  &xHigherPriorityTaskWoken  // 输出参数：是否需要切换任务
		);
	}
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
/* USER CODE END Application */

