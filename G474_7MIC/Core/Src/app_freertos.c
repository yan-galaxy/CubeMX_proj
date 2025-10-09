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
// �¼���־�궨�壨5��ADC��ÿ���а�����ȫ���¼���
#define EVENT_ADC1_DMA_HALF  (0x0001UL)  // ADC1������0~199���ݣ���ADC1������400��
#define EVENT_ADC1_DMA_FULL  (0x0002UL)  // ADC1ȫ����200~399���ݣ�
#define EVENT_ADC2_DMA_HALF  (0x0004UL)  // ADC2������0~199��
#define EVENT_ADC2_DMA_FULL  (0x0008UL)  // ADC2ȫ����200~399��
#define EVENT_ADC3_DMA_HALF  (0x0010UL)  // ADC3������0~99��
#define EVENT_ADC3_DMA_FULL  (0x0020UL)  // ADC3ȫ����100~199��
#define EVENT_ADC4_DMA_HALF  (0x0040UL)  // ADC4������0~99��
#define EVENT_ADC4_DMA_FULL  (0x0080UL)  // ADC4ȫ����100~199��
#define EVENT_ADC5_DMA_HALF  (0x0100UL)  // ADC5������0~99��
#define EVENT_ADC5_DMA_FULL  (0x0200UL)  // ADC5ȫ����100~199��

// 2. ��ϱ�־������ADC�İ����¼�������ͬ���ȴ���
#define EVENT_ALL_HALF (EVENT_ADC1_DMA_HALF | EVENT_ADC2_DMA_HALF | \
                        EVENT_ADC3_DMA_HALF | EVENT_ADC4_DMA_HALF | EVENT_ADC5_DMA_HALF)
// 3. ��ϱ�־������ADC��ȫ���¼�
#define EVENT_ALL_FULL (EVENT_ADC1_DMA_FULL | EVENT_ADC2_DMA_FULL | \
                        EVENT_ADC3_DMA_FULL | EVENT_ADC4_DMA_FULL | EVENT_ADC5_DMA_FULL)
						
// 2. ��ϱ�־������ADC�İ����¼�������ͬ���ȴ���
#define EVENT_345_HALF (EVENT_ADC3_DMA_HALF | EVENT_ADC4_DMA_HALF | EVENT_ADC5_DMA_HALF)
// 3. ��ϱ�־������ADC��ȫ���¼�
#define EVENT_345_FULL (EVENT_ADC3_DMA_FULL | EVENT_ADC4_DMA_FULL | EVENT_ADC5_DMA_FULL)
						
						
// ����֡�ṹ���壺2��֡ͷ��+5��100��5·ADC���ݣ�+2��֡β��=504��Word_union
#define ADC_DATA_CNT  100    // ÿ·ADCͨ�����η���������
#define FRAME_HEAD_CNT 2    // ֡ͷ����
#define FRAME_TAIL_CNT 2    // ֡β����
#define TOTAL_WORD_CNT (FRAME_HEAD_CNT + 5*ADC_DATA_CNT + FRAME_TAIL_CNT)
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
  .stack_size = 2048 * 4
};
/* Definitions for ledTask */
osThreadId_t ledTaskHandle;
const osThreadAttr_t ledTask_attributes = {
  .name = "ledTask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 64 * 4
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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of ledTask */
  ledTaskHandle = osThreadNew(LEDTask, NULL, &ledTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the event(s) */
  /* creation of adc_dma_Event */
  adc_dma_EventHandle = osEventFlagsNew(&adc_dma_Event_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
//  EventGroupHandle_t xEventGroup = (EventGroupHandle_t)adc_dma_EventHandle; // ת���������
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
#define PACK_FRAME 100
uint16_t ADC1_value[PACK_FRAME*4];//MIC7 MIC4
uint16_t ADC2_value[PACK_FRAME*4];//MIC5 MIC6
uint16_t ADC3_value[PACK_FRAME*2];//MIC2
uint16_t ADC4_value[PACK_FRAME*2];//MIC3
uint16_t ADC5_value[PACK_FRAME*2];//MIC1


volatile Word_union mic_data_1[PACK_FRAME*7+4];
volatile Word_union mic_data_2[PACK_FRAME*7+4];
volatile Word_union* mic_send_p=mic_data_1;
volatile Word_union* mic_store_p=mic_data_2;

//��ѹ�ɼ����֮����Ҫ���л����л�
void exchange_res_p(void)
{
	static uint8_t state=1;//��һ�����иú���ǰ �ɼ������ݴ浽mic_data_2   ��һ�����иú�����״̬��Ϊ0�����͵�ָ��ָ��mic_data_1  ֮��ѭ������
	state=!state;
	
	if(state)
	{
		mic_send_p=mic_data_1;
		mic_store_p=mic_data_2;
	}
	else
	{
		mic_send_p=mic_data_2;
		mic_store_p=mic_data_1;
	}
}

/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	char usb_buff[4096]={0};
	char usb_buff2[256]={0};
	LL_TIM_SetCounter(TIM7, 0);
	LL_TIM_EnableCounter(TIM7);//Enable TIM7
	HAL_TIM_Base_Start(&htim6);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	
	
	mic_data_1[0].byte[0]=0x55;//֡ͷ
	mic_data_1[0].byte[1]=0xAA;
	mic_data_1[1].byte[0]=0xBB;
	mic_data_1[1].byte[1]=0xCC;
	for(uint16_t i=0;i<PACK_FRAME*7;i++)
	{
		mic_data_1[2+i].word16=i*1+1;
	}
	mic_data_1[PACK_FRAME*7+2].byte[0]=0xAA;//֡β
	mic_data_1[PACK_FRAME*7+2].byte[1]=0x55;
	mic_data_1[PACK_FRAME*7+3].byte[0]=0x66;
	mic_data_1[PACK_FRAME*7+3].byte[1]=0x77;
	
	mic_data_2[0].byte[0]=0x55;//֡ͷ
	mic_data_2[0].byte[1]=0xAA;
	mic_data_2[1].byte[0]=0xBB;
	mic_data_2[1].byte[1]=0xCC;
	for(uint16_t i=0;i<PACK_FRAME*7;i++)
	{
		mic_data_2[2+i].word16=i*2+2;
	}
	mic_data_2[PACK_FRAME*7+2].byte[0]=0xAA;//֡β
	mic_data_2[PACK_FRAME*7+2].byte[1]=0x55;
	mic_data_2[PACK_FRAME*7+3].byte[0]=0x66;
	mic_data_2[PACK_FRAME*7+3].byte[1]=0x77;
	
	HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(&hadc2,ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(&hadc3,ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(&hadc4,ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(&hadc5,ADC_SINGLE_ENDED);
	
	osDelay(100);
	
	//Ҫ�ر�ADC������ת������Ȼ���ܶ�ʱ��������Լ����һֱת��
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC1_value, PACK_FRAME*4);
	HAL_ADC_Start_DMA(&hadc2, (uint32_t *)ADC2_value, PACK_FRAME*4);
	HAL_ADC_Start_DMA(&hadc3, (uint32_t *)ADC3_value, PACK_FRAME*2);
	HAL_ADC_Start_DMA(&hadc4, (uint32_t *)ADC4_value, PACK_FRAME*2);
	HAL_ADC_Start_DMA(&hadc5, (uint32_t *)ADC5_value, PACK_FRAME*2);
	
	uint32_t t_start = 0, t_string = 0, t_send = 0, t_send2 = 0, t_end = 0;
	uint8_t CDC_result = USBD_OK;
	
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	uint32_t ulReceivedEvents;          // ���յ��¼���־
	
	LL_TIM_SetCounter(TIM7, 0);
  /* Infinite loop */
	for(;;)
	{
		// �ȴ�������ADC���� �� ����ADCȫ��������һ�����㼴��Ӧ��
		ulReceivedEvents = osEventFlagsWait(
		  adc_dma_EventHandle,            // �¼����������ԭ�ж��壩
		  EVENT_ALL_HALF | EVENT_ALL_FULL, // �ȴ����¼����      EVENT_ALL_HALF | EVENT_ALL_FULL  EVENT_ADC5_DMA_HALF | EVENT_ADC5_DMA_FULL
		  osFlagsWaitAny,                 // �����¼����㼴����
		  osWaitForever                    // ���õȴ���ֱ���¼�������
		);

		t_start = LL_TIM_GetCounter(TIM7);
		
		if (ulReceivedEvents & EVENT_ALL_HALF)//EVENT_ADC5_DMA_HALF  EVENT_ALL_HALF
		{
			// ����ADC5�������ݣ�0~99��
			  for(uint16_t i=0; i<PACK_FRAME; i++) {
				mic_store_p[FRAME_HEAD_CNT + i].word16 = ADC5_value[i];
				mic_store_p[FRAME_HEAD_CNT + PACK_FRAME*1 + i].word16 = ADC3_value[i];
				mic_store_p[FRAME_HEAD_CNT + PACK_FRAME*2 + i].word16 = ADC4_value[i];
				mic_store_p[FRAME_HEAD_CNT + PACK_FRAME*3 + i].word16 = ADC1_value[1+i*2];
				mic_store_p[FRAME_HEAD_CNT + PACK_FRAME*4 + i].word16 = ADC2_value[i*2];
				mic_store_p[FRAME_HEAD_CNT + PACK_FRAME*5 + i].word16 = ADC2_value[1+i*2];
				mic_store_p[FRAME_HEAD_CNT + PACK_FRAME*6 + i].word16 = ADC1_value[i*2];
			  }
			  // ������а����¼���־�������ظ�����
			osEventFlagsClear(adc_dma_EventHandle, EVENT_ALL_HALF);
		}
		else if (ulReceivedEvents & EVENT_ALL_FULL)//EVENT_ADC5_DMA_FULL  EVENT_ALL_FULL
		{
			// ����ADC5ȫ�����ݣ�100~199��
			  for(uint16_t i=0; i<PACK_FRAME; i++) {
				mic_store_p[FRAME_HEAD_CNT + i].word16 = ADC5_value[i+PACK_FRAME];
				mic_store_p[FRAME_HEAD_CNT + PACK_FRAME*1 + i].word16 = ADC3_value[i+PACK_FRAME];
				mic_store_p[FRAME_HEAD_CNT + PACK_FRAME*2 + i].word16 = ADC4_value[i+PACK_FRAME];
				mic_store_p[FRAME_HEAD_CNT + PACK_FRAME*3 + i].word16 = ADC1_value[1+i*2+PACK_FRAME*2];
				mic_store_p[FRAME_HEAD_CNT + PACK_FRAME*4 + i].word16 = ADC2_value[i*2+PACK_FRAME*2];
				mic_store_p[FRAME_HEAD_CNT + PACK_FRAME*5 + i].word16 = ADC2_value[1+i*2+PACK_FRAME*2];
				mic_store_p[FRAME_HEAD_CNT + PACK_FRAME*6 + i].word16 = ADC1_value[i*2+PACK_FRAME*2];
			  }
			  // �������ȫ���¼���־
			osEventFlagsClear(adc_dma_EventHandle, EVENT_ALL_FULL);
		}//PACK_FRAMEΪ100ʱ���Ե�������ֵ����80us
		t_string = LL_TIM_GetCounter(TIM7)-t_start;
		
//		sprintf(usb_buff2,"t_send:%.1f us,t_send2:%.1f us,t_string:%.1f us,CDC_result:%u,callback_p:%p,hdma5_callback_p:%p\r\n",t_send/1.0,t_send2/1.0,t_string/1.0,CDC_result,callback_p,hdma_adc5.XferCpltCallback);
//		CDC_Transmit_FS((uint8_t *)usb_buff2,strlen(usb_buff2));
//		CDC_Transmit_FS((uint8_t *)usb_buff,strlen(usb_buff));
		
		exchange_res_p();//�л�������
		CDC_Transmit_FS((uint8_t *)mic_send_p->byte,(PACK_FRAME*7+4)*2);
		
		usb_buff[0]=0;

//		osDelay(1);
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(PACK_FRAME/10));
	}

/*
	
//		sprintf(usb_buff,"%d,%d,%d,%d,%d,%d,%d\r\n",
//		adc_value_struct.IN1_MIC1,adc_value_struct.IN5_MIC2,adc_value_struct.IN3_MIC3,
//		adc_value_struct.IN12_MIC4,adc_value_struct.IN4_MIC5,adc_value_struct.IN13_MIC6,adc_value_struct.IN2_MIC7);
		
		
//		sprintf(usb_buff + strlen((char *)usb_buff),"%d\r\n%d\r\n%d\r\n%d\r\n%d\r\n%d\r\n%d\r\n%d\r\n%d\r\n%d\r\n",
//			ADC5_value[0],
//			ADC5_value[1],
//			ADC5_value[2],
//			ADC5_value[3],
//			ADC5_value[4],
//			ADC5_value[5],
//			ADC5_value[6],
//			ADC5_value[7],
//			ADC5_value[8],
//			ADC5_value[9]
//		);
	
	
	
//	//t_send:8.0 us,t_send2:530.7 us,t_string:477.5 us
//	sprintf(usb_buff2,"t_send:%.1f us,t_send2:%.1f us,t_string:%.1f us,CDC_result:%u\r\n",t_send/10.0,t_send2/10.0,t_string/10.0,CDC_result);

//	t_string = LL_TIM_GetCounter(TIM7)-t_start;

//	CDC_Transmit_FS((uint8_t *)usb_buff,strlen(usb_buff));

//	t_send = LL_TIM_GetCounter(TIM7)-t_string;
//	do
//	{
//		CDC_result = CDC_Transmit_FS((uint8_t *)usb_buff2,strlen(usb_buff2));
//	}
//	while(CDC_result == USBD_BUSY);
//	t_send2 = LL_TIM_GetCounter(TIM7)-t_send;
*/
	
	
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
//		osDelay(100);
	  vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
  }
  /* USER CODE END LEDTask */
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


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	EventGroupHandle_t xEventGroup = (EventGroupHandle_t)adc_dma_EventHandle; // ת���������
	if(hadc == & hadc1)
	{
		xEventGroupSetBitsFromISR(
		  xEventGroup,          // �¼�������ת����
		  EVENT_ADC1_DMA_FULL,  // Ҫ���õ��¼���־
		  &xHigherPriorityTaskWoken  // ����������Ƿ���Ҫ�л�����
		);
	}
	else if(hadc == & hadc2)
	{
		xEventGroupSetBitsFromISR(
		  xEventGroup,          // �¼�������ת����
		  EVENT_ADC2_DMA_FULL,  // Ҫ���õ��¼���־
		  &xHigherPriorityTaskWoken  // ����������Ƿ���Ҫ�л�����
		);
	}
	else if(hadc == & hadc3)
	{
		xEventGroupSetBitsFromISR(
		  xEventGroup,          // �¼�������ת����
		  EVENT_ADC3_DMA_FULL,  // Ҫ���õ��¼���־
		  &xHigherPriorityTaskWoken  // ����������Ƿ���Ҫ�л�����
		);
	}
	else if(hadc == & hadc4)
	{
		xEventGroupSetBitsFromISR(
		  xEventGroup,          // �¼�������ת����
		  EVENT_ADC4_DMA_FULL,  // Ҫ���õ��¼���־
		  &xHigherPriorityTaskWoken  // ����������Ƿ���Ҫ�л�����
		);
	}
	else if(hadc == & hadc5)
	{
		xEventGroupSetBitsFromISR(
		  xEventGroup,          // �¼�������ת����
		  EVENT_ADC5_DMA_FULL,  // Ҫ���õ��¼���־
		  &xHigherPriorityTaskWoken  // ����������Ƿ���Ҫ�л�����
		);
	}
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	EventGroupHandle_t xEventGroup = (EventGroupHandle_t)adc_dma_EventHandle; // ת���������
	if(hadc == & hadc1)
	{
		xEventGroupSetBitsFromISR(
		  xEventGroup,          // �¼�������ת����
		  EVENT_ADC1_DMA_HALF,  // Ҫ���õ��¼���־
		  &xHigherPriorityTaskWoken  // ����������Ƿ���Ҫ�л�����
		);
	}
	else if(hadc == & hadc2)
	{
		xEventGroupSetBitsFromISR(
		  xEventGroup,          // �¼�������ת����
		  EVENT_ADC2_DMA_HALF,  // Ҫ���õ��¼���־
		  &xHigherPriorityTaskWoken  // ����������Ƿ���Ҫ�л�����
		);
	}
	else if(hadc == & hadc3)
	{
		xEventGroupSetBitsFromISR(
		  xEventGroup,          // �¼�������ת����
		  EVENT_ADC3_DMA_HALF,  // Ҫ���õ��¼���־
		  &xHigherPriorityTaskWoken  // ����������Ƿ���Ҫ�л�����
		);
	}
	else if(hadc == & hadc4)
	{
		xEventGroupSetBitsFromISR(
		  xEventGroup,          // �¼�������ת����
		  EVENT_ADC4_DMA_HALF,  // Ҫ���õ��¼���־
		  &xHigherPriorityTaskWoken  // ����������Ƿ���Ҫ�л�����
		);
	}
	else if(hadc == & hadc5)
	{
		xEventGroupSetBitsFromISR(
		  xEventGroup,          // �¼�������ת����
		  EVENT_ADC5_DMA_HALF,  // Ҫ���õ��¼���־
		  &xHigherPriorityTaskWoken  // ����������Ƿ���Ҫ�л�����
		);
	}
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
/* USER CODE END Application */

