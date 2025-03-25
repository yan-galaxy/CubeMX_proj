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
osThreadId usbTaskHandle;
osThreadId notifyTaskHandle;
osThreadId dataTaskHandle;
osMessageQId Line1QueueHandle;
osMessageQId Line2QueueHandle;
osMessageQId Line3QueueHandle;
osMessageQId Line4QueueHandle;
osMessageQId Line5QueueHandle;
osMessageQId Line6QueueHandle;
osMessageQId Line7QueueHandle;
osMessageQId Line8QueueHandle;
osMessageQId Line9QueueHandle;
osMessageQId Line10QueueHandle;
osMessageQId keyQueueHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void Start_TIM17_OnePulse(uint16_t ccr);
void AD5206_SetResistance(uint8_t index, uint8_t channel, uint8_t resistance);
void user_delaynus_tim(uint32_t nus);
void Select_switcher(uint8_t index);
void USART_SendString(char *str);
void exchange_res_p(void);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void USBTask(void const * argument);
void NotifyTask(void const * argument);
void DataTask(void const * argument);

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

  /* Create the queue(s) */
  /* definition and creation of Line1Queue */
  osMessageQDef(Line1Queue, 20, uint16_t);
  Line1QueueHandle = osMessageCreate(osMessageQ(Line1Queue), NULL);

  /* definition and creation of Line2Queue */
  osMessageQDef(Line2Queue, 20, uint16_t);
  Line2QueueHandle = osMessageCreate(osMessageQ(Line2Queue), NULL);

  /* definition and creation of Line3Queue */
  osMessageQDef(Line3Queue, 20, uint16_t);
  Line3QueueHandle = osMessageCreate(osMessageQ(Line3Queue), NULL);

  /* definition and creation of Line4Queue */
  osMessageQDef(Line4Queue, 20, uint16_t);
  Line4QueueHandle = osMessageCreate(osMessageQ(Line4Queue), NULL);

  /* definition and creation of Line5Queue */
  osMessageQDef(Line5Queue, 20, uint16_t);
  Line5QueueHandle = osMessageCreate(osMessageQ(Line5Queue), NULL);

  /* definition and creation of Line6Queue */
  osMessageQDef(Line6Queue, 20, uint16_t);
  Line6QueueHandle = osMessageCreate(osMessageQ(Line6Queue), NULL);

  /* definition and creation of Line7Queue */
  osMessageQDef(Line7Queue, 20, uint16_t);
  Line7QueueHandle = osMessageCreate(osMessageQ(Line7Queue), NULL);

  /* definition and creation of Line8Queue */
  osMessageQDef(Line8Queue, 20, uint16_t);
  Line8QueueHandle = osMessageCreate(osMessageQ(Line8Queue), NULL);

  /* definition and creation of Line9Queue */
  osMessageQDef(Line9Queue, 20, uint16_t);
  Line9QueueHandle = osMessageCreate(osMessageQ(Line9Queue), NULL);

  /* definition and creation of Line10Queue */
  osMessageQDef(Line10Queue, 20, uint16_t);
  Line10QueueHandle = osMessageCreate(osMessageQ(Line10Queue), NULL);

  /* definition and creation of keyQueue */
  osMessageQDef(keyQueue, 8, uint8_t);
  keyQueueHandle = osMessageCreate(osMessageQ(keyQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 512);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of usbTask */
  osThreadDef(usbTask, USBTask, osPriorityBelowNormal, 0, 512);
  usbTaskHandle = osThreadCreate(osThread(usbTask), NULL);

  /* definition and creation of notifyTask */
  osThreadDef(notifyTask, NotifyTask, osPriorityRealtime, 0, 256);
  notifyTaskHandle = osThreadCreate(osThread(notifyTask), NULL);

  /* definition and creation of dataTask */
  osThreadDef(dataTask, DataTask, osPriorityLow, 0, 256);
  dataTaskHandle = osThreadCreate(osThread(dataTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
volatile uint32_t TIM3_base_it_cnt;
volatile Tim_CaptureTypeDef Timer3_ch1_Cap;
volatile Tim_CaptureTypeDef Timer3_ch2_Cap;
volatile Tim_CaptureTypeDef Timer3_ch3_Cap;
volatile Tim_CaptureTypeDef Timer3_ch4_Cap;

volatile uint32_t TIM4_base_it_cnt;
volatile Tim_CaptureTypeDef Timer4_ch1_Cap;
volatile Tim_CaptureTypeDef Timer4_ch2_Cap;
volatile Tim_CaptureTypeDef Timer4_ch3_Cap;
volatile Tim_CaptureTypeDef Timer4_ch4_Cap;

volatile uint32_t TIM2_base_it_cnt;
volatile Tim_CaptureTypeDef Timer2_ch2_Cap;
volatile Tim_CaptureTypeDef Timer2_ch3_Cap;
volatile Tim_CaptureTypeDef Timer2_ch4_Cap;

Word_union test_union[5];
Word_union head_data[2];
Word_union res_value_buff[100][10];

Word_union test_data[1004];
Word_union res_data_1[1004];
Word_union res_data_2[1004];
Word_union* res_send_p=res_data_1;
Word_union* res_store_p=res_data_2;
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
	/* ���Բ��� */
	/* 
	ʹ��KEITHLEYֱ����Դ���ȶ�5V  ��Ƭ��3V3�����3V3����ͨ��5VҲ����ͨ
	����FreeRTOS
	���� LED�Ʋ���
	USB CDC����
	���ֵ�λ��AD5206���� SPI 	CS(NSS)�ûظ߲���Чִ������
	��·ѡ����MUX����  ����ߵ�ƽʱ���ref voltage ����͵�ƽ��ʱ�����GND
	MUX֮����˷�(��ѹ������) ������û���麸�����ս�MUX����ȫΪ�ߵ�ƽʱFPC�������ŵ�IN1~IN10�����ȫ��ref voltage
	�ö�ʱ��7����1us�жϽ���us����ʱ������50us���԰��������ӵ����벶�����ţ����Խ��յ��ļ�����ֵ
	
	����Ƚ���ģ��֮������봫���������·���߹ر����벶�񣬲�Ȼ��Ƭ���Ῠ��
	
	ԭTLV3502��֪ʲôԭ����Ϻ�Vramp���ȱ�С���Ȳ���
	TLV3201 �Ƚ�������˲��β���ԼΪ700ns(��1us��)
	COS3201 �Ƚ�������˲����޲��� ��ƽ�½�ʱ��ԼΪ70ns(��100ns��)	******
	
	������ֲSelect_switcher����������޸�main.h�궨��
	*/
	AD5206_SetResistance(0, 4, 50);//Vramp     ����50��Ӧ43us����ʱ��
	AD5206_SetResistance(0, 5, 4);//ref voltage     
	//21-1.009V    
	//8-0.500V  ��ʱ�˷ŷ��������������Ϊ30 (���200ŷķ����ʱ�ĵ͵�ƽʱ��Ϊ8.8us)
	//6-0.4039V ��ʱ�˷ŷ��������������Ϊ40 (���200ŷķ����ʱ�ĵ͵�ƽʱ��Ϊ8.9us)
	//4-0.2973V ��ʱ�˷ŷ��������������Ϊ58 (���200ŷķ����ʱ�ĵ͵�ƽʱ��Ϊ8.7us)	******
	//2-0.1822V ��ʱ�˷ŷ��������������Ϊ100(���200ŷķ����ʱ�ĵ͵�ƽʱ��Ϊ9.0us) �»�׼��ѹ�ṩ�ĵ�ѹ�������˷ŵ���̫��
	
	AD5206_SetResistance(0, 0, 58);//�˷�ͬ��Ŵ�������   12: 522��~524��
	AD5206_SetResistance(0, 1, 58);
	AD5206_SetResistance(0, 2, 58);
	AD5206_SetResistance(0, 3, 58);
	AD5206_SetResistance(1, 0, 58);
	AD5206_SetResistance(1, 1, 58);
	AD5206_SetResistance(1, 2, 58);
	AD5206_SetResistance(1, 3, 58);
	AD5206_SetResistance(1, 4, 58);
	AD5206_SetResistance(1, 5, 58);
	
	HAL_GPIO_WritePin(SEL1_GPIO_Port, SEL1_Pin ,1);
	HAL_GPIO_WritePin(SEL2_GPIO_Port, SEL2_Pin ,1);
	HAL_GPIO_WritePin(SEL3_GPIO_Port, SEL3_Pin ,1);//U2 ��·ѡ���������� ������·(*�ѽ��*)
	HAL_GPIO_WritePin(SEL4_GPIO_Port, SEL4_Pin ,1);
	HAL_GPIO_WritePin(SEL5_GPIO_Port, SEL5_Pin ,1);
	HAL_GPIO_WritePin(SEL6_GPIO_Port, SEL6_Pin ,1);
	HAL_GPIO_WritePin(SEL7_GPIO_Port, SEL7_Pin ,1);
	HAL_GPIO_WritePin(SEL8_GPIO_Port, SEL8_Pin ,1);
	HAL_GPIO_WritePin(SEL9_GPIO_Port, SEL9_Pin ,1);
	HAL_GPIO_WritePin(SEL10_GPIO_Port,SEL10_Pin,1);
	

	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	for(;;)
	{
//		//���� LED�Ʋ���
//		if(HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin))
//		{
//			vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
//			while(HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin))
//				vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));

//			osMessagePut(keyQueueHandle,1,0);//����з�����Ϣ,֪ͨusb���������ӡ�����¼�
//			
//			vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
//			HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);//������һ�η�תһ��LED
//			
//		}
//		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));//		osDelay(1);
		osDelay(10000);
	}
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_USBTask */
/**
* @brief Function implementing the usbTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_USBTask */
void USBTask(void const * argument)
{
  /* USER CODE BEGIN USBTask */
	uint8_t usb_TxBuf[32];//С�Ĳ�Ҫ��������ջ��С
	uint32_t usb_TxLength=0;
	uint8_t USBD_Result=0;
	
	uint8_t uart_TxBuf[128];//С�Ĳ�Ҫ��������ջ��С
	
	osEvent event[11];
	osEvent key_event;
	uint8_t i = 0;
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
  /* Infinite loop */
	for(;;)
	{
//		//��������������
//		key_event = osMessageGet(keyQueueHandle, 0);
//		if(key_event.status == osEventMessage)
//		{
//			sprintf((char *)uart_TxBuf,"The key is pressed.\r\n");
//			USART_SendString((char*)uart_TxBuf);//���ڷ���
//			xTaskNotifyGive(notifyTaskHandle);
//		}
//		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10000));
		
		/*	��������ͨ������
		0 : Timer3 ch1:PA6 Vcom_out9
		1 : Timer3 ch2:PA4 Vcom_out8
		2 : Timer3 ch3:PB0 Vcom_out4
		3 : Timer3 ch4:PB1 Vcom_out10
		4 : Timer4 ch1:PB6 Vcom_out1
		5 : Timer4 ch2:PB7 Vcom_out3
		6 : Timer4 ch4:PB9 Vcom_out2
		7 : Timer2 ch2:PA1 Vcom_out6
		8 : Timer2 ch3:PA2 Vcom_out7
		9 : Timer2 ch4:PA3 Vcom_out5
		*/
//		for(i=1;i<11;i++)
//		{
//			event[i] = osMessageGet(*(&Line1QueueHandle+i-1), 0);//osWaitForever
//			if (event[i].status == osEventMessage)
//			{
//				sprintf((char *)uart_TxBuf,"event[%2d].value.v:%.3fus,%d\r\n",i,(event[i].value.v*5)/1000.0,event[i].value.v);
//				USART_SendString((char*)uart_TxBuf);//���ڷ���
//			}
//		}
	}
  /* USER CODE END USBTask */
}

/* USER CODE BEGIN Header_NotifyTask */
/**
* @brief Function implementing the notifyTask thread.
* @param argument: Not used
* @retval None
*/

//��ѹ�ɼ����֮����Ҫ���л����л�
void exchange_res_p(void)
{
	static uint8_t state=1;//��һ�����иú���ǰ �ɼ������ݴ浽res_data_1   ��һ�����иú�����״̬��Ϊ0�����͵�ָ��ָ��res_data_2  ֮��ѭ������
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
/* USER CODE END Header_NotifyTask */
void NotifyTask(void const * argument)
{
  /* USER CODE BEGIN NotifyTask */
	uint32_t ulNotificationValue=0;
	osDelay(20);
	uint8_t USBD_Result=0;
	
	test_data[0].byte[0]=0x55;//֡ͷ
	test_data[0].byte[1]=0xAA;
	test_data[1].byte[0]=0xBB;
	test_data[1].byte[1]=0xCC;
	for(uint16_t i=0;i<1000;i++)
	{
		test_data[2+i].word16=i*2+2;
	}
	test_data[1002].byte[0]=0xAA;//֡β
	test_data[1002].byte[1]=0x55;
	test_data[1003].byte[0]=0x66;
	test_data[1003].byte[1]=0x77;
	
	res_data_1[0].byte[0]=0x55;//֡ͷ
	res_data_1[0].byte[1]=0xAA;
	res_data_1[1].byte[0]=0xBB;
	res_data_1[1].byte[1]=0xCC;
	for(uint16_t i=0;i<1000;i++)
	{
//		res_send_p[2+i].word16=i*1+1;
		res_data_1[2+i].word16=i*1+1;
	}
	res_data_1[1002].byte[0]=0xAA;//֡β
	res_data_1[1002].byte[1]=0x55;
	res_data_1[1003].byte[0]=0x66;
	res_data_1[1003].byte[1]=0x77;
	
	res_data_2[0].byte[0]=0x55;//֡ͷ
	res_data_2[0].byte[1]=0xAA;
	res_data_2[1].byte[0]=0xBB;
	res_data_2[1].byte[1]=0xCC;
	for(uint16_t i=0;i<1000;i++)
	{
		res_data_2[2+i].word16=i*2+2;
	}
	res_data_2[1002].byte[0]=0xAA;//֡β
	res_data_2[1002].byte[1]=0x55;
	res_data_2[1003].byte[0]=0x66;
	res_data_2[1003].byte[1]=0x77;
  /* Infinite loop */
	for(;;)
	{
////		exchange_res_p();
//		ulNotificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // wait for notification value
////		do{
//			USBD_Result=CDC_Transmit_FS(res_send_p->byte, 2008);
////		}while(USBD_Result!=USBD_OK);//�������� ������������ͺ���ֻ��Ҫ8us��һ��������2008�ֽڷ��������Ҫ2.3ms(433.7֡/��)
		
		osDelay(10000);
	}
  /* USER CODE END NotifyTask */
}

/* USER CODE BEGIN Header_DataTask */
/**
* @brief Function implementing the dataTask thread.
* @param argument: Not used
* @retval None
*/
uint8_t res_flags[10]={0};
uint8_t TIM16_start_time_order;//��ʱ��7����������1us����һ��
uint32_t TIM16_time_cnt;
uint32_t sample_error_cnt;
/* USER CODE END Header_DataTask */
void DataTask(void const * argument)
{
  /* USER CODE BEGIN DataTask */
	uint32_t receivedMessage=0;
	osEvent event[10];
	uint8_t i=0,j=0;
	
	uint8_t Notify_send_flag=0;
	osDelay(200);
	
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
  /* Infinite loop */
	for(;;)
	{
		/*
		us����ʱ��GPIO����40us����
		����
		*/
//		for(uint8_t k=1;k<11;k++)
//		{
//			Select_switcher(k);
//			user_delaynus_tim(10);
//			Start_TIM17_OnePulse(4300);//���Ծ�ݲ�����
//			//user_delaynus_tim(100);
//			vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));//osDelay(1);
//		}
		
		
		for(j=0;j<100;j++)
		{
			Select_switcher(j%10+1);
//			user_delaynus_tim(5);
			Start_TIM17_OnePulse(4300);
//			if(!Notify_send_flag)//��һ�δ���ɨ�貨�κ���֪ͨ ���ݷ���ָ���ִ��ʱ��С��һ��ɨ��
//			{
//				xTaskNotifyGive(notifyTaskHandle);//����֪ͨ��usbͨ������  ������ǵ÷���ll_Start_TIM17_OnePulse֮�󣬽�Լ����ʱ�䣬�ڱȽ����Ѿ���ʼ�ɼ��µ�һ֡ʱ����һ֡���ͳ�ȥ
//				taskYIELD();// �ֶ������������л�
//				Notify_send_flag = 1;
//			}
			for(i=0;i<10;i++)res_flags[i]=0;//�������/�о�����־
//			user_delaynus_tim(30);
			TIM16_time_cnt=0;
			TIM16_start_time_order=1;
			while(res_flags[0]+res_flags[1]+res_flags[2]+res_flags[3]+res_flags[4]+res_flags[5]+res_flags[6]+res_flags[7]+res_flags[8]+res_flags[9]!=10)
			{
				for(i=0;i<10;i++)
				{
					event[i] = osMessageGet(*(&Line1QueueHandle+i), 0);//osWaitForever
					if (event[i].status == osEventMessage)
					{
////						res_store_p[j*10+i+2].word16=event[i+1].value.v;
//						
//						
////						if(res_store_p==res_data_1)//�����Ƿ���ݻ����л�����ȷ��������
////							res_store_p[j*10+i+2].word16=(j*10+i)*5;
////						if(res_store_p==res_data_2)
////							res_store_p[j*10+i+2].word16=(j*10+i)*5+1;
//						
						res_flags[i]=1;
					}
				}
				if(TIM16_time_cnt>46)
				{
					sample_error_cnt++;
					TIM16_start_time_order=0;
					j--;
					break;
				}
			}
			
		}
		//ȫ��10֡���ݻ������
		exchange_res_p();//�л�������
//		xTaskNotifyGive(notifyTaskHandle);//����֪ͨ��usbͨ������  ������ǵ÷���ll_Start_TIM17_OnePulse֮�󣬽�Լ����ʱ�䣬�ڱȽ����Ѿ���ʼ�ɼ��µ�һ֡ʱ����һ֡���ͳ�ȥ
//		taskYIELD();// �ֶ������������л�
//		CDC_Transmit_FS(res_send_p->byte, 2008);
		
		Notify_send_flag = 0;
		
	}
  /* USER CODE END DataTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void TIM7_IRQHandler(void)
{
	static uint16_t time_cnt=0;
	if (LL_TIM_IsActiveFlag_UPDATE(TIM7))  //check IRQ flag
	{
		LL_TIM_ClearFlag_UPDATE(TIM7);  //clear IRQ flag
		
//		time_cnt++;//ԭ����1us�ж�����ʱ���ֲ���
		
		
	}
}
void TIM1_UP_TIM16_IRQHandler(void)
{
//	static uint32_t time_cnt=0;
	if (LL_TIM_IsActiveFlag_UPDATE(TIM16))  //check IRQ flag
	{
		LL_TIM_ClearFlag_UPDATE(TIM16);  //clear IRQ flag
		
//		time_cnt++;//��ʱ
		
		if(TIM16_start_time_order)TIM16_time_cnt++;
//		if(time_cnt>=100)
//		{
//			time_cnt=0;
////			Start_TIM17_OnePulse(4300);//���Ծ�ݲ�����
//		}

		
	}
}
void TIM3_IRQHandler(void)
{
	if (LL_TIM_IsActiveFlag_UPDATE(TIM3))  //check IRQ flag
	{
		LL_TIM_ClearFlag_UPDATE(TIM3);  //clear IRQ flag
		TIM3_base_it_cnt++;
//		Timer3_ch1_Cap.OverflowCount++;//���������Ī�����һ������������ܴ�ֱ�Ӳ������������������͵�ƽ�ź�ʱ����һ��65536���������ڼ���
//		Timer3_ch2_Cap.OverflowCount++;
//		Timer3_ch3_Cap.OverflowCount++;
//		Timer3_ch4_Cap.OverflowCount++;
	}
	if(LL_TIM_IsActiveFlag_CC1(TIM3))
	{
		LL_TIM_ClearFlag_CC1(TIM3);	//clear CC1 IRQ flag
		
		switch(Timer3_ch1_Cap.capture_Cnt){
			case 0:
				Timer3_ch1_Cap.capture_Buf[0] = LL_TIM_IC_GetCaptureCH1(TIM3);//��ȡ��ǰ�Ĳ���ֵ.
				Timer3_ch1_Cap.OverflowCount=0;
				LL_TIM_IC_SetPolarity(TIM3,LL_TIM_CHANNEL_CH1,LL_TIM_IC_POLARITY_RISING);  //����Ϊ�����ز���
				Timer3_ch1_Cap.capture_Cnt=1;
				break;
			case 1:
				Timer3_ch1_Cap.capture_Buf[1] = LL_TIM_IC_GetCaptureCH1(TIM3);//��ȡ��ǰ�Ĳ���ֵ.
//				Timer3_ch1_Cap.low_time = Timer3_ch1_Cap.capture_Buf[1] - Timer3_ch1_Cap.capture_Buf[0] + ( Timer3_ch1_Cap.OverflowCount << 16 );    //�͵�ƽʱ��
				Timer3_ch1_Cap.low_time = Timer3_ch1_Cap.capture_Buf[1] - Timer3_ch1_Cap.capture_Buf[0];    //�͵�ƽʱ��
				if(Timer3_ch1_Cap.low_time < 0 )Timer3_ch1_Cap.low_time += 0xFFFF;
			
				LL_TIM_IC_SetPolarity(TIM3,LL_TIM_CHANNEL_CH1,LL_TIM_IC_POLARITY_FALLING);  //����Ϊ�½��ز���
				Timer3_ch1_Cap.capture_Cnt=0;
			
				osMessagePut(Line9QueueHandle,(uint16_t)Timer3_ch1_Cap.low_time,0);
//				xReturn[0].status=osMessagePut(tim3ch1QueueHandle,(uint16_t)Timer3_ch1_Cap.low_time,0);
//				//	if(osOK!=xReturn.status)
//				//	{
//				//		while(1);
//				////		printf("send fail\n");
//				//	}
			
//			BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//    uint32_t message = Timer3_ch1_Cap.low_time;  // ��ȡ��ʱ��ֵ��Ϊ��Ϣ

//    // ���ж�����з�����Ϣ
//    xQueueSendFromISR(Line1QueueHandle, &message, &xHigherPriorityTaskWoken);

//    // �����Ҫ�������������л�
//    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
	}
	if(LL_TIM_IsActiveFlag_CC2(TIM3))
	{
		LL_TIM_ClearFlag_CC2(TIM3);	//clear CC2 IRQ flag
		
		switch(Timer3_ch2_Cap.capture_Cnt){
			case 0:
				Timer3_ch2_Cap.capture_Buf[0] = LL_TIM_IC_GetCaptureCH2(TIM3);//��ȡ��ǰ�Ĳ���ֵ.
				Timer3_ch2_Cap.OverflowCount=0;
				LL_TIM_IC_SetPolarity(TIM3,LL_TIM_CHANNEL_CH2,LL_TIM_IC_POLARITY_RISING);  //����Ϊ�����ز���
				Timer3_ch2_Cap.capture_Cnt=1;
				break;
			case 1:
				Timer3_ch2_Cap.capture_Buf[1] = LL_TIM_IC_GetCaptureCH2(TIM3);//��ȡ��ǰ�Ĳ���ֵ.
//				Timer3_ch2_Cap.low_time = Timer3_ch2_Cap.capture_Buf[1] - Timer3_ch2_Cap.capture_Buf[0] + ( Timer3_ch2_Cap.OverflowCount << 16 );    //�͵�ƽʱ��
				Timer3_ch2_Cap.low_time = Timer3_ch2_Cap.capture_Buf[1] - Timer3_ch2_Cap.capture_Buf[0];    //�͵�ƽʱ��
				if(Timer3_ch2_Cap.low_time < 0 )Timer3_ch2_Cap.low_time += 0xFFFF;
			
				LL_TIM_IC_SetPolarity(TIM3,LL_TIM_CHANNEL_CH2,LL_TIM_IC_POLARITY_FALLING);  //����Ϊ�½��ز���
				Timer3_ch2_Cap.capture_Cnt=0;
			
				osMessagePut(Line8QueueHandle,(uint16_t)Timer3_ch2_Cap.low_time,0);
		}
	}
	if(LL_TIM_IsActiveFlag_CC3(TIM3))
	{
		LL_TIM_ClearFlag_CC3(TIM3);	//clear CC3 IRQ flag
		
		switch(Timer3_ch3_Cap.capture_Cnt){
			case 0:
				Timer3_ch3_Cap.capture_Buf[0] = LL_TIM_IC_GetCaptureCH3(TIM3);//��ȡ��ǰ�Ĳ���ֵ.
				Timer3_ch3_Cap.OverflowCount=0;
				LL_TIM_IC_SetPolarity(TIM3,LL_TIM_CHANNEL_CH3,LL_TIM_IC_POLARITY_RISING);  //����Ϊ�����ز���
				Timer3_ch3_Cap.capture_Cnt=1;
				break;
			case 1:
				Timer3_ch3_Cap.capture_Buf[1] = LL_TIM_IC_GetCaptureCH3(TIM3);//��ȡ��ǰ�Ĳ���ֵ.
//				Timer3_ch3_Cap.low_time = Timer3_ch3_Cap.capture_Buf[1] - Timer3_ch3_Cap.capture_Buf[0] + ( Timer3_ch3_Cap.OverflowCount << 16 );    //�͵�ƽʱ��
				Timer3_ch3_Cap.low_time = Timer3_ch3_Cap.capture_Buf[1] - Timer3_ch3_Cap.capture_Buf[0];    //�͵�ƽʱ��
				if(Timer3_ch3_Cap.low_time < 0 )Timer3_ch3_Cap.low_time += 0xFFFF;
			
				LL_TIM_IC_SetPolarity(TIM3,LL_TIM_CHANNEL_CH3,LL_TIM_IC_POLARITY_FALLING);  //����Ϊ�½��ز���
				Timer3_ch3_Cap.capture_Cnt=0;
			
				osMessagePut(Line4QueueHandle,(uint16_t)Timer3_ch3_Cap.low_time,0);
		}
	}
	if(LL_TIM_IsActiveFlag_CC4(TIM3))
	{
		LL_TIM_ClearFlag_CC4(TIM3);	//clear CC4 IRQ flag
		
		switch(Timer3_ch4_Cap.capture_Cnt){
			case 0:
				Timer3_ch4_Cap.capture_Buf[0] = LL_TIM_IC_GetCaptureCH4(TIM3);//��ȡ��ǰ�Ĳ���ֵ.
				Timer3_ch4_Cap.OverflowCount=0;
				LL_TIM_IC_SetPolarity(TIM3,LL_TIM_CHANNEL_CH4,LL_TIM_IC_POLARITY_RISING);  //����Ϊ�����ز���
				Timer3_ch4_Cap.capture_Cnt=1;
				break;
			case 1:
				Timer3_ch4_Cap.capture_Buf[1] = LL_TIM_IC_GetCaptureCH4(TIM3);//��ȡ��ǰ�Ĳ���ֵ.
//				Timer3_ch4_Cap.low_time = Timer3_ch4_Cap.capture_Buf[1] - Timer3_ch4_Cap.capture_Buf[0] + ( Timer3_ch4_Cap.OverflowCount << 16 );    //�͵�ƽʱ��
				Timer3_ch4_Cap.low_time = Timer3_ch4_Cap.capture_Buf[1] - Timer3_ch4_Cap.capture_Buf[0];    //�͵�ƽʱ��
				if(Timer3_ch4_Cap.low_time < 0 )Timer3_ch4_Cap.low_time += 0xFFFF;
			
				LL_TIM_IC_SetPolarity(TIM3,LL_TIM_CHANNEL_CH4,LL_TIM_IC_POLARITY_FALLING);  //����Ϊ�½��ز���
				Timer3_ch4_Cap.capture_Cnt=0;
			
				osMessagePut(Line10QueueHandle,(uint16_t)Timer3_ch4_Cap.low_time,0);
		}
	}
	
	
}

void TIM4_IRQHandler(void)
{
	if (LL_TIM_IsActiveFlag_UPDATE(TIM4))  //check IRQ flag
	{
		LL_TIM_ClearFlag_UPDATE(TIM4);  //clear IRQ flag
		TIM4_base_it_cnt++;
//		Timer4_ch1_Cap.OverflowCount++;
//		Timer4_ch2_Cap.OverflowCount++;
//		Timer4_ch4_Cap.OverflowCount++;
	}
	if(LL_TIM_IsActiveFlag_CC1(TIM4))
	{
		LL_TIM_ClearFlag_CC1(TIM4);	//clear CC1 IRQ flag
		
		switch(Timer4_ch1_Cap.capture_Cnt){
			case 0:
				Timer4_ch1_Cap.capture_Buf[0] = LL_TIM_IC_GetCaptureCH1(TIM4);//��ȡ��ǰ�Ĳ���ֵ.
				Timer4_ch1_Cap.OverflowCount=0;
				LL_TIM_IC_SetPolarity(TIM4,LL_TIM_CHANNEL_CH1,LL_TIM_IC_POLARITY_RISING);  //����Ϊ�����ز���
				Timer4_ch1_Cap.capture_Cnt=1;
				break;
			case 1:
				Timer4_ch1_Cap.capture_Buf[1] = LL_TIM_IC_GetCaptureCH1(TIM4);//��ȡ��ǰ�Ĳ���ֵ.
//				Timer4_ch1_Cap.low_time = Timer4_ch1_Cap.capture_Buf[1] - Timer4_ch1_Cap.capture_Buf[0] + ( Timer4_ch1_Cap.OverflowCount << 16 );    //�͵�ƽʱ��
				Timer4_ch1_Cap.low_time = Timer4_ch1_Cap.capture_Buf[1] - Timer4_ch1_Cap.capture_Buf[0];    //�͵�ƽʱ��
				if(Timer4_ch1_Cap.low_time < 0 )Timer4_ch1_Cap.low_time += 0xFFFF;
			
				LL_TIM_IC_SetPolarity(TIM4,LL_TIM_CHANNEL_CH1,LL_TIM_IC_POLARITY_FALLING);  //����Ϊ�½��ز���
				Timer4_ch1_Cap.capture_Cnt=0;
			
				osMessagePut(Line1QueueHandle,(uint16_t)Timer4_ch1_Cap.low_time,0);
		}
	}
	if(LL_TIM_IsActiveFlag_CC2(TIM4))
	{
		LL_TIM_ClearFlag_CC2(TIM4);	//clear CC2 IRQ flag
		
		switch(Timer4_ch2_Cap.capture_Cnt){
			case 0:
				Timer4_ch2_Cap.capture_Buf[0] = LL_TIM_IC_GetCaptureCH2(TIM4);//��ȡ��ǰ�Ĳ���ֵ.
				Timer4_ch2_Cap.OverflowCount=0;
				LL_TIM_IC_SetPolarity(TIM4,LL_TIM_CHANNEL_CH2,LL_TIM_IC_POLARITY_RISING);  //����Ϊ�����ز���
				Timer4_ch2_Cap.capture_Cnt=1;
				break;
			case 1:
				Timer4_ch2_Cap.capture_Buf[1] = LL_TIM_IC_GetCaptureCH2(TIM4);//��ȡ��ǰ�Ĳ���ֵ.
//				Timer4_ch2_Cap.low_time = Timer4_ch2_Cap.capture_Buf[1] - Timer4_ch2_Cap.capture_Buf[0] + ( Timer4_ch2_Cap.OverflowCount << 16 );    //�͵�ƽʱ��
				Timer4_ch2_Cap.low_time = Timer4_ch2_Cap.capture_Buf[1] - Timer4_ch2_Cap.capture_Buf[0];    //�͵�ƽʱ��
				if(Timer4_ch2_Cap.low_time < 0 )Timer4_ch2_Cap.low_time += 0xFFFF;
			
				LL_TIM_IC_SetPolarity(TIM4,LL_TIM_CHANNEL_CH2,LL_TIM_IC_POLARITY_FALLING);  //����Ϊ�½��ز���
				Timer4_ch2_Cap.capture_Cnt=0;
			
				osMessagePut(Line3QueueHandle,(uint16_t)Timer4_ch2_Cap.low_time,0);
		}
	}
	if(LL_TIM_IsActiveFlag_CC4(TIM4))
	{
		LL_TIM_ClearFlag_CC4(TIM4);	//clear CC4 IRQ flag
		
		switch(Timer4_ch4_Cap.capture_Cnt){
			case 0:
				Timer4_ch4_Cap.capture_Buf[0] = LL_TIM_IC_GetCaptureCH4(TIM4);//��ȡ��ǰ�Ĳ���ֵ.
				Timer4_ch4_Cap.OverflowCount=0;
				LL_TIM_IC_SetPolarity(TIM4,LL_TIM_CHANNEL_CH4,LL_TIM_IC_POLARITY_RISING);  //����Ϊ�����ز���
				Timer4_ch4_Cap.capture_Cnt=1;
				break;
			case 1:
				Timer4_ch4_Cap.capture_Buf[1] = LL_TIM_IC_GetCaptureCH4(TIM4);//��ȡ��ǰ�Ĳ���ֵ.
				Timer4_ch4_Cap.low_time = Timer4_ch4_Cap.capture_Buf[1] - Timer4_ch4_Cap.capture_Buf[0] + ( Timer4_ch4_Cap.OverflowCount << 16 );    //�͵�ƽʱ��
				Timer4_ch4_Cap.low_time = Timer4_ch4_Cap.capture_Buf[1] - Timer4_ch4_Cap.capture_Buf[0];    //�͵�ƽʱ��
				if(Timer4_ch4_Cap.low_time < 0 )Timer4_ch4_Cap.low_time += 0xFFFF;
			
				LL_TIM_IC_SetPolarity(TIM4,LL_TIM_CHANNEL_CH4,LL_TIM_IC_POLARITY_FALLING);  //����Ϊ�½��ز���
				Timer4_ch4_Cap.capture_Cnt=0;
			
				osMessagePut(Line2QueueHandle,(uint16_t)Timer4_ch4_Cap.low_time,0);
		}
	}
}

void TIM2_IRQHandler(void)
{
	if (LL_TIM_IsActiveFlag_UPDATE(TIM2))  //check IRQ flag
	{
		LL_TIM_ClearFlag_UPDATE(TIM2);  //clear IRQ flag
		TIM2_base_it_cnt++;
//		Timer2_ch2_Cap.OverflowCount++;
//		Timer2_ch3_Cap.OverflowCount++;
//		Timer2_ch4_Cap.OverflowCount++;
	}
	if(LL_TIM_IsActiveFlag_CC2(TIM2))
	{
		LL_TIM_ClearFlag_CC2(TIM2);	//clear CC2 IRQ flag
		
		switch(Timer2_ch2_Cap.capture_Cnt){
			case 0:
				Timer2_ch2_Cap.capture_Buf[0] = LL_TIM_IC_GetCaptureCH2(TIM2);//��ȡ��ǰ�Ĳ���ֵ.
				Timer2_ch2_Cap.OverflowCount=0;
				LL_TIM_IC_SetPolarity(TIM2,LL_TIM_CHANNEL_CH2,LL_TIM_IC_POLARITY_RISING);  //����Ϊ�����ز���
				Timer2_ch2_Cap.capture_Cnt=1;
				break;
			case 1:
				Timer2_ch2_Cap.capture_Buf[1] = LL_TIM_IC_GetCaptureCH2(TIM2);//��ȡ��ǰ�Ĳ���ֵ.
//				Timer2_ch2_Cap.low_time = Timer2_ch2_Cap.capture_Buf[1] - Timer2_ch2_Cap.capture_Buf[0] + ( Timer2_ch2_Cap.OverflowCount << 16 );    //�͵�ƽʱ��
				Timer2_ch2_Cap.low_time = Timer2_ch2_Cap.capture_Buf[1] - Timer2_ch2_Cap.capture_Buf[0];    //�͵�ƽʱ��
				if(Timer2_ch2_Cap.low_time < 0 )Timer2_ch2_Cap.low_time += 0xFFFFFFFF;
			
				LL_TIM_IC_SetPolarity(TIM2,LL_TIM_CHANNEL_CH2,LL_TIM_IC_POLARITY_FALLING);  //����Ϊ�½��ز���
				Timer2_ch2_Cap.capture_Cnt=0;
			
				osMessagePut(Line6QueueHandle,(uint16_t)Timer2_ch2_Cap.low_time,0);
		}
	}
	if(LL_TIM_IsActiveFlag_CC3(TIM2))
	{
		LL_TIM_ClearFlag_CC3(TIM2);	//clear CC3 IRQ flag
		
		switch(Timer2_ch3_Cap.capture_Cnt){
			case 0:
				Timer2_ch3_Cap.capture_Buf[0] = LL_TIM_IC_GetCaptureCH3(TIM2);//��ȡ��ǰ�Ĳ���ֵ.
				Timer2_ch3_Cap.OverflowCount=0;
				LL_TIM_IC_SetPolarity(TIM2,LL_TIM_CHANNEL_CH3,LL_TIM_IC_POLARITY_RISING);  //����Ϊ�����ز���
				Timer2_ch3_Cap.capture_Cnt=1;
				break;
			case 1:
				Timer2_ch3_Cap.capture_Buf[1] = LL_TIM_IC_GetCaptureCH3(TIM2);//��ȡ��ǰ�Ĳ���ֵ.
				Timer2_ch3_Cap.low_time = Timer2_ch3_Cap.capture_Buf[1] - Timer2_ch3_Cap.capture_Buf[0] + ( Timer2_ch3_Cap.OverflowCount << 16 );    //�͵�ƽʱ��
				Timer2_ch3_Cap.low_time = Timer2_ch3_Cap.capture_Buf[1] - Timer2_ch3_Cap.capture_Buf[0];    //�͵�ƽʱ��
				if(Timer2_ch3_Cap.low_time < 0 )Timer2_ch3_Cap.low_time += 0xFFFFFFFF;
			
				LL_TIM_IC_SetPolarity(TIM2,LL_TIM_CHANNEL_CH3,LL_TIM_IC_POLARITY_FALLING);  //����Ϊ�½��ز���
				Timer2_ch3_Cap.capture_Cnt=0;
			
				osMessagePut(Line7QueueHandle,(uint16_t)Timer2_ch3_Cap.low_time,0);
		}
	}
	if(LL_TIM_IsActiveFlag_CC4(TIM2))
	{
		LL_TIM_ClearFlag_CC4(TIM2);	//clear CC4 IRQ flag
		
		switch(Timer2_ch4_Cap.capture_Cnt){
			case 0:
				Timer2_ch4_Cap.capture_Buf[0] = LL_TIM_IC_GetCaptureCH4(TIM2);//��ȡ��ǰ�Ĳ���ֵ.
				Timer2_ch4_Cap.OverflowCount=0;
				LL_TIM_IC_SetPolarity(TIM2,LL_TIM_CHANNEL_CH4,LL_TIM_IC_POLARITY_RISING);  //����Ϊ�����ز���
				Timer2_ch4_Cap.capture_Cnt=1;
				break;
			case 1:
				Timer2_ch4_Cap.capture_Buf[1] = LL_TIM_IC_GetCaptureCH4(TIM2);//��ȡ��ǰ�Ĳ���ֵ.
				Timer2_ch4_Cap.low_time = Timer2_ch4_Cap.capture_Buf[1] - Timer2_ch4_Cap.capture_Buf[0] + ( Timer2_ch4_Cap.OverflowCount << 16 );    //�͵�ƽʱ��
				Timer2_ch4_Cap.low_time = Timer2_ch4_Cap.capture_Buf[1] - Timer2_ch4_Cap.capture_Buf[0];    //�͵�ƽʱ��
				if(Timer2_ch4_Cap.low_time < 0 )Timer2_ch4_Cap.low_time += 0xFFFFFFFF;
			
				LL_TIM_IC_SetPolarity(TIM2,LL_TIM_CHANNEL_CH4,LL_TIM_IC_POLARITY_FALLING);  //����Ϊ�½��ز���
				Timer2_ch4_Cap.capture_Cnt=0;
			
				osMessagePut(Line5QueueHandle,(uint16_t)Timer2_ch4_Cap.low_time,0);
		}
	}
	
	
}



/*
	����ʱ�ߵ�ƽ��ccr���õ͵�ƽʱ��
	ccr: 0~4499 ��λ��0.01us(10ns)	�����45us
	4450:44.5us
	4300:43us
*/
void Start_TIM17_OnePulse(uint16_t ccr)
{	
	TIM17->CCR1 = 4499-ccr;//ccr:4450
	// ���� TIM17 CH1 ����Ƚ�ͨ��
	TIM17->CCER |= TIM_CCER_CC1E;  // ���� CC1E (Channel 1 Output Enable)
	// ���� TIM17 CH1 ����Ƚ�Ԥװ�ع���
	TIM17->CCMR1 |= TIM_CCMR1_OC1PE;  // ���� OC1PE (Output Compare Preload Enable)
	// ������ʱ�� TIM17
	TIM17->CR1 |= TIM_CR1_CEN;  // ���� CEN (Counter Enable) λ
	// ���� TIM17 ������ź�
	TIM17->BDTR |= TIM_BDTR_MOE;  // ���� MOE (Main Output Enable) λ

}
// ����AD5206�ĵ���ֵ
// index: 0 1
// channel: 0~5
// resistance: 0~255
void AD5206_SetResistance(uint8_t index, uint8_t channel, uint8_t resistance) {
		uint8_t spi_data[2]={0x00,10};
		
    // ȷ��ͨ���͵���ֵ����Ч��Χ��
    if (channel > 5) return; // AD5206��6��ͨ����ͨ���Ŵ�0��5
    if (resistance > 255) return; // ����ֵӦ��0��255֮��
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

void user_delaynus_tim(uint32_t nus)
{
    uint16_t differ = 0xFFFF - nus - 5;

    // Set the timer counter value
    LL_TIM_SetCounter(TIM7, differ);

    // Enable the timer
    LL_TIM_EnableCounter(TIM7);

    // Wait until the timer reaches the target value
    while (LL_TIM_GetCounter(TIM7) < 0xFFFF - 5)
    {
        // Optionally, add a timeout condition here to avoid an infinite loop
    }

    // Disable the timer
    LL_TIM_DisableCounter(TIM7);

}

//�л���ѡ  ����һ���е������øߵ�ƽ ��ǰ���õ͵�ƽ
/*
index : 1~10
*/
void Select_switcher(uint8_t index)
{
	switch(index)
	{
		case 1:
			SEL10_GPIO_Port->BSRR=SEL10_pin_HIGH;//�ߵ�ƽ
			SEL1_GPIO_Port->BSRR=SEL1_pin_LOW;//�͵�ƽ
			break;
		case 2:
			SEL1_GPIO_Port->BSRR=SEL1_pin_HIGH;//�ߵ�ƽ
			SEL2_GPIO_Port->BSRR=SEL2_pin_LOW;//�͵�ƽ
			break;
		case 3:
			SEL2_GPIO_Port->BSRR=SEL2_pin_HIGH;//�ߵ�ƽ
			SEL3_GPIO_Port->BSRR=SEL3_pin_LOW;//�͵�ƽ
			break;
		case 4:
			SEL3_GPIO_Port->BSRR=SEL3_pin_HIGH;//�ߵ�ƽ
			SEL4_GPIO_Port->BSRR=SEL4_pin_LOW;//�͵�ƽ
			break;
		case 5:
			SEL4_GPIO_Port->BSRR=SEL4_pin_HIGH;//�ߵ�ƽ
			SEL5_GPIO_Port->BSRR=SEL5_pin_LOW;//�͵�ƽ
			break;
		case 6:
			SEL5_GPIO_Port->BSRR=SEL5_pin_HIGH;//�ߵ�ƽ
			SEL6_GPIO_Port->BSRR=SEL6_pin_LOW;//�͵�ƽ
			break;
		case 7:
			SEL6_GPIO_Port->BSRR=SEL6_pin_HIGH;//�ߵ�ƽ
			SEL7_GPIO_Port->BSRR=SEL7_pin_LOW;//�͵�ƽ
			break;
		case 8:
			SEL7_GPIO_Port->BSRR=SEL7_pin_HIGH;//�ߵ�ƽ
			SEL8_GPIO_Port->BSRR=SEL8_pin_LOW;//�͵�ƽ
			break;
		case 9:
			SEL8_GPIO_Port->BSRR=SEL8_pin_HIGH;//�ߵ�ƽ
			SEL9_GPIO_Port->BSRR=SEL9_pin_LOW;//�͵�ƽ
			break;
		case 10:
			SEL9_GPIO_Port->BSRR=SEL9_pin_HIGH;//�ߵ�ƽ
			SEL10_GPIO_Port->BSRR=SEL10_pin_LOW;//�͵�ƽ
			break;
	}
}
// �����ַ���
void USART_SendString(char *str) {
    HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
}
/* USER CODE END Application */

