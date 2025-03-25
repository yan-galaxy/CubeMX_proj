/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
osSemaphoreId usbBinarySemHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void user_delaynus_tim(uint32_t nus)
{
    uint16_t differ = 0xFFFF - nus - 5;

    // Enable TIM7 peripheral clock
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM7);

    // Set the timer counter value
    LL_TIM_SetCounter(TIM7, differ);

    // Configure the timer prescaler if needed (assuming default prescaler 0)
    // LL_TIM_SetPrescaler(TIM7, prescaler_value);

    // Enable the timer
    LL_TIM_EnableCounter(TIM7);

    // Wait until the timer reaches the target value
    while (LL_TIM_GetCounter(TIM7) < 0xFFFF - 5)
    {
        // Optionally, add a timeout condition here to avoid an infinite loop
    }

    // Disable the timer
    LL_TIM_DisableCounter(TIM7);

    // Optionally, disable the TIM7 peripheral clock when done (if no longer needed)
    // LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_TIM7);
}
void ll_Start_TIM17_OnePulse(uint16_t ccr)
{
//	LL_TIM_CC_EnableChannel(TIM17, LL_TIM_CHANNEL_CH1);  // 启用 CH1 输出比较
//	LL_TIM_OC_EnablePreload(TIM17, LL_TIM_CHANNEL_CH1);  // 启用预装载
//	LL_TIM_EnableCounter(TIM17);  // 启动计数器
//	LL_TIM_EnableAllOutputs(TIM17);  // 启用主输出信号   OK可用
	
	TIM17->CCR1 = 4999-ccr;
	// 启用 TIM17 CH1 输出比较通道
	TIM17->CCER |= TIM_CCER_CC1E;  // 设置 CC1E (Channel 1 Output Enable)
	// 启用 TIM17 CH1 输出比较预装载功能
	TIM17->CCMR1 |= TIM_CCMR1_OC1PE;  // 设置 OC1PE (Output Compare Preload Enable)
	// 启动定时器 TIM17
	TIM17->CR1 |= TIM_CR1_CEN;  // 设置 CEN (Counter Enable) 位
	// 启用 TIM17 主输出信号
	TIM17->BDTR |= TIM_BDTR_MOE;  // 设置 MOE (Main Output Enable) 位

}
//切换列选  把上一个列的引脚置高电平 当前列置低电平
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
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
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

  /* Create the semaphores(s) */
  /* definition and creation of usbBinarySem */
  osSemaphoreDef(usbBinarySem);
  usbBinarySemHandle = osSemaphoreCreate(osSemaphore(usbBinarySem), 1);

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
  osMessageQDef(Line8Queue, 100, uint16_t);
  Line8QueueHandle = osMessageCreate(osMessageQ(Line8Queue), NULL);

  /* definition and creation of Line9Queue */
  osMessageQDef(Line9Queue, 100, uint16_t);
  Line9QueueHandle = osMessageCreate(osMessageQ(Line9Queue), NULL);

  /* definition and creation of Line10Queue */
  osMessageQDef(Line10Queue, 100, uint16_t);
  Line10QueueHandle = osMessageCreate(osMessageQ(Line10Queue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 1024);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of notifyTask */
  osThreadDef(notifyTask, NotifyTask, osPriorityRealtime, 0, 128);
  notifyTaskHandle = osThreadCreate(osThread(notifyTask), NULL);

  /* definition and creation of dataTask */
  osThreadDef(dataTask, DataTask, osPriorityBelowNormal, 0, 128);
  dataTaskHandle = osThreadCreate(osThread(dataTask), NULL);

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
volatile uint16_t resistance[100];
volatile uint16_t current[100];

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
typedef union  {
    uint16_t word16;         // 16bit
		uint8_t byte[2];       // 8bit   byte[0]:low_8bit   byte[1]:high_8bit     example:  word16=0x5678  byte[0]=0x78 byte[1]=0x56
}Word_union;
Word_union test_union[5];
Word_union head_data[2];
Word_union res_value_buff[100][10];

/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	//F405:2-999-100-1.7us-70(2843.75Om)   low time about 3.5us
	uint8_t usb_TxBuf[1024];//小心不要超过任务栈大小
	uint8_t resistance_TxBuf[256+4]={0x55,0xaa,0xcc,0xbb};
	uint32_t usb_RxLength=0;
	uint8_t USBD_Result=0;
	
	uint16_t i=0;
	for(i=0;i<6;i++)
	{
		AD5206_SetResistance(0, i, 12);//12-528 Om
	}
	osDelay(100);
	for(i=0;i<6;i++)
	{
		AD5206_SetResistance(1, i, 12);//12-542 Om
	}
	osDelay(100);
	
	AD5206_SetResistance(0, 4, 50);//Vramp1
	//            Prescaler-Period-CCR-wave_time-resistance_para    100pf   dc_power_supply: 5.0V
	//Capture_Filter:x
	//G431 200MHz:0-9999-4450(44.5us)-50(about Om)    200MHz cap PWM_low_value:x   cap VO1_1V1:x
	//G431 200MHz:0-9999-2700(27.0us)-30(about Om)    200MHz cap PWM_low_value:x   cap VO1_1V1:x
	osDelay(100);
	
	AD5206_SetResistance(0, 5, 255);//Vramp2
	//            Prescaler-Period-CCR-wave_time-resistance_para    100pf   usb_power_supply: 4.9V
	//Capture_Filter:15
	//G431 200MHz:0-9999-200(2.0us)-80(about Om)    200MHz cap PWM_low_value:416   cap VO1_1V1:256
	//G431 200MHz:0-9999-300(3.0us)-120(about Om)    200MHz cap PWM_low_value:608   cap VO1_1V1:384
	//G431 200MHz:0-9999-400(4.0us)-160(about Om)    160/80.0*200=400    200MHz cap PWM_low_value:800   cap VO1_1V1:512
	//G431 200MHz:0-9999-637(6.37us)-255(about Om)    255/160.0*400=637.5    200MHz cap PWM_low_value:1280   cap VO1_1V1:832   usb_power_supply: 4.75V
	osDelay(100);
	
	HAL_TIM_PWM_Start(&htim15,TIM_CHANNEL_1);
	htim15.Instance->CCR1 = 637;//Vramp2
	HAL_TIM_PWM_Start(&htim15,TIM_CHANNEL_2);
	htim15.Instance->CCR2 = 4450;//Vramp1 test 现不用
	
	
	uint32_t receivedMessage=0;
	osEvent event;
	
	uint32_t ulNotificationValue=0;
	
	
	
	/* Infinite loop */
	for(;;)
	{
		//焊接运放芯片注意GND引脚不要虚焊，用特尖表笔对每个焊盘和引脚都测一下有没有焊上
		LL_GPIO_SetOutputPin(LED_GPIO_Port,LED_Pin);
		
//		SEL1_GPIO_Port->BSRR=SEL1_pin_HIGH;//高电平
//		SEL2_GPIO_Port->BSRR=SEL2_pin_HIGH;//高电平
//		SEL3_GPIO_Port->BSRR=SEL3_pin_HIGH;//高电平
//		SEL4_GPIO_Port->BSRR=SEL4_pin_HIGH;//高电平
//		SEL5_GPIO_Port->BSRR=SEL5_pin_HIGH;//高电平
//		SEL6_GPIO_Port->BSRR=SEL6_pin_HIGH;//高电平
//		SEL7_GPIO_Port->BSRR=SEL7_pin_HIGH;//高电平
//		SEL8_GPIO_Port->BSRR=SEL8_pin_HIGH;//高电平
//		SEL9_GPIO_Port->BSRR=SEL9_pin_HIGH;//高电平
//		SEL10_GPIO_Port->BSRR=SEL10_pin_HIGH;//高电平
		osDelay(100);
	  
		LL_GPIO_ResetOutputPin(LED_GPIO_Port,LED_Pin);
		
//		SEL1_GPIO_Port->BSRR=SEL1_pin_LOW;//低电平
//		SEL2_GPIO_Port->BSRR=SEL2_pin_LOW;//低电平
//		SEL3_GPIO_Port->BSRR=SEL3_pin_LOW;//低电平
//		SEL4_GPIO_Port->BSRR=SEL4_pin_LOW;//低电平
//		SEL5_GPIO_Port->BSRR=SEL5_pin_LOW;//低电平
//		SEL6_GPIO_Port->BSRR=SEL6_pin_LOW;//低电平
//		SEL7_GPIO_Port->BSRR=SEL7_pin_LOW;//低电平
//		SEL8_GPIO_Port->BSRR=SEL8_pin_LOW;//低电平
//		SEL9_GPIO_Port->BSRR=SEL9_pin_LOW;//低电平
//		SEL10_GPIO_Port->BSRR=SEL10_pin_LOW;//低电平
		osDelay(100);
	  
	  
//		test_union[0].word16 = 0x5678;
//		res_value_buff[0][0].byte[0]='0';
//		res_value_buff[0][0].byte[1]='1';
//		res_value_buff[0][1].byte[0]='2';
//		res_value_buff[0][1].byte[1]='3';
//		res_value_buff[0][2].byte[0]='4';
//		res_value_buff[0][2].byte[1]='5';
//		res_value_buff[0][3].byte[0]='6';
//		res_value_buff[0][3].byte[1]='7';
//		res_value_buff[0][4].byte[0]='8';
//		res_value_buff[0][4].byte[1]='9';
//		res_value_buff[0][5].word16 =0x4241;
//		res_value_buff[0][6].byte[0]='a';
//		res_value_buff[0][6].byte[1]='\n';
//		
//		CDC_Transmit_FS(res_value_buff[0][0].byte, 14);
//		osDelay(1);
	  
		
//		sprintf((char *)usb_TxBuf,"StartDefaultTask!!&res_value_buff[0][0].word16:%p,&res_value_buff[0][0].byte[0]:%p,&head_data[0].byte[0]:%p,&res_value_buff[0][1].word16:%p\r\n",
//			&res_value_buff[0][0].word16,&res_value_buff[0][0].byte[0],&head_data[0].byte[0],&res_value_buff[0][1].word16);
//		usb_RxLength=strlen((char *)usb_TxBuf);
//		
//		osSemaphoreWait(usbBinarySemHandle, osWaitForever);
//		do{
//			USBD_Result=CDC_Transmit_FS(usb_TxBuf, usb_RxLength);
//		}while(USBD_Result!=USBD_OK);
////		USBD_Result=CDC_Transmit_FS(usb_TxBuf, usb_RxLength);
////		while(USBD_Result!=USBD_OK)
////		{
////			USBD_Result=CDC_Transmit_FS(usb_TxBuf, usb_RxLength);
////		}
//		osSemaphoreRelease(usbBinarySemHandle);
		
//		osDelay(10);
		
		
		
//		print char
//		event = osMessageGet(Line1QueueHandle, osWaitForever);
//		receivedMessage = event.value.v;
		
//		sprintf((char *)usb_TxBuf,"Remaining StartDefaultTask stack space: %lu bytes\n TIM3_base_it_cnt:%u\n Timer3_ch1_Cap.low_time:%d\n Timer3_ch2_Cap.low_time:%d\n Timer3_ch3_Cap.low_time:%d\n Timer3_ch4_Cap.low_time:%d\n\n Timer4_ch1_Cap.low_time:%d\n Timer4_ch2_Cap.low_time:%d\n Timer4_ch4_Cap.low_time:%d\n\n Timer2_ch2_Cap.low_time:%d\n Timer2_ch3_Cap.low_time:%d\n Timer2_ch4_Cap.low_time:%d\n receivedMessage:%u\n Line1QueueHandle addr:%p \n Line2QueueHandle addr:%p \n", 
//			uxTaskGetStackHighWaterMark(NULL) * 4,TIM3_base_it_cnt,Timer3_ch1_Cap.low_time,Timer3_ch2_Cap.low_time,Timer3_ch3_Cap.low_time,Timer3_ch4_Cap.low_time,Timer4_ch1_Cap.low_time,Timer4_ch2_Cap.low_time,Timer4_ch4_Cap.low_time,Timer2_ch2_Cap.low_time,Timer2_ch3_Cap.low_time,Timer2_ch4_Cap.low_time,receivedMessage,&Line1QueueHandle,&Line10QueueHandle);
			
//		//wave show
//		sprintf((char *)usb_TxBuf,"%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",  //%u,%u,%u,%u,%u,%u,%u,%u,%u,%u
//			Timer3_ch1_Cap.low_time,Timer3_ch2_Cap.low_time,Timer3_ch3_Cap.low_time,Timer3_ch4_Cap.low_time,Timer4_ch1_Cap.low_time,Timer4_ch2_Cap.low_time,Timer4_ch4_Cap.low_time,Timer2_ch2_Cap.low_time,Timer2_ch3_Cap.low_time,Timer2_ch4_Cap.low_time);
		
//		usb_RxLength=strlen((char *)usb_TxBuf);
//			
//		osSemaphoreWait(usbBinarySemHandle, osWaitForever);
//		USBD_Result=CDC_Transmit_FS(usb_TxBuf, usb_RxLength);
//		while(USBD_Result!=USBD_OK)
//		{
//			USBD_Result=CDC_Transmit_FS(usb_TxBuf, usb_RxLength);
//		}
//		osSemaphoreRelease(usbBinarySemHandle);
		
		
		
		
//		for(uint8_t i=0;i<10;i++)
//		{
//			ulNotificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // wait for notification value
//			sprintf((char *)usb_TxBuf,"res_value[%2d][0~9]:%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\n",
//				ulNotificationValue,res_value[ulNotificationValue][0],res_value[ulNotificationValue][1],res_value[ulNotificationValue][2],res_value[ulNotificationValue][3],res_value[ulNotificationValue][4],
//				res_value[ulNotificationValue][5],res_value[ulNotificationValue][6],res_value[ulNotificationValue][7],res_value[ulNotificationValue][8],res_value[ulNotificationValue][9]);
//			
//			usb_RxLength=strlen((char *)usb_TxBuf);
//			
//			osSemaphoreWait(usbBinarySemHandle, osWaitForever);
//			USBD_Result=CDC_Transmit_FS(usb_TxBuf, usb_RxLength);
//			while(USBD_Result!=USBD_OK)
//			{
//				USBD_Result=CDC_Transmit_FS(usb_TxBuf, usb_RxLength);
//			}
//			osSemaphoreRelease(usbBinarySemHandle);
//		}
		
		
		
		
//		osDelay(10);
	}
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_NotifyTask */
/**
* @brief Function implementing the notifyTask thread.
* @param argument: Not used
* @retval None
*/

Word_union test_data[1004];
Word_union res_data_1[1004];
Word_union res_data_2[1004];
Word_union* res_send_p=res_data_1;
Word_union* res_store_p=res_data_2;

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
uint32_t start_time;//定时器7计数变量，1us计数一次
uint32_t time_cnt;
uint32_t current_time;
/* USER CODE END Header_NotifyTask */
void NotifyTask(void const * argument)
{
  /* USER CODE BEGIN NotifyTask */
	uint32_t ulNotificationValue=0;
	osDelay(20);
	uint8_t USBD_Result=0;
	
	test_data[0].byte[0]=0x55;//帧头
	test_data[0].byte[1]=0xAA;
	test_data[1].byte[0]=0xBB;
	test_data[1].byte[1]=0xCC;
	for(uint16_t i=0;i<1000;i++)
	{
		test_data[2+i].word16=i*2+2;
	}
	test_data[1002].byte[0]=0xAA;//帧尾
	test_data[1002].byte[1]=0x55;
	test_data[1003].byte[0]=0x66;
	test_data[1003].byte[1]=0x77;
	
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
  /* Infinite loop */
  for(;;)
  {
//		start_time=time_cnt;
		ulNotificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // wait for notification value
//		do{
			USBD_Result=CDC_Transmit_FS(res_send_p->byte, 2008);
//		}while(USBD_Result!=USBD_OK);//经过测试 仅运行这个发送函数只需要8us，一次完整的2008字节发送完毕需要2.3ms(433.7帧/秒)
		
//		current_time=time_cnt;
//		res_send_p[2].word16=(uint16_t)(current_time-start_time);
//		res_send_p[3].word16=(uint16_t)((current_time-start_time)>>16);
	  
//		osDelay(20);
		
/*
////		ulNotificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // wait for notification value
////		
////		if (ulNotificationValue) {// == 0x1234
////						LL_GPIO_TogglePin(GPIOC,GPIO_PIN_6);
//////						osDelay(100);
////        }
//		if(LL_GPIO_IsInputPinSet(GPIOC, LL_GPIO_PIN_13))//按键
//		{
//			osDelay(20);
//			while(LL_GPIO_IsInputPinSet(GPIOC, LL_GPIO_PIN_13));
//			osDelay(20);
//			
//			LL_GPIO_SetOutputPin(GPIOB,GPIO_PIN_11);
////			osDelay(1);
//			user_delaynus_tim(3);
//			
////			htim17.Instance->CCR1 = 4450;
////			user_delaynus_tim(500);
//			
////			ll_Start_TIM17_OnePulse(4450);
////			user_delaynus_tim(50);
//			
//			LL_GPIO_ResetOutputPin(GPIOB,GPIO_PIN_11);
//			
//		}		
//		osDelay(20);
*/
  }
  /* USER CODE END NotifyTask */
}

/* USER CODE BEGIN Header_DataTask */
/**
* @brief Function implementing the dataTask thread.
* @param argument: Not used
* @retval None
*/
//union Flags res_flags;
uint8_t res_flags[10]={0};
/* USER CODE END Header_DataTask */
void DataTask(void const * argument)
{
  /* USER CODE BEGIN DataTask */
  /* Infinite loop */
	uint32_t receivedMessage=0;
	osEvent event[10];
	uint8_t i=0,j=0;
	
	uint8_t Notify_send_flag=0;
	osDelay(2000);
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
  for(;;)
  {
			for(j=0;j<100;j++)
			{
				Select_switcher(j%10+1);
//				user_delaynus_tim(5);
				ll_Start_TIM17_OnePulse(4450);
				if(!Notify_send_flag)//第一次触发扫描波形后发送通知 数据发送指令的执行时间小于一次扫描
				{
					xTaskNotifyGive(notifyTaskHandle);//发送通知给usb通信任务  这个还是得放在ll_Start_TIM17_OnePulse之后，节约整体时间，在比较器已经开始采集新的一帧时候将上一帧发送出去
					taskYIELD();// 手动触发上下文切换
					Notify_send_flag = 1;
				}
				
					
				while(res_flags[0]+res_flags[1]+res_flags[2]+res_flags[3]+res_flags[4]+res_flags[5]+res_flags[6]+res_flags[7]+res_flags[8]+res_flags[9]!=10)
				{
					for(i=0;i<10;i++)
					{
						event[i] = osMessageGet(*(&Line1QueueHandle+i), 0);//osWaitForever
						if (event[i].status == osEventMessage)
						{
							res_store_p[j*10+i+2].word16=event[i].value.v;
							
//							if(res_store_p==res_data_1)//测试是否根据缓存切换来正确存入数据
//								res_store_p[j*10+i+2].word16=(j*10+i)*5;
//							if(res_store_p==res_data_2)
//								res_store_p[j*10+i+2].word16=(j*10+i)*5+1;
							
							res_flags[i]=1;
						}
						else
						{
							
						}
					}
				}
				for(i=0;i<10;i++)res_flags[i]=0;//清除单行/列就绪标志
			}
			//全部10帧数据缓存完毕
			exchange_res_p();//切换缓存区
			Notify_send_flag = 0;
			
			
//			xTaskNotifyGive(notifyTaskHandle);
//			xTaskNotifyGive(defaultTaskHandle);
//			xTaskNotify(defaultTaskHandle, j, eSetValueWithOverwrite);
//			vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
//			taskYIELD();// 手动触发上下文切换
  }
	while(1){osDelay(1000);}
  /* USER CODE END DataTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void TIM7_IRQHandler(void)
{
//	static uint16_t cnt=0;
	if (LL_TIM_IsActiveFlag_UPDATE(TIM7))  //check IRQ flag
	{
		LL_TIM_ClearFlag_UPDATE(TIM7);  //clear IRQ flag
		
		time_cnt++;//原用于1us中断来计时，现不用
//		if(cnt<100)cnt++;
//		else
//		{
//			cnt=0;
//			
////			BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//////			xTaskNotifyGive(notifyTaskHandle);//send notification (used in the task!!!)
//////			vTaskNotifyGiveFromISR(notifyTaskHandle, &xHigherPriorityTaskWoken);send notification in IRQ
////			xTaskNotifyFromISR(notifyTaskHandle,0x02,eSetValueWithOverwrite,&xHigherPriorityTaskWoken);//send notification in IRQ with value
////			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);//trigger task switching
//		}

		
	}
}
void TIM3_IRQHandler(void)
{
	if (LL_TIM_IsActiveFlag_UPDATE(TIM3))  //check IRQ flag
	{
		LL_TIM_ClearFlag_UPDATE(TIM3);  //clear IRQ flag
		TIM3_base_it_cnt++;
//		Timer3_ch1_Cap.OverflowCount++;
//		Timer3_ch2_Cap.OverflowCount++;
//		Timer3_ch3_Cap.OverflowCount++;
//		Timer3_ch4_Cap.OverflowCount++;
	}
	if(LL_TIM_IsActiveFlag_CC1(TIM3))
	{
		LL_TIM_ClearFlag_CC1(TIM3);	//clear CC1 IRQ flag
		
		switch(Timer3_ch1_Cap.capture_Cnt){
			case 0:
				Timer3_ch1_Cap.capture_Buf[0] = LL_TIM_IC_GetCaptureCH1(TIM3);//获取当前的捕获值.
				Timer3_ch1_Cap.OverflowCount=0;
				LL_TIM_IC_SetPolarity(TIM3,LL_TIM_CHANNEL_CH1,LL_TIM_IC_POLARITY_RISING);  //设置为上升沿捕获
				Timer3_ch1_Cap.capture_Cnt=1;
				break;
			case 1:
				Timer3_ch1_Cap.capture_Buf[1] = LL_TIM_IC_GetCaptureCH1(TIM3);//获取当前的捕获值.
//				Timer3_ch1_Cap.low_time = Timer3_ch1_Cap.capture_Buf[1] - Timer3_ch1_Cap.capture_Buf[0] + ( Timer3_ch1_Cap.OverflowCount << 16 );    //低电平时间
				Timer3_ch1_Cap.low_time = Timer3_ch1_Cap.capture_Buf[1] - Timer3_ch1_Cap.capture_Buf[0];    //低电平时间
				if(Timer3_ch1_Cap.low_time < 0 )Timer3_ch1_Cap.low_time += 0xFFFF;
			
				LL_TIM_IC_SetPolarity(TIM3,LL_TIM_CHANNEL_CH1,LL_TIM_IC_POLARITY_FALLING);  //设置为下降沿捕获
				Timer3_ch1_Cap.capture_Cnt=0;
			
				osMessagePut(Line1QueueHandle,(uint16_t)Timer3_ch1_Cap.low_time,0);
//				xReturn[0].status=osMessagePut(tim3ch1QueueHandle,(uint16_t)Timer3_ch1_Cap.low_time,0);
//				//	if(osOK!=xReturn.status)
//				//	{
//				//		while(1);
//				////		printf("send fail\n");
//				//	}
			
//			BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//    uint32_t message = Timer3_ch1_Cap.low_time;  // 获取计时器值作为消息

//    // 从中断向队列发送消息
//    xQueueSendFromISR(Line1QueueHandle, &message, &xHigherPriorityTaskWoken);

//    // 如果需要，触发上下文切换
//    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
	}
	if(LL_TIM_IsActiveFlag_CC2(TIM3))
	{
		LL_TIM_ClearFlag_CC2(TIM3);	//clear CC2 IRQ flag
		
		switch(Timer3_ch2_Cap.capture_Cnt){
			case 0:
				Timer3_ch2_Cap.capture_Buf[0] = LL_TIM_IC_GetCaptureCH2(TIM3);//获取当前的捕获值.
				Timer3_ch2_Cap.OverflowCount=0;
				LL_TIM_IC_SetPolarity(TIM3,LL_TIM_CHANNEL_CH2,LL_TIM_IC_POLARITY_RISING);  //设置为上升沿捕获
				Timer3_ch2_Cap.capture_Cnt=1;
				break;
			case 1:
				Timer3_ch2_Cap.capture_Buf[1] = LL_TIM_IC_GetCaptureCH2(TIM3);//获取当前的捕获值.
//				Timer3_ch2_Cap.low_time = Timer3_ch2_Cap.capture_Buf[1] - Timer3_ch2_Cap.capture_Buf[0] + ( Timer3_ch2_Cap.OverflowCount << 16 );    //低电平时间
				Timer3_ch2_Cap.low_time = Timer3_ch2_Cap.capture_Buf[1] - Timer3_ch2_Cap.capture_Buf[0];    //低电平时间
				if(Timer3_ch2_Cap.low_time < 0 )Timer3_ch2_Cap.low_time += 0xFFFF;
			
				LL_TIM_IC_SetPolarity(TIM3,LL_TIM_CHANNEL_CH2,LL_TIM_IC_POLARITY_FALLING);  //设置为下降沿捕获
				Timer3_ch2_Cap.capture_Cnt=0;
			
				osMessagePut(Line2QueueHandle,(uint16_t)Timer3_ch2_Cap.low_time,0);
		}
	}
	if(LL_TIM_IsActiveFlag_CC3(TIM3))
	{
		LL_TIM_ClearFlag_CC3(TIM3);	//clear CC3 IRQ flag
		
		switch(Timer3_ch3_Cap.capture_Cnt){
			case 0:
				Timer3_ch3_Cap.capture_Buf[0] = LL_TIM_IC_GetCaptureCH3(TIM3);//获取当前的捕获值.
				Timer3_ch3_Cap.OverflowCount=0;
				LL_TIM_IC_SetPolarity(TIM3,LL_TIM_CHANNEL_CH3,LL_TIM_IC_POLARITY_RISING);  //设置为上升沿捕获
				Timer3_ch3_Cap.capture_Cnt=1;
				break;
			case 1:
				Timer3_ch3_Cap.capture_Buf[1] = LL_TIM_IC_GetCaptureCH3(TIM3);//获取当前的捕获值.
//				Timer3_ch3_Cap.low_time = Timer3_ch3_Cap.capture_Buf[1] - Timer3_ch3_Cap.capture_Buf[0] + ( Timer3_ch3_Cap.OverflowCount << 16 );    //低电平时间
				Timer3_ch3_Cap.low_time = Timer3_ch3_Cap.capture_Buf[1] - Timer3_ch3_Cap.capture_Buf[0];    //低电平时间
				if(Timer3_ch3_Cap.low_time < 0 )Timer3_ch3_Cap.low_time += 0xFFFF;
			
				LL_TIM_IC_SetPolarity(TIM3,LL_TIM_CHANNEL_CH3,LL_TIM_IC_POLARITY_FALLING);  //设置为下降沿捕获
				Timer3_ch3_Cap.capture_Cnt=0;
			
				osMessagePut(Line3QueueHandle,(uint16_t)Timer3_ch3_Cap.low_time,0);
		}
	}
	if(LL_TIM_IsActiveFlag_CC4(TIM3))
	{
		LL_TIM_ClearFlag_CC4(TIM3);	//clear CC4 IRQ flag
		
		switch(Timer3_ch4_Cap.capture_Cnt){
			case 0:
				Timer3_ch4_Cap.capture_Buf[0] = LL_TIM_IC_GetCaptureCH4(TIM3);//获取当前的捕获值.
				Timer3_ch4_Cap.OverflowCount=0;
				LL_TIM_IC_SetPolarity(TIM3,LL_TIM_CHANNEL_CH4,LL_TIM_IC_POLARITY_RISING);  //设置为上升沿捕获
				Timer3_ch4_Cap.capture_Cnt=1;
				break;
			case 1:
				Timer3_ch4_Cap.capture_Buf[1] = LL_TIM_IC_GetCaptureCH4(TIM3);//获取当前的捕获值.
//				Timer3_ch4_Cap.low_time = Timer3_ch4_Cap.capture_Buf[1] - Timer3_ch4_Cap.capture_Buf[0] + ( Timer3_ch4_Cap.OverflowCount << 16 );    //低电平时间
				Timer3_ch4_Cap.low_time = Timer3_ch4_Cap.capture_Buf[1] - Timer3_ch4_Cap.capture_Buf[0];    //低电平时间
				if(Timer3_ch4_Cap.low_time < 0 )Timer3_ch4_Cap.low_time += 0xFFFF;
			
				LL_TIM_IC_SetPolarity(TIM3,LL_TIM_CHANNEL_CH4,LL_TIM_IC_POLARITY_FALLING);  //设置为下降沿捕获
				Timer3_ch4_Cap.capture_Cnt=0;
			
				osMessagePut(Line4QueueHandle,(uint16_t)Timer3_ch4_Cap.low_time,0);
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
				Timer4_ch1_Cap.capture_Buf[0] = LL_TIM_IC_GetCaptureCH1(TIM4);//获取当前的捕获值.
				Timer4_ch1_Cap.OverflowCount=0;
				LL_TIM_IC_SetPolarity(TIM4,LL_TIM_CHANNEL_CH1,LL_TIM_IC_POLARITY_RISING);  //设置为上升沿捕获
				Timer4_ch1_Cap.capture_Cnt=1;
				break;
			case 1:
				Timer4_ch1_Cap.capture_Buf[1] = LL_TIM_IC_GetCaptureCH1(TIM4);//获取当前的捕获值.
//				Timer4_ch1_Cap.low_time = Timer4_ch1_Cap.capture_Buf[1] - Timer4_ch1_Cap.capture_Buf[0] + ( Timer4_ch1_Cap.OverflowCount << 16 );    //低电平时间
				Timer4_ch1_Cap.low_time = Timer4_ch1_Cap.capture_Buf[1] - Timer4_ch1_Cap.capture_Buf[0];    //低电平时间
				if(Timer4_ch1_Cap.low_time < 0 )Timer4_ch1_Cap.low_time += 0xFFFF;
			
				LL_TIM_IC_SetPolarity(TIM4,LL_TIM_CHANNEL_CH1,LL_TIM_IC_POLARITY_FALLING);  //设置为下降沿捕获
				Timer4_ch1_Cap.capture_Cnt=0;
			
				osMessagePut(Line5QueueHandle,(uint16_t)Timer4_ch1_Cap.low_time,0);
		}
	}
	if(LL_TIM_IsActiveFlag_CC2(TIM4))
	{
		LL_TIM_ClearFlag_CC2(TIM4);	//clear CC2 IRQ flag
		
		switch(Timer4_ch2_Cap.capture_Cnt){
			case 0:
				Timer4_ch2_Cap.capture_Buf[0] = LL_TIM_IC_GetCaptureCH2(TIM4);//获取当前的捕获值.
				Timer4_ch2_Cap.OverflowCount=0;
				LL_TIM_IC_SetPolarity(TIM4,LL_TIM_CHANNEL_CH2,LL_TIM_IC_POLARITY_RISING);  //设置为上升沿捕获
				Timer4_ch2_Cap.capture_Cnt=1;
				break;
			case 1:
				Timer4_ch2_Cap.capture_Buf[1] = LL_TIM_IC_GetCaptureCH2(TIM4);//获取当前的捕获值.
//				Timer4_ch2_Cap.low_time = Timer4_ch2_Cap.capture_Buf[1] - Timer4_ch2_Cap.capture_Buf[0] + ( Timer4_ch2_Cap.OverflowCount << 16 );    //低电平时间
				Timer4_ch2_Cap.low_time = Timer4_ch2_Cap.capture_Buf[1] - Timer4_ch2_Cap.capture_Buf[0];    //低电平时间
				if(Timer4_ch2_Cap.low_time < 0 )Timer4_ch2_Cap.low_time += 0xFFFF;
			
				LL_TIM_IC_SetPolarity(TIM4,LL_TIM_CHANNEL_CH2,LL_TIM_IC_POLARITY_FALLING);  //设置为下降沿捕获
				Timer4_ch2_Cap.capture_Cnt=0;
			
				osMessagePut(Line6QueueHandle,(uint16_t)Timer4_ch2_Cap.low_time,0);
		}
	}
	if(LL_TIM_IsActiveFlag_CC4(TIM4))
	{
		LL_TIM_ClearFlag_CC4(TIM4);	//clear CC4 IRQ flag
		
		switch(Timer4_ch4_Cap.capture_Cnt){
			case 0:
				Timer4_ch4_Cap.capture_Buf[0] = LL_TIM_IC_GetCaptureCH4(TIM4);//获取当前的捕获值.
				Timer4_ch4_Cap.OverflowCount=0;
				LL_TIM_IC_SetPolarity(TIM4,LL_TIM_CHANNEL_CH4,LL_TIM_IC_POLARITY_RISING);  //设置为上升沿捕获
				Timer4_ch4_Cap.capture_Cnt=1;
				break;
			case 1:
				Timer4_ch4_Cap.capture_Buf[1] = LL_TIM_IC_GetCaptureCH4(TIM4);//获取当前的捕获值.
				Timer4_ch4_Cap.low_time = Timer4_ch4_Cap.capture_Buf[1] - Timer4_ch4_Cap.capture_Buf[0] + ( Timer4_ch4_Cap.OverflowCount << 16 );    //低电平时间
				Timer4_ch4_Cap.low_time = Timer4_ch4_Cap.capture_Buf[1] - Timer4_ch4_Cap.capture_Buf[0];    //低电平时间
				if(Timer4_ch4_Cap.low_time < 0 )Timer4_ch4_Cap.low_time += 0xFFFF;
			
				LL_TIM_IC_SetPolarity(TIM4,LL_TIM_CHANNEL_CH4,LL_TIM_IC_POLARITY_FALLING);  //设置为下降沿捕获
				Timer4_ch4_Cap.capture_Cnt=0;
			
				osMessagePut(Line7QueueHandle,(uint16_t)Timer4_ch4_Cap.low_time,0);
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
				Timer2_ch2_Cap.capture_Buf[0] = LL_TIM_IC_GetCaptureCH2(TIM2);//获取当前的捕获值.
				Timer2_ch2_Cap.OverflowCount=0;
				LL_TIM_IC_SetPolarity(TIM2,LL_TIM_CHANNEL_CH2,LL_TIM_IC_POLARITY_RISING);  //设置为上升沿捕获
				Timer2_ch2_Cap.capture_Cnt=1;
				break;
			case 1:
				Timer2_ch2_Cap.capture_Buf[1] = LL_TIM_IC_GetCaptureCH2(TIM2);//获取当前的捕获值.
//				Timer2_ch2_Cap.low_time = Timer2_ch2_Cap.capture_Buf[1] - Timer2_ch2_Cap.capture_Buf[0] + ( Timer2_ch2_Cap.OverflowCount << 16 );    //低电平时间
				Timer2_ch2_Cap.low_time = Timer2_ch2_Cap.capture_Buf[1] - Timer2_ch2_Cap.capture_Buf[0];    //低电平时间
				if(Timer2_ch2_Cap.low_time < 0 )Timer2_ch2_Cap.low_time += 0xFFFFFFFF;
			
				LL_TIM_IC_SetPolarity(TIM2,LL_TIM_CHANNEL_CH2,LL_TIM_IC_POLARITY_FALLING);  //设置为下降沿捕获
				Timer2_ch2_Cap.capture_Cnt=0;
			
				osMessagePut(Line8QueueHandle,(uint16_t)Timer2_ch2_Cap.low_time,0);
		}
	}
	if(LL_TIM_IsActiveFlag_CC3(TIM2))
	{
		LL_TIM_ClearFlag_CC3(TIM2);	//clear CC3 IRQ flag
		
		switch(Timer2_ch3_Cap.capture_Cnt){
			case 0:
				Timer2_ch3_Cap.capture_Buf[0] = LL_TIM_IC_GetCaptureCH3(TIM2);//获取当前的捕获值.
				Timer2_ch3_Cap.OverflowCount=0;
				LL_TIM_IC_SetPolarity(TIM2,LL_TIM_CHANNEL_CH3,LL_TIM_IC_POLARITY_RISING);  //设置为上升沿捕获
				Timer2_ch3_Cap.capture_Cnt=1;
				break;
			case 1:
				Timer2_ch3_Cap.capture_Buf[1] = LL_TIM_IC_GetCaptureCH3(TIM2);//获取当前的捕获值.
				Timer2_ch3_Cap.low_time = Timer2_ch3_Cap.capture_Buf[1] - Timer2_ch3_Cap.capture_Buf[0] + ( Timer2_ch3_Cap.OverflowCount << 16 );    //低电平时间
				Timer2_ch3_Cap.low_time = Timer2_ch3_Cap.capture_Buf[1] - Timer2_ch3_Cap.capture_Buf[0];    //低电平时间
				if(Timer2_ch3_Cap.low_time < 0 )Timer2_ch3_Cap.low_time += 0xFFFFFFFF;
			
				LL_TIM_IC_SetPolarity(TIM2,LL_TIM_CHANNEL_CH3,LL_TIM_IC_POLARITY_FALLING);  //设置为下降沿捕获
				Timer2_ch3_Cap.capture_Cnt=0;
			
				osMessagePut(Line9QueueHandle,(uint16_t)Timer2_ch3_Cap.low_time,0);
		}
	}
	if(LL_TIM_IsActiveFlag_CC4(TIM2))
	{
		LL_TIM_ClearFlag_CC4(TIM2);	//clear CC4 IRQ flag
		
		switch(Timer2_ch4_Cap.capture_Cnt){
			case 0:
				Timer2_ch4_Cap.capture_Buf[0] = LL_TIM_IC_GetCaptureCH4(TIM2);//获取当前的捕获值.
				Timer2_ch4_Cap.OverflowCount=0;
				LL_TIM_IC_SetPolarity(TIM2,LL_TIM_CHANNEL_CH4,LL_TIM_IC_POLARITY_RISING);  //设置为上升沿捕获
				Timer2_ch4_Cap.capture_Cnt=1;
				break;
			case 1:
				Timer2_ch4_Cap.capture_Buf[1] = LL_TIM_IC_GetCaptureCH4(TIM2);//获取当前的捕获值.
				Timer2_ch4_Cap.low_time = Timer2_ch4_Cap.capture_Buf[1] - Timer2_ch4_Cap.capture_Buf[0] + ( Timer2_ch4_Cap.OverflowCount << 16 );    //低电平时间
				Timer2_ch4_Cap.low_time = Timer2_ch4_Cap.capture_Buf[1] - Timer2_ch4_Cap.capture_Buf[0];    //低电平时间
				if(Timer2_ch4_Cap.low_time < 0 )Timer2_ch4_Cap.low_time += 0xFFFFFFFF;
			
				LL_TIM_IC_SetPolarity(TIM2,LL_TIM_CHANNEL_CH4,LL_TIM_IC_POLARITY_FALLING);  //设置为下降沿捕获
				Timer2_ch4_Cap.capture_Cnt=0;
			
				osMessagePut(Line10QueueHandle,(uint16_t)Timer2_ch4_Cap.low_time,0);
		}
	}
	
	
}

// 设置AD5206的电阻值
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

