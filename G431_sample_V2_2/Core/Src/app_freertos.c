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
osThreadId GPIOtriggerTaskHandle;
osThreadId usbTaskHandle;
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
uint16_t countBits(uint16_t n);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void gpiotrigTask(void const * argument);
void USBTask(void const * argument);

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

  /* definition and creation of GPIOtriggerTask */
  osThreadDef(GPIOtriggerTask, gpiotrigTask, osPriorityBelowNormal, 0, 128);
  GPIOtriggerTaskHandle = osThreadCreate(osThread(GPIOtriggerTask), NULL);

  /* definition and creation of usbTask */
  osThreadDef(usbTask, USBTask, osPriorityNormal, 0, 512);
  usbTaskHandle = osThreadCreate(osThread(usbTask), NULL);

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
	/* 测试步骤 */
	/* 
	使用KEITHLEY直流电源供稳定5V  单片机3V3与板载3V3不连通，5V也不连通
	开启FreeRTOS
	按键 LED灯测试
	USB CDC测试
	数字电位器AD5206测试 SPI 	CS(NSS)置回高才生效执行命令
	多路选择器MUX测试  输入高电平时输出ref voltage 输入低电平的时候输出GND
	MUX之后的运放(电压跟随器) 测试有没有虚焊，最终将MUX输入全为高电平时FPC排座引脚的IN1~IN10测出来全是ref voltage
	用定时器7进行1us中断进行us级延时，产生50us测试凹方波，接到输入捕获引脚，测试接收到的计算数值
	
	插入比较器模块之后，需接入传感器矩阵电路或者关闭输入捕获，不然单片机会卡死
	
	原TLV3502不知什么原因插上后Vramp幅度变小，先不管
	TLV3201 比较器输出端波形波动约为700ns(在1us内)
	COS3201 比较器输出端波形无波动 电平下降时间约为70ns(在100ns内)	******
	
	测试移植Select_switcher函数，添加修改main.h宏定义
	*/
	AD5206_SetResistance(0, 4, 50);//Vramp     配置50对应43us波形时间
	AD5206_SetResistance(0, 5, 4);//ref voltage     
	//21-1.009V    
	//8-0.500V  此时运放反馈电阻最大设置为30 (测得200欧姆电阻时的低电平时长为8.8us)
	//6-0.4039V 此时运放反馈电阻最大设置为40 (测得200欧姆电阻时的低电平时长为8.9us)
	//4-0.2973V 此时运放反馈电阻最大设置为58 (测得200欧姆电阻时的低电平时长为8.7us)
	//2-0.1822V 此时运放反馈电阻最大设置为100(测得200欧姆电阻时的低电平时长为9.0us)
	
	AD5206_SetResistance(0, 0, 58);//运放同相放大反馈电阻   12: 522Ω~524Ω
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
	HAL_GPIO_WritePin(SEL3_GPIO_Port, SEL3_Pin ,1);//U2 多路选择器有问题 连锡短路(*已解决*)
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
		//按键 LED灯测试
		if(HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin))
		{
			vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
			while(HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin))
				vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));

			osMessagePut(keyQueueHandle,1,0);//向队列发送消息,通知usb传输任务打印按键事件
			
			vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
			HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);//按键按一次翻转一次LED
			
		}
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));//		osDelay(1);
	}
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_gpiotrigTask */
/**
* @brief Function implementing the GPIOtriggerTask thread.
* @param argument: Not used
* @retval None
*/

// 定义一个联合体，包含一个16位的整数和一个16位的位域结构
typedef union {
    uint16_t all_data;  // 16位原始数据
    struct {
        uint16_t queue0 : 1;   // 第0位表示队列0是否就绪
        uint16_t queue1 : 1;   // 第1位表示队列1是否就绪
        uint16_t queue2 : 1;   // 第2位表示队列2是否就绪
        uint16_t queue3 : 1;   // 第3位表示队列3是否就绪
        uint16_t queue4 : 1;   // 第4位表示队列4是否就绪
        uint16_t queue5 : 1;   // 第5位表示队列5是否就绪
        uint16_t queue6 : 1;   // 第6位表示队列6是否就绪
        uint16_t queue7 : 1;   // 第7位表示队列7是否就绪
        uint16_t queue8 : 1;   // 第8位表示队列8是否就绪
        uint16_t queue9 : 1;   // 第9位表示队列9是否就绪
        uint16_t queue10 : 1;  // 第10位表示队列10是否就绪
//        uint16_t queue11 : 1;  // 第11位表示队列11是否就绪
//        uint16_t queue12 : 1;  // 第12位表示队列12是否就绪
//        uint16_t queue13 : 1;  // 第13位表示队列13是否就绪
//        uint16_t queue14 : 1;  // 第14位表示队列14是否就绪
//        uint16_t queue15 : 1;  // 第15位表示队列15是否就绪
    } queues;  // 位域结构
} QueueStatus;
volatile QueueStatus queuestatus;
volatile osEvent event[11];
volatile uint8_t cnt_order_TIM16;
volatile uint8_t cnt_TIM16;
volatile uint32_t cnt_error;

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
volatile BaseType_t xStatus[11];
volatile uint16_t comparator_data[11];
/* USER CODE END Header_gpiotrigTask */
void gpiotrigTask(void const * argument)
{
  /* USER CODE BEGIN gpiotrigTask */

	osDelay(500);
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = pdMS_TO_TICKS(1);
	
	
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
  /* Infinite loop */
	for(;;)
	{
		/*us级延时，GPIO产生40us方波*/
//		HAL_GPIO_WritePin(TEST_GPIO_Port,TEST_Pin,0);
//		user_delaynus_tim(40);
//		HAL_GPIO_WritePin(TEST_GPIO_Port,TEST_Pin,1);
//		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
		

		for(uint8_t j=0;j<10;j++)
		{
			for(uint8_t i=1;i<11;i++)//不知道为什么开多个顺序就会全乱了 *已解决*
			{
				queuestatus.all_data = 0;
				cnt_TIM16 = 0;//超时计数
				Select_switcher(i);//i
				Start_TIM17_OnePulse(4300);//测试锯齿波生成
				cnt_order_TIM16 = 1;//采样开始，开始超时计数
//				user_delaynus_tim(43);
				while(queuestatus.all_data!=0x07FE)
				{
					if(cnt_TIM16>50)//超过50us直接退出
					{
						cnt_error+=10-countBits(queuestatus.all_data);//记录错误未接收到的个数
						break;
					}
				}
				cnt_order_TIM16 = 0;//采样结束，停止超时计数
				
				{xStatus[ 1] = xQueueReceive(Line1QueueHandle , &comparator_data[ 1], 0);if(xStatus[ 1]){ res_store_p[j*100+(i-1)*10+0+2].word16=comparator_data[ 1]; print_data[j*100+(i-1)*10+0+2].word16=comparator_data[ 1];}  }//comparator_data[ 1]
				{xStatus[ 2] = xQueueReceive(Line2QueueHandle , &comparator_data[ 2], 0);if(xStatus[ 2]){ res_store_p[j*100+(i-1)*10+1+2].word16=comparator_data[ 2]; print_data[j*100+(i-1)*10+1+2].word16=comparator_data[ 2];}  }//comparator_data[ 2]
				{xStatus[ 3] = xQueueReceive(Line3QueueHandle , &comparator_data[ 3], 0);if(xStatus[ 3]){ res_store_p[j*100+(i-1)*10+2+2].word16=comparator_data[ 3]; print_data[j*100+(i-1)*10+2+2].word16=comparator_data[ 3];}  }//comparator_data[ 3]
				{xStatus[ 4] = xQueueReceive(Line4QueueHandle , &comparator_data[ 4], 0);if(xStatus[ 4]){ res_store_p[j*100+(i-1)*10+3+2].word16=comparator_data[ 4]; print_data[j*100+(i-1)*10+3+2].word16=comparator_data[ 4];}  }//comparator_data[ 4]
				{xStatus[ 5] = xQueueReceive(Line5QueueHandle , &comparator_data[ 5], 0);if(xStatus[ 5]){ res_store_p[j*100+(i-1)*10+4+2].word16=comparator_data[ 5]; print_data[j*100+(i-1)*10+4+2].word16=comparator_data[ 5];}  }//comparator_data[ 5]
				{xStatus[ 6] = xQueueReceive(Line6QueueHandle , &comparator_data[ 6], 0);if(xStatus[ 6]){ res_store_p[j*100+(i-1)*10+5+2].word16=comparator_data[ 6]; print_data[j*100+(i-1)*10+5+2].word16=comparator_data[ 6];}  }//comparator_data[ 6]
				{xStatus[ 7] = xQueueReceive(Line7QueueHandle , &comparator_data[ 7], 0);if(xStatus[ 7]){ res_store_p[j*100+(i-1)*10+6+2].word16=comparator_data[ 7]; print_data[j*100+(i-1)*10+6+2].word16=comparator_data[ 7];}  }//comparator_data[ 7]
				{xStatus[ 8] = xQueueReceive(Line8QueueHandle , &comparator_data[ 8], 0);if(xStatus[ 8]){ res_store_p[j*100+(i-1)*10+7+2].word16=comparator_data[ 8]; print_data[j*100+(i-1)*10+7+2].word16=comparator_data[ 8];}  }//comparator_data[ 8]
				{xStatus[ 9] = xQueueReceive(Line9QueueHandle , &comparator_data[ 9], 0);if(xStatus[ 9]){ res_store_p[j*100+(i-1)*10+8+2].word16=comparator_data[ 9]; print_data[j*100+(i-1)*10+8+2].word16=comparator_data[ 9];}  }//comparator_data[ 9]
				{xStatus[10] = xQueueReceive(Line10QueueHandle, &comparator_data[10], 0);if(xStatus[10]){ res_store_p[j*100+(i-1)*10+9+2].word16=comparator_data[10]; print_data[j*100+(i-1)*10+9+2].word16=comparator_data[10];}  }//comparator_data[10]
//				if( queuestatus.queues.queue1  )
//				if( queuestatus.queues.queue2  )	
//				if( queuestatus.queues.queue3  )	
//				if( queuestatus.queues.queue4  )	
//				if( queuestatus.queues.queue5  )	
//				if( queuestatus.queues.queue6  )	
//				if( queuestatus.queues.queue7  )	
//				if( queuestatus.queues.queue8  )	
//				if( queuestatus.queues.queue9  )	
//				if( queuestatus.queues.queue10 )	
					

			}
				vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));//osDelay(1);
		}
//		osDelay(100);
		exchange_res_p();//切换缓存区
		CDC_Transmit_FS(res_send_p->byte, 2008);
		
	}
  /* USER CODE END gpiotrigTask */
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
	uint8_t usb_TxBuf[128];//小心不要超过任务栈大小
	uint32_t usb_RxLength=0;
	uint8_t USBD_Result=0;
	
	osEvent event[11];
	osEvent key_event;
	uint8_t i = 0;
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
  /* Infinite loop */
	for(;;)
	{
		//读按键队列数据
		key_event = osMessageGet(keyQueueHandle, 0);
		if(key_event.status == osEventMessage)
		{
			sprintf((char *)usb_TxBuf,"The key is pressed.queuestatus:%d,countBits:%d,cnt_error:%d\r\n",queuestatus.all_data,countBits(queuestatus.all_data),cnt_error);
			usb_RxLength=strlen((char *)usb_TxBuf);
			CDC_Transmit_FS(usb_TxBuf, usb_RxLength);//USB CDC测试
//			do{
//				USBD_Result=CDC_Transmit_FS(usb_TxBuf, usb_RxLength);
//				vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));//osDelay(1);
//			}while(USBD_Result!=USBD_OK);//发送失败重发
		}
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));
		
		/*	测试所有通道接收
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
////			event[i] = osMessageGet(*(&Line1QueueHandle+i-1), 0);//osWaitForever
////			if (event[i].status == osEventMessage)
//			{
//				sprintf((char *)usb_TxBuf,"event[%2d].value.v:%.3fus\r\n",i,(event[i].value.v*5)/1000.0);
//				usb_RxLength=strlen((char *)usb_TxBuf);
////				do{
////					USBD_Result=CDC_Transmit_FS(usb_TxBuf, usb_RxLength);
////					vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));//osDelay(1);
////				}while(USBD_Result!=USBD_OK);//发送失败重发
//			}
//		}
//		for(i=0;i<100;i++)//print_data[j*100+(i-1)*10+0+2].word16
//		{
////			event[i] = osMessageGet(*(&Line1QueueHandle+i-1), 0);//osWaitForever
////			if (event[i].status == osEventMessage)
//			{
//				sprintf((char *)usb_TxBuf,"print_data[%d].word16:%d\r\n",i,print_data[i+2].word16);
//				usb_RxLength=strlen((char *)usb_TxBuf);
//				do{
//					USBD_Result=CDC_Transmit_FS(usb_TxBuf, usb_RxLength);
//					vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));//osDelay(1);
//				}while(USBD_Result!=USBD_OK);//发送失败重发
//			}
//		}
	}
  /* USER CODE END USBTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void TIM7_IRQHandler(void)
{
	static uint16_t time_cnt=0;
	if (LL_TIM_IsActiveFlag_UPDATE(TIM7))  //check IRQ flag
	{
		LL_TIM_ClearFlag_UPDATE(TIM7);  //clear IRQ flag
		
//		time_cnt++;//原用于1us中断来计时，现不用
		
		
	}
}

void TIM1_UP_TIM16_IRQHandler(void)
{
//	static uint16_t time_cnt=0;
	if (LL_TIM_IsActiveFlag_UPDATE(TIM16))  //check IRQ flag
	{
		LL_TIM_ClearFlag_UPDATE(TIM16);  //clear IRQ flag
		
		if(cnt_order_TIM16)cnt_TIM16++;
//		time_cnt++;//计时
//		
//		if(time_cnt>=100)
//		{
//			time_cnt=0;
////			Start_TIM17_OnePulse(4300);//测试锯齿波生成
//		}

		
	}
}
void TIM3_IRQHandler(void)
{
	if (LL_TIM_IsActiveFlag_UPDATE(TIM3))  //check IRQ flag
	{
		LL_TIM_ClearFlag_UPDATE(TIM3);  //clear IRQ flag
		TIM3_base_it_cnt++;
//		Timer3_ch1_Cap.OverflowCount++;//溢出计数会莫名多计一次数，造成误差很大，直接不溢出计数，保持输入低电平信号时间在一个65536计数周期内即可
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
			
				if(cnt_order_TIM16)
				{
					osMessagePut(Line9QueueHandle,(uint16_t)Timer3_ch1_Cap.low_time,0);
					queuestatus.queues.queue9 = 1;
				}
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
			
				if(cnt_order_TIM16)
				{
				osMessagePut(Line8QueueHandle,(uint16_t)Timer3_ch2_Cap.low_time,0);
				queuestatus.queues.queue8 = 1;
				}
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
			
				if(cnt_order_TIM16)
				{
				osMessagePut(Line4QueueHandle,(uint16_t)Timer3_ch3_Cap.low_time,0);
				queuestatus.queues.queue4 = 1;
				}
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
			
				if(cnt_order_TIM16)
				{
				osMessagePut(Line10QueueHandle,(uint16_t)Timer3_ch4_Cap.low_time,0);
				queuestatus.queues.queue10 = 1;
				}
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
			
				if(cnt_order_TIM16)
				{
				osMessagePut(Line1QueueHandle,(uint16_t)Timer4_ch1_Cap.low_time,0);
				queuestatus.queues.queue1 = 1;
				}
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
			
				if(cnt_order_TIM16)
				{
				osMessagePut(Line3QueueHandle,(uint16_t)Timer4_ch2_Cap.low_time,0);
				queuestatus.queues.queue3 = 1;
				}
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
			
				if(cnt_order_TIM16)
				{
				osMessagePut(Line2QueueHandle,(uint16_t)Timer4_ch4_Cap.low_time,0);
				queuestatus.queues.queue2 = 1;
				}
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
			
				if(cnt_order_TIM16)
				{
				osMessagePut(Line6QueueHandle,(uint16_t)Timer2_ch2_Cap.low_time,0);
				queuestatus.queues.queue6 = 1;
				}
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
			
				if(cnt_order_TIM16)
				{
				osMessagePut(Line7QueueHandle,(uint16_t)Timer2_ch3_Cap.low_time,0);
				queuestatus.queues.queue7 = 1;
				}
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
			
				if(cnt_order_TIM16)
				{
				osMessagePut(Line5QueueHandle,(uint16_t)Timer2_ch4_Cap.low_time,0);
				queuestatus.queues.queue5 = 1;
				}
		}
	}
	
	
}



/*
	空闲时高电平，ccr配置低电平时间
	ccr: 0~4499 单位：0.01us(10ns)	即最大45us
	4450:44.5us
	4300:43us
*/
void Start_TIM17_OnePulse(uint16_t ccr)
{	
	TIM17->CCR1 = 4499-ccr;//ccr:4450
	// 启用 TIM17 CH1 输出比较通道
	TIM17->CCER |= TIM_CCER_CC1E;  // 设置 CC1E (Channel 1 Output Enable)
	// 启用 TIM17 CH1 输出比较预装载功能
	TIM17->CCMR1 |= TIM_CCMR1_OC1PE;  // 设置 OC1PE (Output Compare Preload Enable)
	// 启动定时器 TIM17
	TIM17->CR1 |= TIM_CR1_CEN;  // 设置 CEN (Counter Enable) 位
	// 启用 TIM17 主输出信号
	TIM17->BDTR |= TIM_BDTR_MOE;  // 设置 MOE (Main Output Enable) 位

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

uint16_t countBits(uint16_t n) {
    uint8_t count = 0;
    while (n) {
        n &= (n - 1);  // 消除最低位的1
        count++;
    }
    return count;
}


				/*
//				if( queuestatus.queues.queue1  )
//				{
//					event[1] = osMessageGet(Line1QueueHandle, 0);
//					if (event[1].status == osEventMessage)
//					{
//						res_store_p[j*100+(i-1)*10+1-1+2].word16=event[1].value.v;
//						print_data[j*100+(i-1)*10+1-1+2].word16=event[1].value.v;
//					}
//				}
//			    if( queuestatus.queues.queue2  )
//				{
//					event[2] = osMessageGet(Line2QueueHandle, 0);
//					if (event[2].status == osEventMessage)
//					{
//						res_store_p[j*100+(i-1)*10+2-1+2].word16=event[2].value.v;
//						print_data[j*100+(i-1)*10+2-1+2].word16=event[2].value.v;
//					}
//				}
//			    if( queuestatus.queues.queue3  )
//				{
//					event[3] = osMessageGet(Line3QueueHandle, 0);
//					if (event[3].status == osEventMessage)
//					{
//						res_store_p[j*100+(i-1)*10+3-1+2].word16=event[3].value.v;
//						print_data[j*100+(i-1)*10+3-1+2].word16=event[3].value.v;
//					}
//				}
//			    if( queuestatus.queues.queue4  )
//				{
//					event[4] = osMessageGet(Line4QueueHandle, 0);
//					if (event[4].status == osEventMessage)
//					{
//						res_store_p[j*100+(i-1)*10+4-1+2].word16=event[4].value.v;
//						print_data[j*100+(i-1)*10+4-1+2].word16=event[4].value.v;
//					}
//				}
//			    if( queuestatus.queues.queue5  )
//				{
//					event[5] = osMessageGet(Line5QueueHandle, 0);
//					if (event[5].status == osEventMessage)
//					{
//						res_store_p[j*100+(i-1)*10+5-1+2].word16=event[5].value.v;
//						print_data[j*100+(i-1)*10+5-1+2].word16=event[5].value.v;
//					}
//				}
//			    if( queuestatus.queues.queue6  )
//				{
//					event[6] = osMessageGet(Line6QueueHandle, 0);
//					if (event[6].status == osEventMessage)
//					{
//						res_store_p[j*100+(i-1)*10+6-1+2].word16=event[6].value.v;
//						print_data[j*100+(i-1)*10+6-1+2].word16=event[6].value.v;
//					}
//				}
//			    if( queuestatus.queues.queue7  )
//				{
//					event[7] = osMessageGet(Line7QueueHandle, 0);
//					if (event[7].status == osEventMessage)
//					{
//						res_store_p[j*100+(i-1)*10+7-1+2].word16=event[7].value.v;
//						print_data[j*100+(i-1)*10+7-1+2].word16=event[7].value.v;
//					}
//				}
//			    if( queuestatus.queues.queue8  )
//				{
//					event[8] = osMessageGet(Line8QueueHandle, 0);
//					if (event[8].status == osEventMessage)
//					{
//						res_store_p[j*100+(i-1)*10+8-1+2].word16=event[8].value.v;
//						print_data[j*100+(i-1)*10+8-1+2].word16=event[8].value.v;
//					}
//				}
//			    if( queuestatus.queues.queue9  )
//				{
//					event[9] = osMessageGet(Line9QueueHandle, 0);
//					if (event[9].status == osEventMessage)
//					{
//						res_store_p[j*100+(i-1)*10+9-1+2].word16=event[9].value.v;
//						print_data[j*100+(i-1)*10+9-1+2].word16=event[9].value.v;
//					}
//				}
//			    if( queuestatus.queues.queue10 )
//				{
//					event[10] = osMessageGet(Line10QueueHandle, 0);
//					if (event[10].status == osEventMessage)
//					{
//						res_store_p[j*100+(i-1)*10+10-1+2].word16=event[10].value.v;
//						print_data[j*100+(i-1)*10+10-1+2].word16=event[10].value.v;
//					}
//				}
			*/
/* USER CODE END Application */

