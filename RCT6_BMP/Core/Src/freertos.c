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
#include "usbd_cdc_if.h"
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

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void LED_blink_1_4(void);
float LowHighFilter(float current, float fc, float fs, 
                   float* prev_lowpass_output, 
                   float* prev_highpass_output, 
                   float* prev_input);
float LowPassFilter_direct(float current,float alpha, float* prev_lowpass_output);

void LPF_Init(LowPassFilter *lpf, float fc, float fs);
float LPF_Compute(LowPassFilter *lpf, float x);
void HPF_Init(HighPassFilter *hpf, float fc, float fs);
float HPF_Compute(HighPassFilter *hpf, float x);
				   
				   
				   
				   
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void LedTask(void const * argument);

extern void MX_USB_DEVICE_Init(void);
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
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 1024);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of ledTask */
  osThreadDef(ledTask, LedTask, osPriorityLow, 0, 128);
  ledTaskHandle = osThreadCreate(osThread(ledTask), NULL);

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
#define PRINT_MESSAGE 0



float Prev_low_value[4];
float Prev_high_value[4];
float Prev_input_value[4];

float Prev_value11=0.0;
float Prev_value22=0.0;
float Prev_value33=0.0;
float Prev_value44=0.0;

LowPassFilter LFilter[4];
HighPassFilter HFilter[4];
float HighPass_value[4];

Protocol_struct Highpass_fdata;

BMP280_Device bmp_dev1;
BMP280_Device bmp_dev2;
BMP280_Device bmp_dev3;
BMP280_Device bmp_dev4;

Word_union Binary_data[4];
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
	char usb_TxBuf[1024]={0};
	
	bmp_dev1.hspi = &hspi1;
    bmp_dev1.cs_port = SPI_CS1_GPIO_Port;
    bmp_dev1.cs_pin = SPI_CS1_Pin;
	
	bmp_dev2.hspi = &hspi1;
    bmp_dev2.cs_port = SPI_CS2_GPIO_Port;
    bmp_dev2.cs_pin = SPI_CS2_Pin;
	
	bmp_dev3.hspi = &hspi1;
    bmp_dev3.cs_port = SPI_CS3_GPIO_Port;
    bmp_dev3.cs_pin = SPI_CS3_Pin;
	
	bmp_dev4.hspi = &hspi1;
//    bmp_dev4.cs_port = SPI_CS4_GPIO_Port;
//    bmp_dev4.cs_pin = SPI_CS4_Pin;
	bmp_dev4.cs_port = GPIOA;
	bmp_dev4.cs_pin = GPIO_PIN_2;
	
	while(BMP280_Init(&bmp_dev1) != HAL_OK)
	{
		sprintf(usb_TxBuf,"dev1初始化失败\r\n");
		CDC_Transmit_FS((uint8_t *)usb_TxBuf,strlen(usb_TxBuf));
		osDelay(500);
	}
	while(BMP280_Init(&bmp_dev2) != HAL_OK)
	{
		sprintf(usb_TxBuf,"dev2初始化失败\r\n");
		CDC_Transmit_FS((uint8_t *)usb_TxBuf,strlen(usb_TxBuf));
		osDelay(500);
	}
	while(BMP280_Init(&bmp_dev3) != HAL_OK)
	{
		sprintf(usb_TxBuf,"dev3初始化失败\r\n");
		CDC_Transmit_FS((uint8_t *)usb_TxBuf,strlen(usb_TxBuf));
		osDelay(500);
	}
	while(BMP280_Init(&bmp_dev4) != HAL_OK)
	{
		sprintf(usb_TxBuf,"dev4初始化失败\r\n");
		CDC_Transmit_FS((uint8_t *)usb_TxBuf,strlen(usb_TxBuf));
		osDelay(500);
	}
	
	Highpass_fdata.head[0]=0x55;//帧头
	Highpass_fdata.head[1]=0xAA;
	Highpass_fdata.head[2]=0xBB;
	Highpass_fdata.head[3]=0xCC;
	
	Highpass_fdata.tail[0]=0xAA;//帧尾
	Highpass_fdata.tail[1]=0x55;
	Highpass_fdata.tail[2]=0x66;
	Highpass_fdata.tail[3]=0x77;
	
	HFilter[0].x_prev=100.0;
	HFilter[1].x_prev=100.0;
	HFilter[2].x_prev=100.0;
	HFilter[3].x_prev=100.0;
	
	LPF_Init(&LFilter[0],10,50);
	LPF_Init(&LFilter[1],10,50);
	LPF_Init(&LFilter[2],10,50);
	LPF_Init(&LFilter[3],10,50);
						   
	HPF_Init(&HFilter[0],0.5,50);
	HPF_Init(&HFilter[1],0.5,50);
	HPF_Init(&HFilter[2],0.5,50);
	HPF_Init(&HFilter[3],0.5,50);
	
	TickType_t xLastWakeTime = xTaskGetTickCount();
  /* Infinite loop */
	for(;;)
	{
		#if PRINT_MESSAGE == 1
		  sprintf(usb_TxBuf,"\r\n\r\n");
		  /* 读取压力和温度 */
			if (BMP280_ReadPressureTemperature(&bmp_dev1) == HAL_OK) {
				sprintf(usb_TxBuf+strlen(usb_TxBuf),"dev1\r\n");
				sprintf(usb_TxBuf+strlen(usb_TxBuf),"温度: %.2f °C\r\n", bmp_dev1.temperature);
				sprintf(usb_TxBuf+strlen(usb_TxBuf),"压力: %.2f kPa\r\n", bmp_dev1.pressure);
				sprintf(usb_TxBuf+strlen(usb_TxBuf),"海拔: %.2f m\r\n", bmp_dev1.altitude);
				sprintf(usb_TxBuf+strlen(usb_TxBuf),"------------------------\r\n");
			} else {
				sprintf(usb_TxBuf,"dev1读取数据失败\r\n");
			}
			if (BMP280_ReadPressureTemperature(&bmp_dev2) == HAL_OK) {
				sprintf(usb_TxBuf+strlen(usb_TxBuf),"dev2\r\n");
				sprintf(usb_TxBuf+strlen(usb_TxBuf),"温度: %.2f °C\r\n", bmp_dev2.temperature);
				sprintf(usb_TxBuf+strlen(usb_TxBuf),"压力: %.2f kPa\r\n", bmp_dev2.pressure);
				sprintf(usb_TxBuf+strlen(usb_TxBuf),"海拔: %.2f m\r\n", bmp_dev2.altitude);
				sprintf(usb_TxBuf+strlen(usb_TxBuf),"------------------------\r\n");
			} else {
				sprintf(usb_TxBuf,"dev2读取数据失败\r\n");
			}
			if (BMP280_ReadPressureTemperature(&bmp_dev3) == HAL_OK) {
				sprintf(usb_TxBuf+strlen(usb_TxBuf),"dev3\r\n");
				sprintf(usb_TxBuf+strlen(usb_TxBuf),"温度: %.2f °C\r\n", bmp_dev3.temperature);
				sprintf(usb_TxBuf+strlen(usb_TxBuf),"压力: %.2f kPa\r\n", bmp_dev3.pressure);
				sprintf(usb_TxBuf+strlen(usb_TxBuf),"海拔: %.2f m\r\n", bmp_dev3.altitude);
				sprintf(usb_TxBuf+strlen(usb_TxBuf),"------------------------\r\n");
			} else {
				sprintf(usb_TxBuf,"dev3读取数据失败\r\n");
			}
			if (BMP280_ReadPressureTemperature(&bmp_dev4) == HAL_OK) {
				sprintf(usb_TxBuf+strlen(usb_TxBuf),"dev4\r\n");
				sprintf(usb_TxBuf+strlen(usb_TxBuf),"温度: %.2f °C\r\n", bmp_dev4.temperature);
				sprintf(usb_TxBuf+strlen(usb_TxBuf),"压力: %.2f kPa\r\n", bmp_dev4.pressure);
				sprintf(usb_TxBuf+strlen(usb_TxBuf),"海拔: %.2f m\r\n", bmp_dev4.altitude);
				sprintf(usb_TxBuf+strlen(usb_TxBuf),"------------------------\r\n");
			} else {
				sprintf(usb_TxBuf,"dev4读取数据失败\r\n");
			}
			CDC_Transmit_FS((uint8_t *)usb_TxBuf,strlen(usb_TxBuf));
		#endif
			
		#if PRINT_MESSAGE == 0
			BMP280_ReadPressureTemperature(&bmp_dev1);
			BMP280_ReadPressureTemperature(&bmp_dev2);
			BMP280_ReadPressureTemperature(&bmp_dev3);
			BMP280_ReadPressureTemperature(&bmp_dev4);
			
			
			Highpass_fdata.value[0] = HPF_Compute(&HFilter[0], bmp_dev1.pressure);
			Highpass_fdata.value[1] = HPF_Compute(&HFilter[1], bmp_dev2.pressure);
			Highpass_fdata.value[2] = HPF_Compute(&HFilter[2], bmp_dev3.pressure);
			Highpass_fdata.value[3] = HPF_Compute(&HFilter[3], bmp_dev4.pressure);
			
			
			Highpass_fdata.value[0] = LPF_Compute(&LFilter[0], Highpass_fdata.value[0]);
			Highpass_fdata.value[1] = LPF_Compute(&LFilter[1], Highpass_fdata.value[1]);
			Highpass_fdata.value[2] = LPF_Compute(&LFilter[2], Highpass_fdata.value[2]);
			Highpass_fdata.value[3] = LPF_Compute(&LFilter[3], Highpass_fdata.value[3]);
			
//			Highpass_fdata.value[0]=LowPassFilter_direct(Highpass_fdata.value[0],0.2,&Prev_value11);
//			Highpass_fdata.value[1]=LowPassFilter_direct(Highpass_fdata.value[1],0.2,&Prev_value22);
//			Highpass_fdata.value[2]=LowPassFilter_direct(Highpass_fdata.value[2],0.2,&Prev_value33);
//			Highpass_fdata.value[3]=LowPassFilter_direct(Highpass_fdata.value[3],0.2,&Prev_value44);
			
//			Binary_data[0].word16=(uint16_t)(HighPass_value1*100.0);
//			sprintf(usb_TxBuf,"%f,%f\r\n",bmp_dev1.pressure,LowPass_value1);
			sprintf(usb_TxBuf,"%f,%f,%f,%f,%f,%f,%f,%f\r\n",bmp_dev1.pressure,bmp_dev2.pressure,bmp_dev3.pressure,bmp_dev4.pressure,
			Highpass_fdata.value[0],Highpass_fdata.value[1],Highpass_fdata.value[2],Highpass_fdata.value[3]
			);
			
//			CDC_Transmit_FS((uint8_t *)usb_TxBuf,strlen(usb_TxBuf));
			CDC_Transmit_FS((uint8_t *)(&Highpass_fdata),24);
		#endif
		
//		HAL_GPIO_TogglePin(SPI_CS1_GPIO_Port,SPI_CS1_Pin);
//		HAL_GPIO_TogglePin(SPI_CS2_GPIO_Port,SPI_CS2_Pin);
//		HAL_GPIO_TogglePin(SPI_CS3_GPIO_Port,SPI_CS3_Pin);
//		HAL_GPIO_TogglePin(SPI_CS4_GPIO_Port,SPI_CS4_Pin);
//		
//		HAL_GPIO_TogglePin(SPI_CS5_GPIO_Port,SPI_CS5_Pin);
//		HAL_GPIO_TogglePin(SPI_CS6_GPIO_Port,SPI_CS6_Pin);
//		HAL_GPIO_TogglePin(SPI_CS7_GPIO_Port,SPI_CS7_Pin);
//		HAL_GPIO_TogglePin(SPI_CS8_GPIO_Port,SPI_CS8_Pin);
//		
//		HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_2);
		
//		LED_blink_1_4();
//		HAL_GPIO_TogglePin(LED5_GPIO_Port,LED5_Pin);
//		HAL_GPIO_TogglePin(LED6_GPIO_Port,LED6_Pin);
//		HAL_GPIO_TogglePin(LED7_GPIO_Port,LED7_Pin);
//		HAL_GPIO_TogglePin(LED8_GPIO_Port,LED8_Pin);
		
		
//		osDelay(20);
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20));
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
	TickType_t xLastWakeTime = xTaskGetTickCount();
  /* Infinite loop */
	for(;;)
	{
		if(Highpass_fdata.value[0]>=0.1)	HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,0); else	HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,1);
		if(Highpass_fdata.value[1]>=0.1)	HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,0); else	HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,1);
		if(Highpass_fdata.value[2]>=0.1)	HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,0); else	HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,1);
		if(Highpass_fdata.value[3]>=0.1)	HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin,0); else	HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin,1);
		
//		osDelay(10);
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
	}
  /* USER CODE END LedTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void LED_blink_1_4(void)
{
	static uint8_t cnt=0;
	switch(cnt)
	{
		case 0:
		{
			HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,0);
			HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,1);
			HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,1);
			HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin,1);
			break;
		}
		case 1:
		{
			HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,1);
			HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,0);
			HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,1);
			HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin,1);
			break;
		}
		case 2:
		{
			HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,1);
			HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,1);
			HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,0);
			HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin,1);
			break;
		}
		case 3:
		{
			HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,1);
			HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,1);
			HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,1);
			HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin,0);
			break;
		}
		default:
			HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,1);	
			HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,1);
			HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,1);
			HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin,1);
	}
	if(cnt<3)cnt++;
	else cnt=0;
	
}



/**
 * @brief 一阶IIR低通滤波函数（浮点数版）
 * @param current 当前传感器采样值（已转换为物理量，如电压）
 * @param fc 截止频率（单位：Hz，需小于fs/2）
 * @param fs 采样频率（单位：Hz）
 * @param prev_lowpass_output 前一次低通滤波输出值的指针（输入/输出）
 * @param prev_highpass_output 前一次高通滤波输出值的指针（输入/输出）
 * @param prev_input 前一次输入值的指针（输入/输出）
 * @param highpass_output 高通滤波输出值的指针（输出）
 * @return 低通滤波后的值
 */
float LowHighFilter(float current, float fc, float fs, 
                   float* prev_lowpass_output, 
                   float* prev_highpass_output, 
                   float* prev_input) {
    // 计算采样周期
    float Ts = 1.0f / fs;
    
    // 计算正确的滤波系数α
    float alpha = (2.0f * 3.1416f * fc * Ts) / (1.0f + 2.0f * 3.1416f * fc * Ts);

    // 低通滤波计算
    float lowpass_output = alpha * current + (1.0f - alpha) * (*prev_lowpass_output);
    
    // 高通滤波（单独实现，不依赖低通结果）
    float high_alpha = alpha / (alpha + 2.0f);
    float hp_output = high_alpha * (*prev_highpass_output + current - *prev_input);
    
    // 更新输出和状态
    *prev_highpass_output = hp_output;
    *prev_input = current;
    *prev_lowpass_output = lowpass_output;

    return lowpass_output;
}


/**
 * @brief 一阶IIR低通滤波函数（浮点数版）直接指定α
 * @param current 当前传感器采样值（已转换为物理量，如电压）
 * @param fc 截止频率（单位：Hz，需小于fs/2）
 * @param fs 采样频率（单位：Hz）
 * @return 滤波后的值
 */
float LowPassFilter_direct(float current,float alpha, float* prev_lowpass_output) {

    // 滤波计算：当前输出 = α*当前输入 + (1-α)*前一次输出    α越小信号越缓慢
    float output = alpha * current + (1.0f - alpha) * (*prev_lowpass_output);

    // 更新前一次输出
    (*prev_lowpass_output) = output;

    return output;
}



/**
 * 初始化低通滤波器
 * @param lpf 低通滤波器结构体指针
 * @param fc 截止频率(Hz)
 * @param fs 采样频率(Hz)
 */
void LPF_Init(LowPassFilter *lpf, float fc, float fs) {
    lpf->fc = fc;
    lpf->fs = fs;
    float tau = 1.0f / (2.0f * 3.1415926f * fc);  // 时间常数
    float dt = 1.0f / fs;                           // 采样周期
    lpf->alpha = dt / (tau + dt);                   // 计算滤波系数
    lpf->y_prev = 0.0f;                             // 初始化历史值
}

/**
 * 计算低通滤波器输出
 * @param lpf 低通滤波器结构体指针
 * @param x 当前输入值
 * @return 滤波后的输出值
 */
float LPF_Compute(LowPassFilter *lpf, float x) {
    float y = lpf->alpha * x + (1.0f - lpf->alpha) * lpf->y_prev;
    lpf->y_prev = y;  // 更新历史值
    return y;
}

/**
 * 初始化高通滤波器
 * @param hpf 高通滤波器结构体指针
 * @param fc 截止频率(Hz)
 * @param fs 采样频率(Hz)
 */
void HPF_Init(HighPassFilter *hpf, float fc, float fs) {
    hpf->fc = fc;
    hpf->fs = fs;
    float tau = 1.0f / (2.0f * 3.1415926f * fc);  // 时间常数
    float dt = 1.0f / fs;                           // 采样周期
    hpf->alpha = tau / (tau + dt);                  // 计算滤波系数
    hpf->x_prev = 0.0f;                             // 初始化历史输入
    hpf->y_prev = 0.0f;                             // 初始化历史输出
}

/**
 * 计算高通滤波器输出
 * @param hpf 高通滤波器结构体指针
 * @param x 当前输入值
 * @return 滤波后的输出值
 */
float HPF_Compute(HighPassFilter *hpf, float x) {
    float y = hpf->alpha * (hpf->y_prev + x - hpf->x_prev);
	
    hpf->x_prev = x;      // 更新历史输入
    hpf->y_prev = y;      // 更新历史输出
	
	if(y<0)y=0;
	
    return y;
}
/* USER CODE END Application */

