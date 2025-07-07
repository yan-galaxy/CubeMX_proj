/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "spi.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	char usb_TxBuf[512]={0};
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	BMP280_Device bmp_dev1;
	BMP280_Device bmp_dev2;
	BMP280_Device bmp_dev3;
	BMP280_Device bmp_dev4;
	BMP280_Device bmp_dev5;
	BMP280_Device bmp_dev6;
	BMP280_Device bmp_dev7;
	BMP280_Device bmp_dev8;
	
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
    bmp_dev4.cs_port = SPI_CS4_GPIO_Port;
    bmp_dev4.cs_pin = SPI_CS4_Pin;
	
	bmp_dev5.hspi = &hspi1;
    bmp_dev5.cs_port = SPI_CS5_GPIO_Port;
    bmp_dev5.cs_pin = SPI_CS5_Pin;
	
	bmp_dev6.hspi = &hspi1;
    bmp_dev6.cs_port = SPI_CS6_GPIO_Port;
    bmp_dev6.cs_pin = SPI_CS6_Pin;
	
	bmp_dev7.hspi = &hspi1;
    bmp_dev7.cs_port = SPI_CS7_GPIO_Port;
    bmp_dev7.cs_pin = SPI_CS7_Pin;
	
	bmp_dev8.hspi = &hspi1;
    bmp_dev8.cs_port = SPI_CS8_GPIO_Port;
    bmp_dev8.cs_pin = SPI_CS8_Pin;
	
//	while(BMP280_Init(&bmp_dev1) != HAL_OK)
//	{
//		sprintf(usb_TxBuf,"dev1初始化失败\r\n");
//		CDC_Transmit_FS((uint8_t *)usb_TxBuf,strlen(usb_TxBuf));
//		HAL_Delay(500);
//	}
//	while(BMP280_Init(&bmp_dev2) != HAL_OK)
//	{
//		sprintf(usb_TxBuf,"dev2初始化失败\r\n");
//		CDC_Transmit_FS((uint8_t *)usb_TxBuf,strlen(usb_TxBuf));
//		HAL_Delay(500);
//	}
//	while(BMP280_Init(&bmp_dev3) != HAL_OK)
//	{
//		sprintf(usb_TxBuf,"dev3初始化失败\r\n");
//		CDC_Transmit_FS((uint8_t *)usb_TxBuf,strlen(usb_TxBuf));
//		HAL_Delay(500);
//	}
//	while(BMP280_Init(&bmp_dev4) != HAL_OK)
//	{
//		sprintf(usb_TxBuf,"dev4初始化失败\r\n");
//		CDC_Transmit_FS((uint8_t *)usb_TxBuf,strlen(usb_TxBuf));
//		HAL_Delay(500);
//	}
	while(BMP280_Init(&bmp_dev5) != HAL_OK)
	{
		sprintf(usb_TxBuf,"dev5初始化失败\r\n");
		CDC_Transmit_FS((uint8_t *)usb_TxBuf,strlen(usb_TxBuf));
		HAL_Delay(500);
	}
	
  while (1)
  {
//	  sprintf(usb_TxBuf,"Frame %d: \r\n",123);
//	  CDC_Transmit_FS((uint8_t *)usb_TxBuf, strlen((char *)usb_TxBuf));//USB CDC
//	  HAL_UART_Transmit(&huart2,(uint8_t *)usb_TxBuf,strlen(usb_TxBuf),0xffff);
	  
	  /* 读取压力和温度 */
		if (BMP280_ReadPressureTemperature(&bmp_dev1) == HAL_OK) {
			sprintf(usb_TxBuf,"dev1\r\n");
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
		if (BMP280_ReadPressureTemperature(&bmp_dev5) == HAL_OK) {
			sprintf(usb_TxBuf+strlen(usb_TxBuf),"dev5\r\n");
			sprintf(usb_TxBuf+strlen(usb_TxBuf),"温度: %.2f °C\r\n", bmp_dev5.temperature);
			sprintf(usb_TxBuf+strlen(usb_TxBuf),"压力: %.2f kPa\r\n", bmp_dev5.pressure);
			sprintf(usb_TxBuf+strlen(usb_TxBuf),"海拔: %.2f m\r\n", bmp_dev5.altitude);
			sprintf(usb_TxBuf+strlen(usb_TxBuf),"------------------------\r\n");
		} else {
			sprintf(usb_TxBuf,"dev5读取数据失败\r\n");
		}
//		HAL_GPIO_TogglePin(SPI_CS3_GPIO_Port,SPI_CS3_Pin);
//		HAL_GPIO_TogglePin(SPI_CS4_GPIO_Port,SPI_CS4_Pin);
		
		HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
		HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
		HAL_GPIO_TogglePin(LED3_GPIO_Port,LED3_Pin);
		HAL_GPIO_TogglePin(LED4_GPIO_Port,LED4_Pin);
		HAL_GPIO_TogglePin(LED5_GPIO_Port,LED5_Pin);
		HAL_GPIO_TogglePin(LED6_GPIO_Port,LED6_Pin);
		HAL_GPIO_TogglePin(LED7_GPIO_Port,LED7_Pin);
		HAL_GPIO_TogglePin(LED8_GPIO_Port,LED8_Pin);
		
		CDC_Transmit_FS((uint8_t *)usb_TxBuf,strlen(usb_TxBuf));
	  HAL_Delay(500);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
