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

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

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
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

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
//#define DAC1_BASE             0x40007400U
#define DAC_DHR12R1_OFFSET    0x08U

static inline void SetDAC1_CH1_Value(uint32_t Data)
{
    // 强制转换基地址为寄存器结构体指针
    DAC_TypeDef *DACx = (DAC_TypeDef *)DAC1_BASE;
    
    // 直接写入12位右对齐数据寄存器
    DACx->DHR12R1 = (Data & 0xFFF);  // 保留低12位
}

void Start_DAC_DMA_Optimized(DAC_HandleTypeDef *hdac, uint32_t Channel, uint32_t *pData, uint32_t Length)
{
    /* Check if the DAC handle is valid */
    if (hdac == NULL || pData == NULL || Length == 0)
    {
        return;
    }

    /* Get the DMA handle for the specified channel */
    DMA_HandleTypeDef *hdma = (Channel == DAC_CHANNEL_1) ? hdac->DMA_Handle1 : hdac->DMA_Handle2;

    /* Ensure the DMA is disabled before configuration */
    __HAL_DMA_DISABLE(hdma);

    /* Configure the DMA channel */
    hdma->Instance->CCR &= ~DMA_CCR_EN; /* Disable the DMA channel */
    
    /* Set the number of data items */
    hdma->Instance->CNDTR = Length;
    
    /* Set the peripheral address */
    if (Channel == DAC_CHANNEL_1)
    {
        hdma->Instance->CPAR = (uint32_t)&hdac->Instance->DHR12R1;
    }
    else
    {
        hdma->Instance->CPAR = (uint32_t)&hdac->Instance->DHR12R2;
    }
    
    /* Set the memory address */
    hdma->Instance->CMAR = (uint32_t)pData;
    
    /* Configure DMA for memory increment (NO circular mode) */
    hdma->Instance->CCR |= DMA_CCR_MINC;  // 仅保留内存递增
    hdma->Instance->CCR &= ~DMA_CCR_CIRC; // 确保关闭循环模式
    
    /* Set peripheral to memory direction */
    hdma->Instance->CCR |= DMA_CCR_DIR;

    /* Enable the DMA channel */
    __HAL_DMA_ENABLE(hdma);

    /* Enable the DAC DMA request */
    if (Channel == DAC_CHANNEL_1)
    {
        hdac->Instance->CR |= DAC_CR_DMAEN1;
    }
    else
    {
        hdac->Instance->CR |= DAC_CR_DMAEN2;
    }

    /* Enable DAC DMA underrun interrupt */
    if (Channel == DAC_CHANNEL_1)
    {
        __HAL_DAC_ENABLE_IT(hdac, DAC_IT_DMAUDR1);
    }
    else
    {
        __HAL_DAC_ENABLE_IT(hdac, DAC_IT_DMAUDR2);
    }

    /* Start the DAC */
    if (Channel == DAC_CHANNEL_1)
    {
        hdac->Instance->CR |= DAC_CR_EN1;
    }
    else
    {
        hdac->Instance->CR |= DAC_CR_EN2;
    }
}


uint16_t adc_value[10];

//400真实长度，剩下100全是0
uint32_t arr1[500] = {
0,10,21,31,41,51,62,72,82,92,103,113,123,133,144,154,164,174,185,195,205,216,226,236,246,257,267,277,287,298,308,318,328,339,349,359,369,380,390,400,411,421,431,441,452,462,472,482,493,503,513,523,534,544,554,564,575,585,595,606,616,626,636,647,657,667,677,688,698,708,718,729,739,749,759,770,780,790,801,811,821,831,842,852,862,872,883,893,903,913,924,934,944,954,965,975,985,996,1006,1016,1026,1037,1047,1057,1067,1078,1088,1098,1108,1119,1129,1139,1149,1160,1170,1180,1191,1201,1211,1221,1232,1242,1252,1262,1273,1283,1293,1303,1314,1324,1334,1344,1355,1365,1375,1386,1396,1406,1416,1427,1437,1447,1457,1468,1478,1488,1498,1509,1519,1529,1539,1550,1560,1570,1581,1591,1601,1611,1622,1632,1642,1652,1663,1673,1683,1693,1704,1714,1724,1734,1745,1755,1765,1776,1786,1796,1806,1817,1827,1837,1847,1858,1868,1878,1888,1899,1909,1919,1929,1940,1950,1960,1971,1981,1991,2001,2012,2022,2032,2042,2053,2063,2073,2083,2094,2104,2114,2124,2135,2145,2155,2166,2176,2186,2196,2207,2217,2227,2237,2248,2258,2268,2278,2289,2299,2309,2319,2330,2340,2350,2361,2371,2381,2391,2402,2412,2422,2432,2443,2453,2463,2473,2484,2494,2504,2514,2525,2535,2545,2556,2566,2576,2586,2597,2607,2617,2627,2638,2648,2658,2668,2679,2689,2699,2709,2720,2730,2740,2751,2761,2771,2781,2792,2802,2812,2822,2833,2843,2853,2863,2874,2884,2894,2904,2915,2925,2935,2946,2956,2966,2976,2987,2997,3007,3017,3028,3038,3048,3058,3069,3079,3089,3099,3110,3120,3130,3141,3151,3161,3171,3182,3192,3202,3212,3223,3233,3243,3253,3264,3274,3284,3294,3305,3315,3325,3336,3346,3356,3366,3377,3387,3397,3407,3418,3428,3438,3448,3459,3469,3479,3489,3500,3510,3520,3531,3541,3551,3561,3572,3582,3592,3602,3613,3623,3633,3643,3654,3664,3674,3684,3695,3705,3715,3726,3736,3746,3756,3767,3777,3787,3797,3808,3818,3828,3838,3849,3859,3869,3879,3890,3900,3910,3921,3931,3941,3951,3962,3972,3982,3992,4003,4013,4023,4033,4044,4054,4064,4074,4085,4095
};//均匀
uint32_t arr2[500] = {
0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,41,43,45,48,50,52,54,57,59,62,64,67,70,72,75,78,81,84,87,90,93,96,99,102,105,109,112,115,119,122,126,130,133,137,141,145,149,153,156,161,165,169,173,177,181,186,190,195,199,204,208,213,218,222,227,232,237,242,247,252,257,262,268,273,278,284,289,294,300,306,311,317,323,328,334,340,346,352,358,364,370,377,383,389,396,402,408,415,421,428,435,441,448,455,462,469,476,483,490,497,504,511,519,526,533,541,548,556,563,571,579,586,594,602,610,618,626,634,642,650,658,667,675,683,692,700,709,717,726,735,743,752,761,770,779,788,797,806,815,824,833,843,852,861,871,880,890,899,909,919,929,938,948,958,968,978,988,998,1008,1019,1029,1039,1050,1060,1070,1081,1092,1102,1113,1124,1134,1145,1156,1167,1178,1189,1200,1211,1222,1234,1245,1256,1268,1279,1291,1302,1314,1325,1337,1349,1361,1373,1384,1396,1408,1421,1433,1445,1457,1469,1482,1494,1506,1519,1531,1544,1557,1569,1582,1595,1608,1621,1633,1646,1659,1673,1686,1699,1712,1725,1739,1752,1766,1779,1793,1806,1820,1834,1847,1861,1875,1889,1903,1917,1931,1945,1959,1974,1988,2002,2017,2031,2046,2060,2075,2089,2104,2119,2134,2148,2163,2178,2193,2208,2223,2238,2254,2269,2284,2300,2315,2330,2346,2362,2377,2393,2409,2424,2440,2456,2472,2488,2504,2520,2536,2552,2569,2585,2601,2618,2634,2650,2667,2684,2700,2717,2734,2750,2767,2784,2801,2818,2835,2852,2869,2887,2904,2921,2939,2956,2973,2991,3009,3026,3044,3062,3079,3097,3115,3133,3151,3169,3187,3205,3223,3242,3260,3278,3297,3315,3334,3352,3371,3389,3408,3427,3446,3464,3483,3502,3521,3540,3560,3579,3598,3617,3637,3656,3675,3695,3714,3734,3753,3773,3793,3813,3833,3852,3872,3892,3912,3932,3953,3973,3993,4013,4034,4054,4074,4095
};//下凹
uint32_t arr3[500] = {
0,21,41,61,82,102,122,142,163,183,203,223,243,262,282,302,322,342,361,381,400,420,439,458,478,497,516,535,555,574,593,612,631,649,668,687,706,724,743,761,780,798,817,835,853,872,890,908,926,944,962,980,998,1016,1033,1051,1069,1086,1104,1122,1139,1156,1174,1191,1208,1226,1243,1260,1277,1294,1311,1328,1345,1361,1378,1395,1411,1428,1445,1461,1477,1494,1510,1526,1543,1559,1575,1591,1607,1623,1639,1655,1671,1686,1702,1718,1733,1749,1765,1780,1795,1811,1826,1841,1857,1872,1887,1902,1917,1932,1947,1961,1976,1991,2006,2020,2035,2049,2064,2078,2093,2107,2121,2136,2150,2164,2178,2192,2206,2220,2234,2248,2261,2275,2289,2302,2316,2329,2343,2356,2370,2383,2396,2409,2422,2436,2449,2462,2474,2487,2500,2513,2526,2538,2551,2564,2576,2589,2601,2613,2626,2638,2650,2662,2674,2687,2699,2711,2722,2734,2746,2758,2770,2781,2793,2804,2816,2827,2839,2850,2861,2873,2884,2895,2906,2917,2928,2939,2950,2961,2971,2982,2993,3003,3014,3025,3035,3045,3056,3066,3076,3087,3097,3107,3117,3127,3137,3147,3157,3166,3176,3186,3196,3205,3215,3224,3234,3243,3252,3262,3271,3280,3289,3298,3307,3316,3325,3334,3343,3352,3360,3369,3378,3386,3395,3403,3412,3420,3428,3437,3445,3453,3461,3469,3477,3485,3493,3501,3509,3516,3524,3532,3539,3547,3554,3562,3569,3576,3584,3591,3598,3605,3612,3619,3626,3633,3640,3647,3654,3660,3667,3674,3680,3687,3693,3699,3706,3712,3718,3725,3731,3737,3743,3749,3755,3761,3767,3772,3778,3784,3789,3795,3801,3806,3811,3817,3822,3827,3833,3838,3843,3848,3853,3858,3863,3868,3873,3877,3882,3887,3891,3896,3900,3905,3909,3914,3918,3922,3926,3930,3934,3939,3942,3946,3950,3954,3958,3962,3965,3969,3973,3976,3980,3983,3986,3990,3993,3996,3999,4002,4005,4008,4011,4014,4017,4020,4023,4025,4028,4031,4033,4036,4038,4041,4043,4045,4047,4050,4052,4054,4056,4058,4060,4062,4063,4065,4067,4069,4070,4072,4073,4075,4076,4078,4079,4080,4081,4083,4084,4085,4086,4087,4088,4089,4090,4091,4092,4093,4094,4095,4096,4097,4098,4099,4100,4101,4102,4103,4104,4095
};//上凸

/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	char usb_buff[128]={0};
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,1);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc_value, 10);
	

	HAL_TIM_Base_Start(&htim3);//DAC必须开启定时器事件触发
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
	osDelay(500);
  /* Infinite loop */
	for(;;)
	{

		HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
		sprintf(usb_buff,"PB2 adc_value:%d\r\n",adc_value[0]);
		CDC_Transmit_FS((uint8_t *)usb_buff,strlen(usb_buff));
		SetDAC1_CH1_Value(0);
		osDelay(1);
		SetDAC1_CH1_Value(4095);//高电平时间代表函数运行时间
//		HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_2, (uint32_t*)arr1, 500, DAC_ALIGN_12B_R);//DMA要配置成WORD  DAC极限频率15MHz
		Start_DAC_DMA_Optimized(&hdac1, DAC_CHANNEL_2, (uint32_t*)arr1, 500);
		SetDAC1_CH1_Value(0);
		osDelay(1);
	}
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

