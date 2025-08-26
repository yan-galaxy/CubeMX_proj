/* BMP280.c - 功能实现 */
#include "BMP280.h"
#include <math.h>
#include "main.h"
//#include "cmsis_os.h"

/* SPI读写辅助函数 */
static HAL_StatusTypeDef BMP280_SPI_Write(BMP280_Device *dev, uint8_t reg_addr, uint8_t data) {

	uint8_t txData[2] = {reg_addr & 0x7F, data};  // 写操作地址最高位为0
					   
	Input_74HC595_CH64(0xFFFFFFFFFFFFFFFF & (~( (unsigned long long)(0x0000000000000001)<<(63-dev->cs_index) )) 	);//CS置 低 电平
//	Input_74HC595_CH8(0xFF & (~( 0x01<<(7-dev->cs_index) )) 	);//CS置 低 电平
//    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
	
    HAL_StatusTypeDef status = HAL_SPI_Transmit(dev->hspi, txData, 2, 100);
	
//    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
//	Input_74HC595_CH8(0xFF);//CS置 高 电平
	Input_74HC595_CH64(0xFFFFFFFFFFFFFFFF);//CS置 高 电平
    
//	HAL_StatusTypeDef status=0;
    return status;
}
//一次赋值八个片选，选中的那个为0，其余为1。效率低
static HAL_StatusTypeDef BMP280_SPI_Read(BMP280_Device *dev, uint8_t reg_addr, uint8_t *data, uint16_t len) {
    uint8_t tx_buf[1];
    tx_buf[0] = reg_addr | 0x80;  // 设置最高位为1（读操作）
    
	Input_74HC595_CH64(0xFFFFFFFFFFFFFFFF & (~( (unsigned long long)(0x0000000000000001)<<(63-dev->cs_index) )) 	);//CS置 低 电平
//	Input_74HC595_CH8(0xFF & (~( 0x01<<(7-dev->cs_index) )) 	);//CS置 低 电平
//    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
	
    HAL_SPI_Transmit(dev->hspi, tx_buf, 1, 100);
    HAL_StatusTypeDef status = HAL_SPI_Receive(dev->hspi, data, len, 100);
	
//    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
//	Input_74HC595_CH8(0xFF);//CS置 高 电平
	Input_74HC595_CH64(0xFFFFFFFFFFFFFFFF);//CS置 高 电平
    
//	HAL_StatusTypeDef status=0;
    return status;
}

//移位寄存器一位一位移动
static HAL_StatusTypeDef BMP280_SPI_Read_optimized(BMP280_Device *dev,uint8_t dev_index, uint8_t reg_addr, uint8_t *data, uint16_t len) {

    uint8_t tx_buf[1];
    tx_buf[0] = reg_addr | 0x80;  // 设置最高位为1（读操作）
	
	
    if(dev_index==0)
		Input_74HC595(0);//CS置 低 电平
	else
		Input_74HC595(1);//CS置 高 电平
	
    HAL_SPI_Transmit(dev->hspi, tx_buf, 1, 100);
    HAL_StatusTypeDef status = HAL_SPI_Receive(dev->hspi, data, len, 100);
	
//	HAL_StatusTypeDef status=0;
    return status;
}


/* 初始化BMP280 HAL_StatusTypeDef */
HAL_StatusTypeDef BMP280_Init(BMP280_Device *dev) {
    
    /* 复位设备 */
    uint8_t reset_data = 0xB6;
    BMP280_SPI_Write(dev, BMP280_RESET, reset_data);
    HAL_Delay(10);
    
    /* 读取芯片ID */
    HAL_StatusTypeDef status = BMP280_ReadChipID(dev);
    if (status != HAL_OK || dev->chip_id != 0x58) {
        return HAL_ERROR;
    }
    
    /* 读取校准参数 */
    status = BMP280_ReadCalibrationParams(dev);
    if (status != HAL_OK) {
        return HAL_ERROR;
    }
	
    /* 设置模式：正常模式，过采样16X压力，2X温度，滤波器4 */
    status = BMP280_SetMode(dev, BMP280_OSRS_T_2X, BMP280_OSRS_P_16X, BMP280_MODE_NORMAL);
    if (status != HAL_OK) {
        return HAL_ERROR;
    }
    
    /* 设置配置：滤波器4，待机时间62.5ms */
    status = BMP280_SetConfig(dev, BMP280_FILTER_4, BMP280_T_STANDBY_62_5MS);
    
    return status;
}

/* 读取芯片ID */
HAL_StatusTypeDef BMP280_ReadChipID(BMP280_Device *dev) {
    uint8_t data,status;
	status=BMP280_SPI_Read(dev, BMP280_CHIP_ID, &data, 1);
	dev->chip_id = data;
    return status;
}

/* 软复位设备 */
HAL_StatusTypeDef BMP280_SoftReset(BMP280_Device *dev) {
    uint8_t reset_data = 0xB6;
    return BMP280_SPI_Write(dev, BMP280_RESET, reset_data);
}

/* 设置工作模式 */
HAL_StatusTypeDef BMP280_SetMode(BMP280_Device *dev, uint8_t osrs_t, uint8_t osrs_p, uint8_t mode) {
    uint8_t ctrl_meas = (osrs_t << 5) | (osrs_p << 2) | mode;
    return BMP280_SPI_Write(dev, BMP280_CTRL_MEAS, ctrl_meas);
}

/* 设置配置参数 */
HAL_StatusTypeDef BMP280_SetConfig(BMP280_Device *dev, uint8_t filter, uint8_t t_standby) {
    uint8_t config = (t_standby << 5) | (filter << 2);
    return BMP280_SPI_Write(dev, BMP280_CONFIG, config);
}

/* 读取校准参数 */
HAL_StatusTypeDef BMP280_ReadCalibrationParams(BMP280_Device *dev) {
    uint8_t buffer[24];
    HAL_StatusTypeDef status = BMP280_SPI_Read(dev, BMP280_DIG_T1_LSB, buffer, 24);
    
    if (status != HAL_OK) {
        return HAL_ERROR;
    }
    
    /* 解析校准参数 */
    dev->dig_T1 = (buffer[0] | (buffer[1] << 8));
    dev->dig_T2 = (buffer[2] | (buffer[3] << 8));
    dev->dig_T3 = (buffer[4] | (buffer[5] << 8));
    dev->dig_P1 = (buffer[6] | (buffer[7] << 8));
    dev->dig_P2 = (buffer[8] | (buffer[9] << 8));
    dev->dig_P3 = (buffer[10] | (buffer[11] << 8));
    dev->dig_P4 = (buffer[12] | (buffer[13] << 8));
    dev->dig_P5 = (buffer[14] | (buffer[15] << 8));
    dev->dig_P6 = (buffer[16] | (buffer[17] << 8));
    dev->dig_P7 = (buffer[18] | (buffer[19] << 8));
    dev->dig_P8 = (buffer[20] | (buffer[21] << 8));
    dev->dig_P9 = (buffer[22] | (buffer[23] << 8));
    
    return HAL_OK;
}

/* 读取压力和温度数据并计算 */
HAL_StatusTypeDef BMP280_ReadPressureTemperature(BMP280_Device *dev,uint8_t dev_index) {
    uint8_t data[6];
//    HAL_StatusTypeDef status = BMP280_SPI_Read(dev, BMP280_PRESSURE_MSB, data, 6);
	HAL_StatusTypeDef status = BMP280_SPI_Read_optimized(dev,dev_index, BMP280_PRESSURE_MSB, data, 6);
    
    if (status != HAL_OK) {
        return HAL_ERROR;
    }
    
    /* 组合20位压力和温度数据 */
    int32_t adc_p = ((int32_t)data[0] << 12) | ((int32_t)data[1] << 4) | (data[2] >> 4);
    int32_t adc_t = ((int32_t)data[3] << 12) | ((int32_t)data[4] << 4) | (data[5] >> 4);
    
    /* 计算温度（保持不变） */
    int32_t var1, var2;
    var1 = ((((adc_t >> 3) - ((int32_t)dev->dig_T1 << 1)) ) * ((int32_t)dev->dig_T2)) >> 11;
    var2 = (((((adc_t >> 4) - ((int32_t)dev->dig_T1)) * ((adc_t >> 4) - ((int32_t)dev->dig_T1))) >> 12) * 
           ((int32_t)dev->dig_T3)) >> 14;
    dev->t_fine = var1 + var2;
    dev->temperature = (dev->t_fine * 5 + 128) >> 8;
    dev->temperature /= 100.0;  // 转换为°C
    
    /* 修复压力计算*/
	double var1_p,var2_p,p;
    
    var1_p = ((double)dev->t_fine/2.0) - 64000.0;
	var2_p = var1_p * var1_p * ((double)dev->dig_P6) / 32768.0;
	var2_p = var2_p + var1_p * ((double)dev->dig_P5) * 2.0;
	var2_p = (var2_p/4.0)+(((double)dev->dig_P4) * 65536.0);
	var1_p = (((double)dev->dig_P3) * var1_p * var1_p /524288.0 + ((double)dev->dig_P2)* var1_p) / 524288.0;
	var1_p = (1.0 + var1_p / 32768.0) * ((double)dev->dig_P1);
	
	if (var1_p == 0) {
        return HAL_ERROR;  // 避免除零错误
    }
    
    p = (1048576.0 - (double)adc_p);
	p = (p -(var2_p/4096.0)) * 6250.0 / var1_p;
	var1_p = ((double)dev->dig_P9) * p * p /2147483648.0;
	var2_p = p * ((double)dev->dig_P8) / 32768.0;
	p = p +(var1_p + var2_p + ((double)dev->dig_P7)) /16.0;
    
    dev->pressure = (float)p / 100.0;  // 转换为hPa
    
    /* 计算海拔（保持不变） */
    dev->altitude = 44330.0 * (1.0 - pow(dev->pressure / 1013.25, 0.1902949f));
	
	dev->pressure/=10.0;// 转换为kPa
    
    return HAL_OK;
}

/* 计算海拔 */
float BMP280_CalculateAltitude(float pressure, float sea_level_pressure) {
    return 44330.0 * (1.0 - pow(pressure / sea_level_pressure, 0.1902949f));
}