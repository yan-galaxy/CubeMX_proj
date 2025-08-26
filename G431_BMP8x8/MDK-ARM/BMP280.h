/* BMP280.h - 头文件定义 */
#ifndef BMP280_H
#define BMP280_H

#include "stm32g4xx_hal.h"

/* BMP280寄存器地址定义 */
#define BMP280_DIG_T1_LSB                  0x88
#define BMP280_DIG_T1_MSB                  0x89
#define BMP280_DIG_T2_LSB                  0x8A
#define BMP280_DIG_T2_MSB                  0x8B
#define BMP280_DIG_T3_LSB                  0x8C
#define BMP280_DIG_T3_MSB                  0x8D
#define BMP280_DIG_P1_LSB                  0x8E
#define BMP280_DIG_P1_MSB                  0x8F
#define BMP280_DIG_P2_LSB                  0x90
#define BMP280_DIG_P2_MSB                  0x91
#define BMP280_DIG_P3_LSB                  0x92
#define BMP280_DIG_P3_MSB                  0x93
#define BMP280_DIG_P4_LSB                  0x94
#define BMP280_DIG_P4_MSB                  0x95
#define BMP280_DIG_P5_LSB                  0x96
#define BMP280_DIG_P5_MSB                  0x97
#define BMP280_DIG_P6_LSB                  0x98
#define BMP280_DIG_P6_MSB                  0x99
#define BMP280_DIG_P7_LSB                  0x9A
#define BMP280_DIG_P7_MSB                  0x9B
#define BMP280_DIG_P8_LSB                  0x9C
#define BMP280_DIG_P8_MSB                  0x9D
#define BMP280_DIG_P9_LSB                  0x9E
#define BMP280_DIG_P9_MSB                  0x9F

#define BMP280_CHIP_ID                     0xD0
#define BMP280_RESET                       0xE0
#define BMP280_STATUS                      0xF3
#define BMP280_CTRL_MEAS                   0xF4
#define BMP280_CONFIG                      0xF5
#define BMP280_PRESSURE_MSB                0xF7
#define BMP280_PRESSURE_LSB                0xF8
#define BMP280_PRESSURE_XLSB               0xF9
#define BMP280_TEMPERATURE_MSB             0xFA
#define BMP280_TEMPERATURE_LSB             0xFB
#define BMP280_TEMPERATURE_XLSB            0xFC

/* 模式定义 */
#define BMP280_MODE_SLEEP                  0x00
#define BMP280_MODE_FORCED                 0x01
#define BMP280_MODE_NORMAL                 0x03

/* 过采样设置 */
#define BMP280_OSRS_T_SKIPPED              0x00
#define BMP280_OSRS_T_1X                   0x01
#define BMP280_OSRS_T_2X                   0x02
#define BMP280_OSRS_T_4X                   0x03
#define BMP280_OSRS_T_8X                   0x04
#define BMP280_OSRS_T_16X                  0x05

#define BMP280_OSRS_P_SKIPPED              0x00
#define BMP280_OSRS_P_1X                   0x01
#define BMP280_OSRS_P_2X                   0x02
#define BMP280_OSRS_P_4X                   0x03
#define BMP280_OSRS_P_8X                   0x04
#define BMP280_OSRS_P_16X                  0x05

/* 滤波器设置 */
#define BMP280_FILTER_OFF                  0x00
#define BMP280_FILTER_2                    0x01
#define BMP280_FILTER_4                    0x02
#define BMP280_FILTER_8                    0x03
#define BMP280_FILTER_16                   0x04

/* 待机时间设置 */
#define BMP280_T_STANDBY_0_5MS             0x00
#define BMP280_T_STANDBY_62_5MS            0x01
#define BMP280_T_STANDBY_125MS             0x02
#define BMP280_T_STANDBY_250MS             0x03
#define BMP280_T_STANDBY_500MS             0x04
#define BMP280_T_STANDBY_1000MS            0x05
#define BMP280_T_STANDBY_2000MS            0x06
#define BMP280_T_STANDBY_4000MS            0x07

/* 设备类型定义 */
typedef struct {
    SPI_HandleTypeDef *hspi;
    uint8_t cs_index;
    uint8_t chip_id;
    
    /* 校准参数 */
    uint16_t dig_T1;
    int16_t dig_T2, dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
    
    int32_t t_fine;
    float temperature;
    float pressure;
    float altitude;
} BMP280_Device;

/* 函数声明 */
HAL_StatusTypeDef BMP280_Init(BMP280_Device *dev);
HAL_StatusTypeDef BMP280_ReadChipID(BMP280_Device *dev);
HAL_StatusTypeDef BMP280_SoftReset(BMP280_Device *dev);
HAL_StatusTypeDef BMP280_SetMode(BMP280_Device *dev, uint8_t osrs_t, uint8_t osrs_p, uint8_t mode);
HAL_StatusTypeDef BMP280_SetConfig(BMP280_Device *dev, uint8_t filter, uint8_t t_standby);
HAL_StatusTypeDef BMP280_ReadCalibrationParams(BMP280_Device *dev);
HAL_StatusTypeDef BMP280_ReadPressureTemperature(BMP280_Device *dev,uint8_t dev_index);
float BMP280_CalculateAltitude(float pressure, float sea_level_pressure);

#endif /* BMP280_H */

