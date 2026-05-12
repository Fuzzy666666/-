#ifndef __ICM42688_H__
#define __ICM42688_H__

#include "main.h"

/* ---------------- Bank 0 寄存器地址 ---------------- */
#define ICM42688_WHO_AM_I      0x75  /* 固定器件ID寄存器，期望值0x47 */
#define ICM42688_REG_BANK_SEL  0x76  /* 寄存器Bank选择 */
#define ICM42688_DEVICE_CONFIG 0x11  /* 设备配置（含软复位） */
#define ICM42688_PWR_MGMT0     0x4E  /* 电源管理（加计/陀螺工作模式） */
#define ICM42688_GYRO_CONFIG0  0x4F  /* 陀螺量程与ODR配置 */
#define ICM42688_ACCEL_CONFIG0 0x50  /* 加计量程与ODR配置 */
#define ICM42688_TEMP_DATA1    0x1D  /* 温度高字节起始地址 */
#define ICM42688_ACCEL_DATA_X1 0x1F  /* 加速度X高字节起始地址（连续6字节） */
#define ICM42688_GYRO_DATA_X1  0x25  /* 陀螺X高字节起始地址（连续6字节） */

/* ---------------- 数字滤波相关寄存器 ---------------- */
#define ICM42688_GYRO_CONFIG1  0x51  /* 陀螺滤波配置 */
#define ICM42688_ACCEL_CONFIG1 0x53  /* 加计滤波配置 */

/* ---------------- 常用配置值 ---------------- */
#define PWR_MGMT0_ACCEL_LN_GYRO_LN 0x0F  /* 加计+陀螺均为低噪声模式 */
#define GYRO_ROBOT_2000_1KHZ       0x06  /* 陀螺: ±2000dps, ODR=1kHz */
#define ACCEL_ROBOT_16G_1KHZ       0x06  /* 加计: ±16g, ODR=1kHz */
#define FILTER_BW_100HZ_3RD        0x43  /* 三阶滤波，截止约100Hz */

/* 六轴与温度数据结构 */
typedef struct
{
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float temp;
} ICM42688_Data_t;

/* 对外接口 */
void TIM3_Init_Freq( uint32_t freq_hz );
uint8_t ICM42688_Init( void );
void ICM42688_Read_6Axis_Temp( ICM42688_Data_t *data );

/* 驱动内部接口 */
void ICM42688_SPI_Init( void );
uint8_t SPI_ReadWriteByte( uint8_t TxData );
uint8_t ICM42688_ReadReg( uint8_t regAddr );
void ICM42688_ReadRegs( uint8_t regAddr, uint8_t *pData, uint16_t len );
void ICM42688_WriteReg( uint8_t regAddr, uint8_t writeData );

#endif



