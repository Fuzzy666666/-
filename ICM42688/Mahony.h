#ifndef __MAHONY_H__
#define __MAHONY_H__

#include "main.h"
#include <stdint.h>

/* IMU姿态融合状态结构体 */
typedef struct
{
    /* 欧拉角输出（单位：度） */
    float Roll;
    float Pitch;
    float Yaw;

    /* 四元数状态：q0, q1, q2, q3 */
    float q[4];
    /* Mahony PI校正中的积分项 */
    float eInt[3];
    /* 陀螺仪静态零偏（单位：rad/s），由标定写入，运行时在线微调 */
    float gyro_bias[3];

    /* 温度补偿参考点：标定时温度 + 标定时零偏（rad/s） */
    float calib_temp;
    float calib_bias[3];
    /* 温度漂移系数（rad/s/°C），每轴一个，离线标定后写入 */
    float temp_coef[3];

    /* 静止检测状态 */
    uint32_t still_counter;     /* 连续判定静止的采样数 */
    uint8_t  is_still;          /* 当前是否处于静止状态 */

    /* 初始化标志：0未初始化，1已初始化 */
    uint8_t init_flag;
} IMU_Data;

/* 复位融合状态到单位姿态，并清零积分项/零偏 */
void IMU_Data_Init( IMU_Data *IMU );
/* 设置陀螺仪标定零偏（rad/s） */
void Mahony_Set_Gyro_Bias( IMU_Data *IMU, float bx, float by, float bz );
/* 设置温度补偿参考点：标定时温度（°C）和当时的零偏（rad/s） */
void Mahony_Set_Temp_Ref( IMU_Data *IMU, float temp_c, float bx, float by, float bz );
/* 设置每轴温度漂移系数（rad/s/°C），可离线标定后填入 */
void Mahony_Set_Temp_Coef( IMU_Data *IMU, float cx, float cy, float cz );
/* Mahony更新（显式传入采样周期dt，单位秒，温度单位°C） */
void Mahony_Update_IMU_Dt( IMU_Data *IMU, float gx, float gy, float gz, float ax, float ay, float az, float temp_c, float dt );

// =============== 校准零偏及定时器中断获取角度数据 ===============
extern int test;
extern ICM42688_Data_t imu_data;
extern IMU_Data IMU;
void Gyro_Bias_Calibrate( void );		// 校准零偏
void USE_TIM_Get_Data( void );			// 使用定时器中断获取数据

#endif
