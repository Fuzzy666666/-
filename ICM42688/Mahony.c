#include <math.h>
#include "Mahony.h"


// =============== 校准零偏及定时器中断获取角度数据 ===============

#include "ICM42688.h"
#define IMU_UPDATE_HZ     	1000U
#define IMU_DT            	(1.0f / 1000.0f)
#define DEG_TO_RAD        	0.0174532925f
#define GYRO_CALIB_SAMPLES 	2000U
/* 标定时静止方差判据：单位 (rad/s)^2，约等于 (0.5°/s)^2 */
#define GYRO_CALIB_VAR_MAX  (0.5f * DEG_TO_RAD) * (0.5f * DEG_TO_RAD)
/* 标定最多重试次数，超过后接受当前结果（避免被动环境永远卡住） */
#define GYRO_CALIB_MAX_RETRY 3U

int test = 0;
ICM42688_Data_t imu_data;
IMU_Data IMU;

/* 单次标定采集：返回均值与方差，返回 1 表示静止方差合格 */
static uint8_t gyro_calib_once(float *mx, float *my, float *mz, float *vx, float *vy, float *vz, float *mean_temp)
{
    uint32_t i;
    float sum_x = 0, sum_y = 0, sum_z = 0;
    float sum_x2 = 0, sum_y2 = 0, sum_z2 = 0;
    float sum_t = 0;

    for(i = 0; i < GYRO_CALIB_SAMPLES; i++)
    {
        ICM42688_Read_6Axis_Temp( &imu_data );
        float gx = imu_data.gyro_x * DEG_TO_RAD;
        float gy = imu_data.gyro_y * DEG_TO_RAD;
        float gz = imu_data.gyro_z * DEG_TO_RAD;
        sum_x += gx;  sum_y += gy;  sum_z += gz;
        sum_x2 += gx*gx;  sum_y2 += gy*gy;  sum_z2 += gz*gz;
        sum_t += imu_data.temp;
        HAL_Delay(1);
    }

    float n = (float)GYRO_CALIB_SAMPLES;
    *mx = sum_x / n;
    *my = sum_y / n;
    *mz = sum_z / n;
    *vx = sum_x2 / n - (*mx) * (*mx);
    *vy = sum_y2 / n - (*my) * (*my);
    *vz = sum_z2 / n - (*mz) * (*mz);
    *mean_temp = sum_t / n;

    if (*vx < GYRO_CALIB_VAR_MAX && *vy < GYRO_CALIB_VAR_MAX && *vz < GYRO_CALIB_VAR_MAX)
        return 1;
    return 0;
}

void Gyro_Bias_Calibrate( void )
{
    float mx = 0, my = 0, mz = 0;
    float vx = 0, vy = 0, vz = 0;
    float mean_temp = 25.0f;
    uint32_t retry;

    for (retry = 0; retry < GYRO_CALIB_MAX_RETRY; retry++)
    {
        if (gyro_calib_once(&mx, &my, &mz, &vx, &vy, &vz, &mean_temp))
            break;
        /* 采样不够静，稍等重试 */
        HAL_Delay(100);
    }

    Mahony_Set_Gyro_Bias(&IMU, mx, my, mz);
    /* 把标定温度和当时的零偏记下来，作为温度补偿的参考点 */
    Mahony_Set_Temp_Ref(&IMU, mean_temp, mx, my, mz);
}

void USE_TIM_Get_Data( void )
{
	ICM42688_Read_6Axis_Temp(&imu_data);
    Mahony_Update_IMU_Dt(&IMU,
            imu_data.gyro_x * DEG_TO_RAD,
            imu_data.gyro_y * DEG_TO_RAD,
            imu_data.gyro_z * DEG_TO_RAD,
            imu_data.accel_x,
            imu_data.accel_y,
            imu_data.accel_z,
            imu_data.temp,
            IMU_DT);
}
//void HAL_TIM_PeriodElapsedCallback( TIM_HandleTypeDef *htim )
//{
//  if(htim->Instance == TIM2)
//	{
//      USE_TIM_Get_Data();
//  }
//}



// =============== Mahony滤波解算 ===============

/* 兼容接口使用的默认采样时序 */
#define DEFAULT_SAMPLE_FREQ 1000.0f
#define DEFAULT_DT          (1.0f / DEFAULT_SAMPLE_FREQ)
/* Mahony PI参数 */
#define Kp_Normal           2.0f
#define Ki                  0.005f
/* 以1g为中心的加速度置信窗口，用于动态降低加计修正权重 */
#define ACC_TRUST_RANGE_G   0.20f
/* 积分限幅，防止积分饱和 */
#define EINT_LIM            0.20f
#define RAD_TO_DEG          57.2957795f

/* 静止检测参数 */
#define STILL_ACC_TOL       0.05f                    /* |‖a‖-1g| 小于此值视为准静止 */
#define STILL_GYRO_TOL      (0.8f * DEG_TO_RAD)      /* 去偏后陀螺模长阈值，rad/s */
#define STILL_HOLD_SAMPLES  500U                     /* 连续 500 次（约 500ms@1kHz）判定进入静止 */
/* 静止时用于零偏在线跟踪的一阶 LPF 系数（每采样点），越小越慢越稳 */
#define BIAS_LPF_ALPHA      0.001f
/* 为避免高频抖动污染，把在线更新幅度限制住（rad/s 每次最多变化） */
#define BIAS_LPF_MAX_STEP   (0.02f * DEG_TO_RAD)

/* 轻量级浮点限幅函数（适合嵌入式） */
static float clampf(float x, float min_v, float max_v)
{
    if (x < min_v)
    {
        return min_v;
    }
    if (x > max_v)
    {
        return max_v;
    }
    return x;
}

void IMU_Data_Init( IMU_Data *IMU )
{
    /* 单位四元数：表示无旋转 */
    IMU->q[0] = 1.0f;
    IMU->q[1] = 0.0f;
    IMU->q[2] = 0.0f;
    IMU->q[3] = 0.0f;

    /* 清空积分修正项 */
    IMU->eInt[0] = 0.0f;
    IMU->eInt[1] = 0.0f;
    IMU->eInt[2] = 0.0f;

    /* 零偏由上电标定流程写入 */
    IMU->gyro_bias[0] = 0.0f;
    IMU->gyro_bias[1] = 0.0f;
    IMU->gyro_bias[2] = 0.0f;

    /* 温度补偿参考点与系数 */
    IMU->calib_temp    = 25.0f;
    IMU->calib_bias[0] = 0.0f;
    IMU->calib_bias[1] = 0.0f;
    IMU->calib_bias[2] = 0.0f;
    IMU->temp_coef[0]  = 0.0f;
    IMU->temp_coef[1]  = 0.0f;
    IMU->temp_coef[2]  = 0.0f;

    /* 静止检测状态 */
    IMU->still_counter = 0;
    IMU->is_still      = 0;

    /* 对外姿态角输出 */
    IMU->Roll = 0.0f;
    IMU->Pitch = 0.0f;
    IMU->Yaw = 0.0f;

    IMU->init_flag = 1;
}

void Mahony_Set_Gyro_Bias( IMU_Data *IMU, float bx, float by, float bz )
{
    if( IMU->init_flag == 0 )
    {
        IMU_Data_Init( IMU );
    }

    IMU->gyro_bias[0] = bx;
    IMU->gyro_bias[1] = by;
    IMU->gyro_bias[2] = bz;
}

void Mahony_Set_Temp_Ref( IMU_Data *IMU, float temp_c, float bx, float by, float bz )
{
    if( IMU->init_flag == 0 )
    {
        IMU_Data_Init( IMU );
    }

    IMU->calib_temp    = temp_c;
    IMU->calib_bias[0] = bx;
    IMU->calib_bias[1] = by;
    IMU->calib_bias[2] = bz;
}

void Mahony_Set_Temp_Coef( IMU_Data *IMU, float cx, float cy, float cz )
{
    if( IMU->init_flag == 0 )
    {
        IMU_Data_Init( IMU );
    }

    IMU->temp_coef[0] = cx;
    IMU->temp_coef[1] = cy;
    IMU->temp_coef[2] = cz;
}

void Mahony_Update_IMU_Dt( IMU_Data *IMU, float gx, float gy, float gz, float ax, float ay, float az, float temp_c, float dt )
{
    float vx;
    float vy;
    float vz;
    float ex = 0.0f;
    float ey = 0.0f;
    float ez = 0.0f;
    float acc_weight = 0.0f;

    if( IMU->init_flag == 0 )
    {
        IMU_Data_Init( IMU );
    }

    if( dt <= 0.0f )
    {
        /* 调用方传入无效dt时回退到默认值 */
        dt = DEFAULT_DT;
    }

    /* 温度补偿：以标定温度为基准，用离线标定的斜率外推当前零偏参考点
       若未设置 temp_coef（全为0），则等价于只用 gyro_bias，不影响原行为 */
    float dT = temp_c - IMU->calib_temp;
    float bias_x = IMU->calib_bias[0] + IMU->temp_coef[0] * dT;
    float bias_y = IMU->calib_bias[1] + IMU->temp_coef[1] * dT;
    float bias_z = IMU->calib_bias[2] + IMU->temp_coef[2] * dT;

    /* ZUPT 在线跟踪得到的慢漂修正（以当前 gyro_bias 与温补基准的差作为缓慢积累值） */
    float trim_x = IMU->gyro_bias[0] - IMU->calib_bias[0];
    float trim_y = IMU->gyro_bias[1] - IMU->calib_bias[1];
    float trim_z = IMU->gyro_bias[2] - IMU->calib_bias[2];

    /* 最终使用的总零偏 = 温补外推 + ZUPT 慢漂修正 */
    float use_bx = bias_x + trim_x;
    float use_by = bias_y + trim_y;
    float use_bz = bias_z + trim_z;

    /* 积分前先减去总零偏 */
    float gx_c = gx - use_bx;
    float gy_c = gy - use_by;
    float gz_c = gz - use_bz;

    float q0 = IMU->q[0];
    float q1 = IMU->q[1];
    float q2 = IMU->q[2];
    float q3 = IMU->q[3];

    float q0q0 = q0*q0;
    float q0q1 = q0*q1;
    float q0q2 = q0*q2;
    float q0q3 = q0*q3;
    float q1q1 = q1*q1;
    float q1q2 = q1*q2;
    float q1q3 = q1*q3;
    float q2q2 = q2*q2;
    float q2q3 = q2*q3;
    float q3q3 = q3*q3;

    float acc_norm = 0.0f;
    {
        float acc_sq = ax*ax + ay*ay + az*az;
        if( acc_sq > 1.0e-8f )
        {
            float inv_acc_norm;
            acc_norm = sqrtf( acc_sq );
            float acc_dev = fabsf( acc_norm - 1.0f );

            /* 加速度模长越接近1g，说明越可信 */
            if( acc_dev < ACC_TRUST_RANGE_G )
            {
                acc_weight = 1.0f - (acc_dev / ACC_TRUST_RANGE_G);
            }
            else
            {
                acc_weight = 0.0f;
            }

            inv_acc_norm = 1.0f / acc_norm;
            ax *= inv_acc_norm;
            ay *= inv_acc_norm;
            az *= inv_acc_norm;

            /* 由当前四元数估计重力方向 */
            vx = 2.0f * ( q1q3 - q0q2 );
            vy = 2.0f * ( q0q1 + q2q3 );
            vz = q0q0 - q1q1 - q2q2 + q3q3;

            /* 实测重力与估计重力之间的误差 */
            ex = ( ay * vz - az * vy );
            ey = ( az * vx - ax * vz );
            ez = ( ax * vy - ay * vx );
        }
    }

    /* --------- 静止检测（加速度接近 1g + 去偏后角速率很小） --------- */
    {
        float g_mag_sq = gx_c*gx_c + gy_c*gy_c + gz_c*gz_c;
        uint8_t acc_ok  = (fabsf(acc_norm - 1.0f) < STILL_ACC_TOL);
        uint8_t gyro_ok = (g_mag_sq < STILL_GYRO_TOL * STILL_GYRO_TOL);
        if (acc_ok && gyro_ok)
        {
            if (IMU->still_counter < 0xFFFFFFFFU) IMU->still_counter++;
            if (IMU->still_counter >= STILL_HOLD_SAMPLES) IMU->is_still = 1;
        }
        else
        {
            IMU->still_counter = 0;
            IMU->is_still      = 0;
        }
    }

    /* --------- ZUPT：静止时用极慢 LPF 把原始陀螺读数往 gyro_bias 拉 --------- */
    if (IMU->is_still)
    {
        float step_x = BIAS_LPF_ALPHA * (gx - IMU->gyro_bias[0]);
        float step_y = BIAS_LPF_ALPHA * (gy - IMU->gyro_bias[1]);
        float step_z = BIAS_LPF_ALPHA * (gz - IMU->gyro_bias[2]);
        step_x = clampf(step_x, -BIAS_LPF_MAX_STEP, BIAS_LPF_MAX_STEP);
        step_y = clampf(step_y, -BIAS_LPF_MAX_STEP, BIAS_LPF_MAX_STEP);
        step_z = clampf(step_z, -BIAS_LPF_MAX_STEP, BIAS_LPF_MAX_STEP);
        IMU->gyro_bias[0] += step_x;
        IMU->gyro_bias[1] += step_y;
        IMU->gyro_bias[2] += step_z;
    }

    /* 积分项更新并做抗饱和限幅 */
    if( Ki > 0.0f && acc_weight > 0.0f )
    {
        IMU->eInt[0] += ex * Ki * acc_weight * dt;
        IMU->eInt[1] += ey * Ki * acc_weight * dt;
        IMU->eInt[2] += ez * Ki * acc_weight * dt;

        IMU->eInt[0] = clampf(IMU->eInt[0], -EINT_LIM, EINT_LIM);
        IMU->eInt[1] = clampf(IMU->eInt[1], -EINT_LIM, EINT_LIM);
        IMU->eInt[2] = clampf(IMU->eInt[2], -EINT_LIM, EINT_LIM);
    }

    gx_c = gx_c + (Kp_Normal * acc_weight) * ex + IMU->eInt[0];
    gy_c = gy_c + (Kp_Normal * acc_weight) * ey + IMU->eInt[1];
    gz_c = gz_c + (Kp_Normal * acc_weight) * ez + IMU->eInt[2];

    {
        /* 四元数微分方程离散积分 */
        float halfT = 0.5f * dt;
        IMU->q[0] += (-q1 * gx_c - q2 * gy_c - q3 * gz_c) * halfT;
        IMU->q[1] += ( q0 * gx_c + q2 * gz_c - q3 * gy_c) * halfT;
        IMU->q[2] += ( q0 * gy_c - q1 * gz_c + q3 * gx_c) * halfT;
        IMU->q[3] += ( q0 * gz_c + q1 * gy_c - q2 * gx_c) * halfT;
    }

    {
        /* 四元数归一化，抑制数值漂移 */
        float q_norm_sq = IMU->q[0]*IMU->q[0] + IMU->q[1]*IMU->q[1] + IMU->q[2]*IMU->q[2] + IMU->q[3]*IMU->q[3];
        if( q_norm_sq > 1.0e-12f )
        {
            float inv_q_norm = 1.0f / sqrtf( q_norm_sq );
            IMU->q[0] *= inv_q_norm;
            IMU->q[1] *= inv_q_norm;
            IMU->q[2] *= inv_q_norm;
            IMU->q[3] *= inv_q_norm;
        }
        else
        {
            /* 罕见数值异常兜底：重置姿态但保留已标定零偏/温补参数 */
            float bx = IMU->gyro_bias[0];
            float by = IMU->gyro_bias[1];
            float bz = IMU->gyro_bias[2];
            float ct = IMU->calib_temp;
            float cbx = IMU->calib_bias[0];
            float cby = IMU->calib_bias[1];
            float cbz = IMU->calib_bias[2];
            float tcx = IMU->temp_coef[0];
            float tcy = IMU->temp_coef[1];
            float tcz = IMU->temp_coef[2];
            IMU_Data_Init( IMU );
            IMU->gyro_bias[0] = bx;
            IMU->gyro_bias[1] = by;
            IMU->gyro_bias[2] = bz;
            IMU->calib_temp    = ct;
            IMU->calib_bias[0] = cbx;
            IMU->calib_bias[1] = cby;
            IMU->calib_bias[2] = cbz;
            IMU->temp_coef[0]  = tcx;
            IMU->temp_coef[1]  = tcy;
            IMU->temp_coef[2]  = tcz;
        }
    }

    {
        /* 四元数转欧拉角（度），便于监控与输出 */
        float pitch_temp = 2.0f * (IMU->q[0]*IMU->q[2] - IMU->q[1]*IMU->q[3]);
        pitch_temp = clampf(pitch_temp, -1.0f, 1.0f);

        IMU->Pitch = asinf( pitch_temp ) * RAD_TO_DEG;
        IMU->Roll  = atan2f(2.0f * (IMU->q[0]*IMU->q[1] + IMU->q[2]*IMU->q[3]), 1.0f - 2.0f * (IMU->q[1]*IMU->q[1] + IMU->q[2]*IMU->q[2])) * RAD_TO_DEG;
        IMU->Yaw   = atan2f(2.0f * (IMU->q[0]*IMU->q[3] + IMU->q[1]*IMU->q[2]), 1.0f - 2.0f * (IMU->q[2]*IMU->q[2] + IMU->q[3]*IMU->q[3])) * RAD_TO_DEG;
    }
}
