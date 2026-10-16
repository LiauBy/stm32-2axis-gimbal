#include "stm32f10x.h"
#include "MPU6050.h"
#include "IMU.h"
#include <math.h>

/* ============================================================
 * 三轴姿态解算：Mahony 四元数互补滤波（无磁力计版本）
 * ------------------------------------------------------------
 * 原理：
 *   1) 陀螺仪积分提供高频姿态（短时准确、长时间漂移）
 *   2) 加速度计提供重力方向参考（无漂移、但有震动噪声）
 *   3) 用加速度计估计的重力方向与四元数推算的重力方向做
 *      叉乘得到误差，经 PI 环节反馈修正陀螺仪角速度，
 *      从而抑制 roll / pitch 的积分漂移
 *   4) yaw 没有绝对参考（无磁力计），由 Z 轴陀螺仪积分得到，
 *      长时间会有缓慢漂移，这是无磁力计方案的固有特性
 * ============================================================ */

/* 三轴姿态角（度）——ISR 写、主循环读，必须 volatile 防止 -O1 读到缓存旧值 */
volatile float IMU_Roll  = 0.0f;
volatile float IMU_Pitch = 0.0f;
volatile float IMU_Yaw   = 0.0f;

/* 三轴角速度（度/秒）——陀螺仪去零偏后直接得到，供控制环做扭矩级阻尼（Level 1） */
volatile float IMU_RollRate  = 0.0f;
volatile float IMU_PitchRate = 0.0f;
volatile float IMU_YawRate   = 0.0f;

/* 姿态四元数 */
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

/* 陀螺仪零偏（LSB），上电静止校准 */
static float gyroOffsetX = 0.0f;
static float gyroOffsetY = 0.0f;
static float gyroOffsetZ = 0.0f;

/* Mahony 积分误差项 */
static float integralFBx = 0.0f;
static float integralFBy = 0.0f;
static float integralFBz = 0.0f;

/* 陀螺仪量程 ±2000dps：原始值(LSB) → 弧度/秒 */
#define GYRO_RAW_TO_RAD  (2000.0f / 32768.0f * 3.14159265f / 180.0f)

/* Mahony 增益：2*Kp 收敛速度 / 2*Ki 零偏积分补偿
 * 自稳云台震动较大，Ki 建议设小或 0（零偏已静态校准）*/
#define TWO_KP  (2.0f * 1.5f)   /* Kp = 1.5 */
#define TWO_KI  (2.0f * 0.0f)   /* Ki = 0.0 */

/* 快速平方根倒数（Mahony 原文的优化实现） */
static float invSqrt(float x)
{
	union { float f; long i; } u;
	float halfx = 0.5f * x;
	u.f = x;
	u.i = 0x5F3759DF - (u.i >> 1);
	u.f = u.f * (1.5f - halfx * u.f * u.f);
	return u.f;
}

/* 初始化：配置 MPU6050，并在静止状态下校准陀螺仪零偏 */
void IMU_Init(void)
{
	uint32_t i, ok = 0;
	int32_t sumX = 0, sumY = 0, sumZ = 0;
	int16_t ax, ay, az, gx, gy, gz;

	MPU6050_Init();

	/* 静止采样 500 次，平均作为陀螺仪零偏（读取失败跳过，避免把坏数据算进零偏） */
	for (i = 0; i < 500; i ++)
	{
		if (MPU6050_GetData(&ax, &ay, &az, &gx, &gy, &gz) != 0)
			continue;
		sumX += gx;
		sumY += gy;
		sumZ += gz;
		ok ++;
	}
	if (ok > 0)
	{
		gyroOffsetX = (float)sumX / (float)ok;
		gyroOffsetY = (float)sumY / (float)ok;
		gyroOffsetZ = (float)sumZ / (float)ok;
	}

	/* 复位姿态 */
	q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
	integralFBx = integralFBy = integralFBz = 0.0f;
	IMU_Roll = IMU_Pitch = IMU_Yaw = 0.0f;
}

/* Mahony 互补滤波单步更新（输入原始 LSB，dt 单位秒） */
void IMU_Update(int16_t ax, int16_t ay, int16_t az,
                int16_t gx, int16_t gy, int16_t gz, float dt)
{
	float axf, ayf, azf;
	float gxr, gyr, gzr;
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	/* 陀螺仪：去零偏 → 弧度/秒 */
	gxr = ((float)gx - gyroOffsetX) * GYRO_RAW_TO_RAD;
	gyr = ((float)gy - gyroOffsetY) * GYRO_RAW_TO_RAD;
	gzr = ((float)gz - gyroOffsetZ) * GYRO_RAW_TO_RAD;

	/* 加速度计：归一化，只用其方向 */
	axf = (float)ax; ayf = (float)ay; azf = (float)az;
	recipNorm = invSqrt(axf * axf + ayf * ayf + azf * azf);
	axf *= recipNorm;
	ayf *= recipNorm;
	azf *= recipNorm;

	/* 由当前四元数推算的重力方向（估计值） */
	halfvx = q1 * q3 - q0 * q2;
	halfvy = q0 * q1 + q2 * q3;
	halfvz = q0 * q0 - 0.5f + q3 * q3;

	/* 误差 = 测量值 × 估计值（叉乘，等价于两方向的夹角正弦） */
	halfex = (ayf * halfvz - azf * halfvy);
	halfey = (azf * halfvx - axf * halfvz);
	halfez = (axf * halfvy - ayf * halfvx);

	/* 积分项（补偿陀螺仪零偏漂移） */
	if (TWO_KI > 0.0f)
	{
		integralFBx += TWO_KI * halfex * dt;
		integralFBy += TWO_KI * halfey * dt;
		integralFBz += TWO_KI * halfez * dt;
		gxr += integralFBx;
		gyr += integralFBy;
		gzr += integralFBz;
	}
	else
	{
		integralFBx = integralFBy = integralFBz = 0.0f;
	}

	/* 比例项（误差直接修正角速度） */
	gxr += TWO_KP * halfex;
	gyr += TWO_KP * halfey;
	gzr += TWO_KP * halfez;

	/* ★ 导出角速度（度/秒）：控制环用它做扭矩级阻尼，直接抵抗相机转动 */
	IMU_RollRate  = gxr * 57.29578f;
	IMU_PitchRate = gyr * 57.29578f;
	IMU_YawRate   = gzr * 57.29578f;

	/* 四元数一阶积分（dt 很小，一阶足够） */
	qa = q0; qb = q1; qc = q2;
	q0 += (-qb * gxr - qc * gyr - q3 * gzr) * dt * 0.5f;
	q1 += ( qa * gxr + qc * gzr - q3 * gyr) * dt * 0.5f;
	q2 += ( qa * gyr - qb * gzr + q3 * gxr) * dt * 0.5f;
	q3 += ( qa * gzr + qb * gyr - qc * gxr) * dt * 0.5f;

	/* 四元数归一化，防止漂移 */
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

	/* 四元数 → 欧拉角（度）
	 * Roll  : 绕X轴  左右倾
	 * Pitch : 绕Y轴  前后倾
	 * Yaw   : 绕Z轴  水平转向 */
	IMU_Roll  = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * 57.29578f;
	IMU_Pitch = asinf(2.0f * (q0 * q2 - q3 * q1)) * 57.29578f;
	IMU_Yaw   = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * 57.29578f;
}
