#ifndef __IMU_H
#define __IMU_H

#include <stdint.h>

/* 三轴姿态角（单位：度），全局可读。ISR 写、主循环读，须为 volatile */
extern volatile float IMU_Roll;   // 横滚角 Roll
extern volatile float IMU_Pitch;  // 俯仰角 Pitch
extern volatile float IMU_Yaw;    // 偏航角 Yaw

/* 三轴角速度（单位：度/秒），控制环扭矩级阻尼用（Level 1） */
extern volatile float IMU_RollRate;   // 横滚角速度
extern volatile float IMU_PitchRate;  // 俯仰角速度
extern volatile float IMU_YawRate;    // 偏航角速度

/* 初始化 MPU6050 并自动校准陀螺仪零偏（上电时保持静止约几十ms） */
void IMU_Init(void);

/* Mahony 互补滤波姿态解算，每 dt 秒调用一次
 * 输入为 MPU6050_GetData 读出的原始值（LSB）
 * dt 为两次调用的时间间隔，单位秒（本工程为 0.01f）*/
void IMU_Update(int16_t ax, int16_t ay, int16_t az,
                int16_t gx, int16_t gy, int16_t gz, float dt);

#endif
