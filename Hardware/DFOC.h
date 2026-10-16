#ifndef __DFOC_H
#define __DFOC_H

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "pwm.h"
#include "AS5600.h"
#include "Delay.h"
#include "Serial.h"
#include "Lowpass_Filter.h"
#include "PID_Control.h"
#include "Inlinecurrent.h"

/* 电机 FOC 状态（M0=横滚，M1=俯仰） */
struct Motor_{
	int Mot_num;
	float Ua;
	float Ub;
	float Uc;
	float Ubeta;
	float Ualpha;
	float dc_a;
	float dc_b;
	float dc_c;
	volatile float Enc_Angle;   /* 最近一次编码器绝对角(rad)，ISR 写、主循环读，须为 volatile */
};

extern struct Motor_ M0;
extern struct Motor_ M1;
extern int M0_PP, M0_DIR;   /* 极对数 / 编码器方向（±1），在 main.c 定义 */
extern int M1_PP, M1_DIR;

/* ★ 传感器故障锁存（ISR 检测到 I2C 持续失败后置位） */
extern volatile uint8_t FOC_Fault_Flag;    /* 1 = 已停机（锁存，需重新上电） */
extern volatile uint8_t FOC_Fault_Notify;  /* 1 = 待主循环串口上报一次 */

/* ★ Level 1 相机角速度（deg/s）：ISR 写入、M0/M1_Set_Angle 读取做扭矩级阻尼 */
extern volatile float g_IMU_RollRate;
extern volatile float g_IMU_PitchRate;

void Motor_en(void);
float constrain(float amt, float low, float high);
void SetPwm(int Mot_num, float Ua, float Ub, float Uc);
float normalizeAngle(float angle);
void SetPhaseVoltage(struct Motor_ *Motor, float Uq, float angle_el);
void Check_Sensor(void);
void FOC_Init(float power);
float M0_electricAngle(void);
float M1_electricAngle(void);
void tripod_Iint(void);

/* ★ 软启动 / 故障停机 */
void FOC_SoftStart_Begin(void);    /* 进入自稳前清零斜坡 */
void FOC_SoftStart_Tick(void);     /* 每个控制周期推进斜坡 */
float FOC_GetStartRamp(void);      /* 诊断：读取当前斜坡系数(0~1) */
void FOC_Disable(void);            /* 锁存停机 + 上报一次 */
void FOC_CutPower(void);           /* 直接切断驱动（可重复调用） */

void M0_Set_Angle(float Target);
void M1_Set_Angle(float Target);
void M0_Set_Velocity(float Target);
void M1_Set_Velocity(float Target);
void M0_Set_CurTorque(float Target);
void M1_Set_CurTorque(float Target);

#endif
