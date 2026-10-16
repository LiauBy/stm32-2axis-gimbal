#ifndef __PID_CONTROL_H
#define __PID_CONTROL_H

/* 单实例 PID：每轴一份状态（M0 / M1 分开），避免互相污染 D 项与前次误差 */
typedef struct{
	float Kp;
	float Ki;
	float Kd;
	float limit;             /* 输出限幅（V） */
	float Last_Error;
	float Last_integration;
} PID_t;

void PID_Init(PID_t *pid, float Kp, float Ki, float Kd, float limit);
float PID_Controller(PID_t *pid, float Error, float Ts);
float _constrain(float amt, float low, float high);

#endif
