#include "stm32f10x.h"
#include "PID_Control.h"

//限幅函数
float _constrain(float amt, float low, float high)
{
	return ((amt<low)?(low):((amt)>(high)?(high):(amt)));
}

void PID_Init(PID_t *pid, float Kp, float Ki, float Kd, float limit)
{
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;
	pid->limit = limit;
	pid->Last_Error = 0.0f;
	pid->Last_integration = 0.0f;
}

/* Ts: 采样周期(秒)，由调用方固定传入（固定时间步长方案） */
float PID_Controller(PID_t *pid, float Error, float Ts)
{
	float proportion = pid->Kp * Error;//P

	float integration = pid->Last_integration + pid->Ki * 0.5f * Ts * Error;//I
	integration = _constrain(integration, -pid->limit, pid->limit);

	float differential = pid->Kd * (Error - pid->Last_Error)/Ts;//D

	float Output = proportion + integration + differential;
	Output = _constrain(Output, -pid->limit, pid->limit);

	pid->Last_Error = Error;
	pid->Last_integration = integration;

	return Output;
}
