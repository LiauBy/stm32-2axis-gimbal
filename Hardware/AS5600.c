#include "stm32f10x.h"
#include "IWI2C.h"
#include "MyI2C.h"
#include "Serial.h"
#include "Delay.h"
#include <stdio.h>
#include <math.h>
#include "AS5600.h"

#define AS5600_ADDRESS        0x6C //I2C从机地址
#define AS5600_RAW_ANGLE_H    0x0C
#define AS5600_RAW_ANGLE_L    0x0D

#define PI       3.14159265359f
#define _2PI       6.28318530718f

// ★ 软件 I2C 通信错误计数：任一事务出现 NACK 即 +1（ISR 用增量检测持续故障）
volatile uint16_t AS5600_I2C_ErrCnt = 0;

// 地址字节 ACK 检查：返回 1 = 从机不应答（错误）
static uint8_t AS_CheckAck(void)
{
	if (I2C_RecviveAck())
	{
		AS5600_I2C_ErrCnt++;
		return 1;
	}
	return 0;
}

uint8_t AS5600_WriteReg(uint8_t RegAddress, uint8_t Data)
{
	I2C_Start();
	I2C_SendByte(AS5600_ADDRESS);
	if (AS_CheckAck()) { I2C_Stop(); return 1; }
	I2C_SendByte(RegAddress);
	if (AS_CheckAck()) { I2C_Stop(); return 1; }
	I2C_SendByte(Data);
	if (AS_CheckAck()) { I2C_Stop(); return 1; }
	I2C_Stop();
	return 0;
}

uint8_t AS5600_ReadReg(uint8_t RegAddress)
{
	uint8_t Data;

	I2C_Start();
	I2C_SendByte(AS5600_ADDRESS);
	if (AS_CheckAck()) { I2C_Stop(); return 0; }
	I2C_SendByte(RegAddress);
	if (AS_CheckAck()) { I2C_Stop(); return 0; }

	I2C_Start();
	I2C_SendByte(AS5600_ADDRESS | 0x01);
	if (AS_CheckAck()) { I2C_Stop(); return 0; }
	Data = I2C_RecviveData();
	I2C_SendAck(1);
	I2C_Stop();

	return Data;
}

uint8_t  AS5600_ReadNowReg(void)
{
	uint8_t Data;
	
	I2C_Start();
	I2C_SendByte(AS5600_ADDRESS | 0x01);
	I2C_RecviveAck();
	Data = I2C_RecviveData();
	I2C_SendAck(1);
	I2C_Stop();
	
	return Data;
}

// 读取 12bit 原始角度（0~4095）。
// 通信失败返回 -1（正常返回值恒为 0~4095，不会与 -1 冲突）。
float AS5600_GetRawData(void)
{
	uint8_t Data_L;
	uint8_t Data_H;

	I2C_Start();
	I2C_SendByte(AS5600_ADDRESS);
	if (AS_CheckAck()) { I2C_Stop(); return -1; }
	I2C_SendByte(AS5600_RAW_ANGLE_H);
	if (AS_CheckAck()) { I2C_Stop(); return -1; }

	I2C_Start();
	I2C_SendByte(AS5600_ADDRESS | 0x01);
	if (AS_CheckAck()) { I2C_Stop(); return -1; }
	Data_H = I2C_RecviveData();
	I2C_SendAck(0);				// 主机 ACK：继续读下一字节
	Data_L = I2C_RecviveData();
	I2C_SendAck(1);				// 主机 NACK：结束读取
	I2C_Stop();

	return ((Data_H << 8) | Data_L);
}

void AS5600_Init(void)
{
	MyI2C_Init();
}

float GetAngle_NoTrack(struct AS5600_Sensor *AS5600)
{
	static float LastGoodAngle[2] = {0.0f, 0.0f};	// 每路编码器最后一次有效角(rad)

	Set_Ang_Sensor(AS5600->Mot_num);

	float Raw = AS5600_GetRawData();
	if (Raw < 0)								// ★ 读失败：沿用上次有效值，避免位置/电角跳变
		return LastGoodAngle[AS5600->Mot_num];

	LastGoodAngle[AS5600->Mot_num] = (Raw / 4096.0f) * _2PI;
	return LastGoodAngle[AS5600->Mot_num];
}


float GetAngle(struct AS5600_Sensor *AS5600)
{
//	float D_Angle = 0.0;
	AS5600->Angle = GetAngle_NoTrack(AS5600);
	float D_Angle = AS5600->Angle - AS5600->Last_Angle;

	if( fabs(D_Angle) > (0.8f*2*PI) )
	{
		AS5600->full_rotations = AS5600->full_rotations + ((D_Angle > 0) ? -1 :1);
	}
	
	AS5600->Last_Angle = AS5600->Angle;
	
	AS5600->Angle = (AS5600->full_rotations * _2PI + AS5600->Last_Angle);
	
	return AS5600->Angle;
}

float GetVelocity(struct AS5600_Sensor *AS5600_Vel)
{
	float dt = 0.0;
	float Vel_ts = SysTick -> VAL;
	if(Vel_ts < AS5600_Vel->Last_Vel_ts) dt = (AS5600_Vel->Last_Vel_ts - Vel_ts)/9*1e-6f;
	else dt = (0xFFFFFF - Vel_ts + AS5600_Vel->Last_Vel_ts)/9*1e-6f;
	
	if(dt < 0.0001) dt = 10000;
	
	float Vel_Angle = GetAngle(AS5600_Vel);
	
	float dv = Vel_Angle - AS5600_Vel->Vel_Last_Angle;

	AS5600_Vel->velocity = (Vel_Angle - AS5600_Vel->Vel_Last_Angle)/dt;

	AS5600_Vel->Last_Vel_ts = Vel_ts;
  AS5600_Vel->Vel_Last_Angle = Vel_Angle;
  
  return AS5600_Vel->velocity;
}


