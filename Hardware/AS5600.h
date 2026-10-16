#ifndef __AS5600_H
#define __AS5600_H

#include <stdint.h>

struct AS5600_Sensor{
	int Mot_num;
	float Angle;
	float velocity;
	float full_rotations;
	float Last_Angle;
	float Last_Vel_ts;
	float Vel_Last_Angle;
};

/* ★ 软件 I2C 通信错误计数：任一事务出现 NACK 即 +1（ISR 用增量检测持续故障） */
extern volatile uint16_t AS5600_I2C_ErrCnt;

uint8_t AS5600_WriteReg(uint8_t RegAddress, uint8_t Data);
uint8_t  AS5600_ReadReg(uint8_t RegAddress);
uint8_t  AS5600_ReadNowReg(void);
void AS5600_Init(void);
float AS5600_GetRawData(void);
float GetAngle(struct AS5600_Sensor *AS5600);
float GetAngle_NoTrack(struct AS5600_Sensor *AS5600);
float GetVelocity(struct AS5600_Sensor *AS5600_Vel);
#endif
