#ifndef __MPU6050_H
#define __MPU6050_H

#include <stdint.h>

/* ★ 软件 I2C 通信错误计数：任一事务出现 NACK 即 +1（ISR 用增量检测持续故障） */
extern volatile uint16_t MPU_I2C_ErrCnt;

/* 返回 0 = 成功，1 = I2C 通信失败 */
uint8_t MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data);
uint8_t MPU6050_ReadReg(uint8_t RegAddress);

void MPU6050_Init(void);
uint8_t MPU6050_GetID(void);
/* 返回 0 = 读取成功，1 = I2C 通信失败（数据无效） */
uint8_t MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
						int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ);

#endif
