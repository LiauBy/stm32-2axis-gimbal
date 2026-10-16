#include "stm32f10x.h"                  // Device header
#include "MyI2C.h"
#include "MPU6050_Reg.h"

#define MPU6050_ADDRESS		0xD0

// ★ 软件 I2C 通信错误计数：任一事务出现 NACK 即 +1（ISR 用增量检测持续故障）
volatile uint16_t MPU_I2C_ErrCnt = 0;

// 地址字节 ACK 检查：返回 1 = 从机不应答（错误）
static uint8_t MPU_CheckAck(void)
{
	if (MyI2C_ReceiveAck())
	{
		MPU_I2C_ErrCnt++;
		return 1;
	}
	return 0;
}

uint8_t MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{
	MyI2C_Start();
	MyI2C_SendByte(MPU6050_ADDRESS);
	if (MPU_CheckAck()) { MyI2C_Stop(); return 1; }
	MyI2C_SendByte(RegAddress);
	if (MPU_CheckAck()) { MyI2C_Stop(); return 1; }
	MyI2C_SendByte(Data);
	if (MPU_CheckAck()) { MyI2C_Stop(); return 1; }
	MyI2C_Stop();
	return 0;
}

uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
	uint8_t Data;

	MyI2C_Start();
	MyI2C_SendByte(MPU6050_ADDRESS);
	if (MPU_CheckAck()) { MyI2C_Stop(); return 0; }
	MyI2C_SendByte(RegAddress);
	if (MPU_CheckAck()) { MyI2C_Stop(); return 0; }

	MyI2C_Start();
	MyI2C_SendByte(MPU6050_ADDRESS | 0x01);
	if (MPU_CheckAck()) { MyI2C_Stop(); return 0; }
	Data = MyI2C_ReceiveByte();
	MyI2C_SendAck(1);
	MyI2C_Stop();

	return Data;
}

uint8_t MPU6050_ReadRegs(uint8_t RegAddress, uint8_t *DataArray, uint8_t Count)
{
	uint8_t i;

	MyI2C_Start();
	MyI2C_SendByte(MPU6050_ADDRESS);
	if (MPU_CheckAck()) { MyI2C_Stop(); return 1; }
	MyI2C_SendByte(RegAddress);
	if (MPU_CheckAck()) { MyI2C_Stop(); return 1; }

	MyI2C_Start();
	MyI2C_SendByte(MPU6050_ADDRESS | 0x01);
	if (MPU_CheckAck()) { MyI2C_Stop(); return 1; }
	for (i = 0; i < Count; i ++)
	{
		DataArray[i] = MyI2C_ReceiveByte();
		if (i < Count - 1)
		{
			MyI2C_SendAck(0);
		}
		else
		{
			MyI2C_SendAck(1);
		}
	}
	MyI2C_Stop();
	return 0;
}

void MPU6050_Init(void)
{
	MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);
	MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);
	MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x07);
	MPU6050_WriteReg(MPU6050_CONFIG, 0x00);
	MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);
	MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18);
}

uint8_t MPU6050_GetID(void)
{
	return MPU6050_ReadReg(MPU6050_WHO_AM_I);
}

//void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ, 
//						int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
//{
//	uint8_t DataH, DataL;
//	
//	DataH = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H);
//	DataL = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L);
//	*AccX = (DataH << 8) | DataL;
//	
//	DataH = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H);
//	DataL = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L);
//	*AccY = (DataH << 8) | DataL;
//	
//	DataH = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H);
//	DataL = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L);
//	*AccZ = (DataH << 8) | DataL;
//	
//	DataH = MPU6050_ReadReg(MPU6050_GYRO_XOUT_H);
//	DataL = MPU6050_ReadReg(MPU6050_GYRO_XOUT_L);
//	*GyroX = (DataH << 8) | DataL;
//	
//	DataH = MPU6050_ReadReg(MPU6050_GYRO_YOUT_H);
//	DataL = MPU6050_ReadReg(MPU6050_GYRO_YOUT_L);
//	*GyroY = (DataH << 8) | DataL;
//	
//	DataH = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H);
//	DataL = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);
//	*GyroZ = (DataH << 8) | DataL;
//}

// 返回 0 = 读取成功，1 = I2C 通信失败（数据无效，调用方应跳过姿态更新）
uint8_t MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
						int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{
	uint8_t Data[14];

	if (MPU6050_ReadRegs(MPU6050_ACCEL_XOUT_H, Data, 14) != 0)
		return 1;

	*AccX = (Data[0] << 8) | Data[1];
	*AccY = (Data[2] << 8) | Data[3];
	*AccZ = (Data[4] << 8) | Data[5];

	*GyroX = (Data[8] << 8) | Data[9];
	*GyroY = (Data[10] << 8) | Data[11];
	*GyroZ = (Data[12] << 8) | Data[13];

	return 0;
}
