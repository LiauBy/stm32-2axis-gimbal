#include "stm32f10x.h"
#include "Delay.h"
#include "MyI2C.h"

uint16_t SCL_PIN, SDA_PIN;

//I2C引脚映射
void Set_Ang_Sensor(int Mot)
{
	if(Mot == 0)
	{
		SCL_PIN = GPIO_Pin_10;
		SDA_PIN = GPIO_Pin_11;
	}
	else if(Mot == 1)
	{
		SCL_PIN = GPIO_Pin_6;
		SDA_PIN = GPIO_Pin_7;
	}
}

//��������I2CЭ��
void I2C_W_SCL(uint8_t BitValue)
{
	GPIO_WriteBit(GPIOB, SCL_PIN, (BitAction)BitValue);
	Delay_us(1);	// ★ SCL 高低电平脉宽 ≥1µs，fSCL≈320kHz（快速模式 400kHz 内）
}

void I2C_W_SDA(uint8_t BitValue)
{
	GPIO_WriteBit(GPIOB, SDA_PIN, (BitAction)BitValue);
	Delay_us(1);	// ★ SDA 建立时间：SCL 上升沿前 ≥1µs
}

//��ȡʱ��������
uint8_t I2C_R_SDA(void)
{
	uint8_t BitValue;
	BitValue = GPIO_ReadInputDataBit(GPIOB, SDA_PIN);

	return BitValue;
}

void I2C_Start(void)
{
	I2C_W_SCL(1);
	I2C_W_SDA(1);
	I2C_W_SDA(0);
	I2C_W_SCL(0);
}

void I2C_Stop(void)
{
	I2C_W_SDA(0);
	I2C_W_SCL(1);
	I2C_W_SDA(1);

}

void I2C_SendByte(uint8_t Byte)
{
	uint8_t i;
	
	for(i=0;i<8;i++)
	{
		I2C_W_SDA(Byte & (0x80>>i));
		I2C_W_SCL(1);
		I2C_W_SCL(0);
	}

}

uint8_t I2C_RecviveData(void)
{
	uint8_t i, Byte = 0x00;
	I2C_W_SDA(1);
	for(i=0;i<8;i++)
	{
		I2C_W_SCL(1);
		if(I2C_R_SDA() == 1) {Byte |= (0x80 >> i);}
		I2C_W_SCL(0);

	}
	return Byte;
	
}

void I2C_SendAck(uint8_t AckBit)
{

		I2C_W_SDA(AckBit);
		I2C_W_SCL(1);
		I2C_W_SCL(0);

}

uint8_t I2C_RecviveAck(void)
{
	uint8_t AckBit;

	I2C_W_SDA(1);
	I2C_W_SCL(1);
	AckBit = I2C_R_SDA();
	I2C_W_SCL(0);
	
	return AckBit;
}









