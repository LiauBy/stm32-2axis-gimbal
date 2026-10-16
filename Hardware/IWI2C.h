#ifndef __IWI2C_H
#define __IWI2C_H

void Set_Ang_Sensor(int Mot);
void I2C_W_SCL(uint8_t BitValue);
void I2C_W_SDA(uint8_t BitValue);
uint8_t I2C_R_SDA(void);
void I2C_Start(void);
void I2C_Stop(void);
void I2C_SendByte(uint8_t Byte);
uint8_t I2C_RecviveData(void);
void I2C_SendAck(uint8_t AckBit);
uint8_t I2C_RecviveAck(void);




#endif




