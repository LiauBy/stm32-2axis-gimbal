#ifndef __SERIAL_H
#define __SERIAL_H

#include <stdint.h>

/* ★ 蓝牙接收缓冲区大小（[joystick,LH,LV,RH,RV] 约 24 字节，留余量） */
#define SERIAL_RX_BUF_SIZE  100

/* ★ 蓝牙接收：ISR 拆帧填入 RxPacket，收齐一帧置 RxFlag=1；主循环解析后清 0 */
extern char Serial_RxPacket[SERIAL_RX_BUF_SIZE];
extern volatile uint8_t Serial_RxFlag;

void Serial_Init(void);
void Serial_SendByte(uint8_t Byte);
void Serial_SendArray(uint8_t *Array, uint16_t Length);
void Serial_SendString(char *String);
void Serial_Printf(char *format, ...);
void Serial_PrintFloat(float Value, uint8_t Decimals);

#endif
