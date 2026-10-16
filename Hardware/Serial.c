#include "stm32f10x.h"                  // Device header
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include "Serial.h"

/* ★ 蓝牙接收缓冲与标志（协议与平衡车一致：[joystick,LH,LV,RH,RV]） */
char Serial_RxPacket[SERIAL_RX_BUF_SIZE];
volatile uint8_t Serial_RxFlag = 0;

/* ★ USART1 现在身兼两职：
 *   PA9  (TX) —— 调试输出（BOOT/ALGN/CSV），同时接蓝牙模块 RXD → 手机可无线看调试数据
 *   PA10 (RX) —— 蓝牙模块 TXD → 摇杆指令输入（原为空闲引脚）
 *   波特率 9600：与平衡车同一蓝牙模块（HC-05/06）一致。
 *   调试 CSV 每 100ms 一帧约 40 字节 ≈ 3.3kbps，9600 带宽足够。 */
void Serial_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* ★ PA10 上拉输入：蓝牙没插时引脚稳定为高，不会收到乱码 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART1, &USART_InitStructure);

	/* ★ 接收中断使能 */
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	/* ★ 抢占优先级 1 > TIM1 的 2：串口 ISR 极短（约 20 条指令），
	 *   但 TIM1 里软件 I2C 位操作可能占 0.5ms 以上，串口中断必须能打断它，
	 *   否则 9600 波特下（约 1ms 一字节）会溢出丢字节。 */
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);

	USART_Cmd(USART1, ENABLE);
}

void Serial_SendByte(uint8_t Byte)
{
	uint32_t Timeout = 0xFFFF;				// 超时保护，防止 USART 异常时死循环
	USART_SendData(USART1, Byte);
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
	{
		if (--Timeout == 0) return;			// 超时放弃发送
	}
}

void Serial_SendString(char *String)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i++)
	{
		Serial_SendByte((uint8_t)String[i]);
	}
}

void Serial_SendArray(uint8_t *Array, uint16_t Length)
{
	uint16_t i;
	for (i = 0; i < Length; i++)
	{
		Serial_SendByte(Array[i]);
	}
}

/* 格式化发送（支持 %d 等整型格式化，%f 请用 Serial_PrintFloat） */
void Serial_Printf(char *format, ...)
{
	char String[100];
	va_list arg;
	va_start(arg, format);
	vsnprintf(String, sizeof(String), format, arg);
	va_end(arg);
	Serial_SendArray((uint8_t *)String, (uint16_t)strlen(String));
}

/* 发送浮点数（定点打印，不用 %f，避免标准库浮点打印的栈开销与依赖） */
void Serial_PrintFloat(float Value, uint8_t Decimals)
{
	char String[16];
	uint8_t i;
	int32_t IntPart, DecPart;
	float AbsVal = Value;
	uint32_t Pow10 = 1;
	uint32_t Div;

	for (i = 1; i < Decimals; i++) Pow10 *= 10;	/* 10^(dec-1) */
	Div = Pow10 * 10;

	if (Value < 0)
	{
		Serial_SendByte('-');
		AbsVal = -Value;
	}

	IntPart = (int32_t)AbsVal;
	DecPart = (int32_t)((AbsVal - IntPart) * (float)Div + 0.5f);	/* 四舍五入 */
	if (DecPart >= (int32_t)Div) { DecPart = 0; IntPart++; }

	i = 0;
	do {
		String[i++] = (char)(IntPart % 10 + '0');
		IntPart /= 10;
	} while (IntPart > 0);
	while (i > 0) Serial_SendByte((uint8_t)String[--i]);

	if (Decimals > 0)
	{
		Serial_SendByte('.');
		for (i = 0; i < Decimals; i++)
		{
			uint8_t d = (uint8_t)(DecPart / Pow10);
			Serial_SendByte((uint8_t)(d + '0'));
			DecPart %= (int32_t)Pow10;
			Pow10 /= 10;
		}
	}
}

/* ★ 蓝牙接收中断（移植自平衡车 BlueSerial.c，协议不变）：
 *   状态机拆帧——'[' 开始收集，']' 结束置标志。
 *   收到完整一帧前若上一帧未处理完（RxFlag=1），新帧直接丢弃（防撕裂）。
 *   缓冲区带边界保护：超长/无 ']' 的垃圾数据不会溢出，靠下一个 '[' 重新同步。 */
void USART1_IRQHandler(void)
{
	static uint8_t RxState = 0;
	static uint8_t pRxPacket = 0;

	if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
	{
		uint8_t RxData = USART_ReceiveData(USART1);

		if (RxState == 0)
		{
			if (RxData == '[' && Serial_RxFlag == 0)
			{
				RxState = 1;
				pRxPacket = 0;
			}
		}
		else if (RxState == 1)
		{
			if (RxData == ']')
			{
				RxState = 0;
				Serial_RxPacket[pRxPacket] = '\0';
				Serial_RxFlag = 1;
			}
			else if (pRxPacket < SERIAL_RX_BUF_SIZE - 1)
			{
				Serial_RxPacket[pRxPacket] = RxData;
				pRxPacket++;
			}
		}

		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	}
}
