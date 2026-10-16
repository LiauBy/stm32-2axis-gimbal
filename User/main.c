#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "Timer.h"
#include "MPU6050.h"
#include "IMU.h"
#include "Serial.h"
#include "DFOC.h"
#include <string.h>
#include <stdlib.h>

int16_t AX, AY, AZ, GX, GY, GZ;

// ★ 电机参数：M0=横滚轴，M1=俯仰轴
//   极对数按实际电机改；DIR 是编码器方向(±1)，补偿方向不对就调它或调 GAIN 符号
int M0_PP  = 7;
int M0_DIR = -1;
int M1_PP  = 7;
int M1_DIR = -1;

volatile uint8_t AngleSendFlag;				// 100ms 置位，主循环串口发送

// ★ 横滚/俯仰自稳参数（角度补偿法，二轴测试）
//   每个 10ms 周期：读 IMU_Roll/Pitch（度），让转子目标角 = -(当前角)，电机反向转同样的角度做补偿
//   *_GAIN 带符号：
//     -1.0 = 等量反向补偿（默认）；若越控越偏/发散，改成正号
//     若 MPU6050 装在云台上（测的是云台角），|gain|=1 只能补一半 → 加大到 2~4 更硬
//     数值越大响应越硬，但起步瞬间转子摆动越大、越容易振荡，请逐步加大
#define ROLL_TARGET    0.0f              // 目标横滚角（度）：锁 0°（水平）
#define ROLL_GAIN      -1.0f             // 横滚补偿增益（含方向符号）
#define PITCH_TARGET   0.0f              // 目标俯仰角（度）：锁 0°（水平）
#define PITCH_GAIN     1.0f             // 俯仰补偿增益（含方向符号）
#define DEG_TO_RAD     0.0174532925f     // 度 → 弧度

// ★ Level 1 俯仰外环角度低通时间常数(秒)，固定 10ms 采样。
//   作用：滤掉加速度计振动噪声灌进电机指令的"持续小抖"（只作用于俯仰轴，
//   横滚不滤波以保刚度）。调参：俯仰抖动明显→加大(0.03→0.05)；跟手慢→减小。
#define ANGLE_LPF_TF   0.04f

/* ============================================================
 * ★ 蓝牙摇杆控制（位置映射模式，协议与平衡车一致）
 * ------------------------------------------------------------
 *   手机APP发 [joystick,LH,LV,RH,RV]（±100），蓝牙模块接 USART1：
 *     右摇杆水平 RH → 横滚目标角偏移
 *     左摇杆竖直 LV → 俯仰目标角偏移
 *   位置映射：摇杆满偏(±100) = 偏移 JOY_RANGE_DEG 度；松手回中 → 云台回水平。
 *   自稳控制链不变，只是把"锁 0°"变成"锁 摇杆指定的角度"。
 * ============================================================ */
#define JOY_RANGE_DEG   45.0f    // 摇杆满偏对应的目标角偏移(度)。想活动范围更大就加大
#define JOY_DEADBAND    8        // 死区：|摇杆值|<8 视为回中，消除漂移
#define JOY_SIGN        1.0f     // 方向不对(推左云台往右)就改成 -1.0f
#define JOY_TIMEOUT_MS  500      // 蓝牙断连保护：500ms 没收到摇杆包 → 偏移归零(回水平)

/* 摇杆目标角偏移(度)：主循环写、TIM1 中断读。对齐的 float 写入在 M3 上原子。 */
volatile float g_JoyRollOff  = 0.0f;
volatile float g_JoyPitchOff = 0.0f;
/* 10ms 节拍计数（ISR 自增）：主循环记录收到摇杆包的时刻，ISR 据此判断断连 */
volatile uint16_t g_TenMsTick = 0;
volatile uint16_t g_JoyRxTick = 0;

int main(void)
{
	// ★ 上电时序：
	//   1) 串口最先初始化，任何一步卡住都能从 BOOT/进度帧定位问题；
	//   2) FOC_Init 内部"先 PWM_Init 再使能电机驱动"，避免使能瞬间 PWM 引脚浮空误动作；
	//   3) 电角对齐（约 7s）期间串口打印 ALGN 进度；
	//   4) 最后再启动 TIM1 中断和看门狗。
	Serial_Init();						// 串口（USART1 9600，TX=调试 PA9 / RX=蓝牙 PA10）
	Serial_SendString("BOOT\r\n");		// ★ 诊断帧：上电立即发送，不依赖任何外设

	FOC_Init(13.4);						// PWM → 电机使能 → 电角对齐（内部打印 ALGN 进度）

	IMU_Init();							// 初始化 MPU6050 并校准陀螺仪零偏

	// ★ 诊断：检查 MPU6050 是否在线（WHO_AM_I 应为 0x68）
	if (MPU6050_GetID() == 0x68)
		Serial_SendString("MPU6050 OK\r\n");
	else
		Serial_SendString("MPU6050 ERR\r\n");

	// ★ 电机上电初始化：M0/M1 先转到固定角度 0 rad 并等待到位，再启动自稳
	tripod_Iint();
	FOC_SoftStart_Begin();				// ★ 进入自稳前清零软启动斜坡（对齐/锁位阶段用满输出）

	Timer_Init();						// TIM1 1ms 中断，10ms 采样一次
	IWDG_Init();						// ★ 独立看门狗：主循环喂狗，1s 超时防挂死

	while (1)
	{
		IWDG_Feed();					// ★ 喂狗：主循环存活 = 系统正常

		// ★ 传感器故障上报（锁存，只上报一次，需重新上电恢复）
		if (FOC_Fault_Notify)
		{
			FOC_Fault_Notify = 0;
			Serial_SendString("SENSOR FAULT\r\n");
		}

		// ★ 蓝牙摇杆解析（移植自平衡车，协议一致）
		//   只认 joystick 帧；死区外把摇杆值映射成 ±JOY_RANGE_DEG 的目标角偏移。
		//   松手回中 → 偏移 0 → 云台回到水平锁定。
		if (Serial_RxFlag)
		{
			char *Tag = strtok(Serial_RxPacket, ",");
			if (Tag != NULL && strcmp(Tag, "joystick") == 0)
			{
				char *sLH = strtok(NULL, ",");
				char *sLV = strtok(NULL, ",");
				char *sRH = strtok(NULL, ",");
				char *sRV = strtok(NULL, ",");

				if (sLV != NULL && sRH != NULL)
				{
					int lv = atoi(sLV);      // 左摇杆竖直 → 俯仰
					int rh = atoi(sRH);      // 右摇杆水平 → 横滚

					float rollOff  = 0.0f, pitchOff = 0.0f;
					if (rh >  JOY_DEADBAND || rh < -JOY_DEADBAND)
						rollOff  = (float)rh / 100.0f * JOY_RANGE_DEG * JOY_SIGN;
					if (lv >  JOY_DEADBAND || lv < -JOY_DEADBAND)
						pitchOff = (float)lv / 100.0f * JOY_RANGE_DEG * JOY_SIGN;

					g_JoyRollOff  = rollOff;    // 限幅：JOY_RANGE_DEG ≤ 45°，天然满足防缠线
					g_JoyPitchOff = pitchOff;
					g_JoyRxTick   = g_TenMsTick;   // 记录收到包的时刻（断连检测用）
				}
			}
			Serial_RxFlag = 0;
		}

		// 定时串口发送姿态角 + 两个电机角（100ms 一帧）
		// 格式：Roll,Pitch,Yaw,M0Angle(rad),M1Angle(rad)（CSV，便于串口助手/上位机绘图）
		if (AngleSendFlag)
		{
			AngleSendFlag = 0;
			Serial_PrintFloat(IMU_Roll, 2);       //横滚角（自稳轴，应≈0）
			Serial_SendByte(',');
			Serial_PrintFloat(IMU_Pitch, 2);      //俯仰角（自稳轴，应≈0）
			Serial_SendByte(',');
			Serial_PrintFloat(IMU_Yaw, 2);        //航向角
			Serial_SendByte(',');
			Serial_PrintFloat(M0_DIR * M0.Enc_Angle, 2); //横滚电机转子角(rad)
			Serial_SendByte(',');
			Serial_PrintFloat(M1_DIR * M1.Enc_Angle, 2); //俯仰电机转子角(rad)
			Serial_SendByte(',');
			Serial_PrintFloat(FOC_GetStartRamp(), 3); //诊断：软启动斜坡(0~1)，正常应到1.000
			Serial_SendString("\r\n");
		}
	}
}

void TIM1_UP_IRQHandler(void)
{
	static uint8_t Count0, AngleTick;
	static float PitchF = 0.0f;                 // ★ Level 1 俯仰外环角度低通滤波状态（横滚不滤波，保刚度）
	// ★ 传感器故障监视：两条软件 I2C 总线的错误计数增量 + 连续故障周期计数
	static uint16_t SensorFault = 0;
	static uint16_t last_mpu_err = 0;
	static uint16_t last_as5600_err = 0;

	if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
	{
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);

		Count0 ++;
		if (Count0 >= 10)				// 每 10ms
		{
			Count0 = 0;

			// 已触发故障停机：保持切断，不再控制
			if (FOC_Fault_Flag) { FOC_CutPower(); return; }

			// ★ 软启动斜坡推进。★ 移到故障门之前：偶发的单次 I2C 故障
			//   只是跳过本周期控制，不该把软启动斜坡一直卡在 0 附近。
			FOC_SoftStart_Tick();

			// ★ 先读 MPU（读取失败会自增 MPU_I2C_ErrCnt）
			uint8_t mpu_read_fail = (MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ) != 0);

			// ★ I2C 校验：本周期与上周期错误计数之差非 0 = 有传感器通信失败
			//   连续 20 个周期（200ms）故障 → 锁存停机，防止垃圾数据驱动电机失控
			{
				uint16_t mpu_delta = (uint16_t)(MPU_I2C_ErrCnt - last_mpu_err);
				uint16_t as_delta  = (uint16_t)(AS5600_I2C_ErrCnt - last_as5600_err);
				last_mpu_err  = MPU_I2C_ErrCnt;
				last_as5600_err = AS5600_I2C_ErrCnt;

				if (mpu_read_fail || mpu_delta || as_delta)
				{
					if (++SensorFault >= 20)
					{
						FOC_Disable();			// 锁存停机 + 上报
						return;
					}
					return;						// 偶发故障：本周期跳过控制，保持上次输出
				}
				SensorFault = 0;
			}

			// ★ 传感器数据有效：Mahony 互补滤波解算三轴角
			IMU_Update(AX, AY, AZ, GX, GY, GZ, 0.01f);

			// ★ 蓝牙摇杆：10ms 节拍 + 断连保护。
			//   超过 500ms 没收到摇杆包（断连/关APP）→ 偏移归零，云台回到水平锁定。
			//   无符号减法，16 位计数回绕也正确。
			g_TenMsTick++;
			if ((uint16_t)(g_TenMsTick - g_JoyRxTick) > (JOY_TIMEOUT_MS / 10))
			{
				g_JoyRollOff  = 0.0f;
				g_JoyPitchOff = 0.0f;
			}

			// ★ Level 1：把角速度复制到 DFOC 全局，供 M0/M1_Set_Angle 做扭矩级阻尼
			g_IMU_RollRate  = IMU_RollRate;
			g_IMU_PitchRate = IMU_PitchRate;

			// ★ Level 1：俯仰外环角度低通滤波（一阶，固定10ms采样）。
			//   只滤俯仰（解决俯仰抖动）；横滚不滤波，避免相位滞后让横滚发软。
			//   只滤角度、不滤速率阻尼（阻尼项用的是原始陀螺速率）。
			{
				float a = ANGLE_LPF_TF / (ANGLE_LPF_TF + 0.01f);
				PitchF = a * PitchF + (1.0f - a) * IMU_Pitch;
			}

			// ★ 横滚自稳（M0）：转子目标角 = ROLL_GAIN × (目标角 + 摇杆偏移 − 当前横滚角)，度→弧度
			//   M0_Set_Angle 用 AS5600 位置闭环把转子伺服到该角 → 电机反向转同样的角度补偿
			//   摇杆偏移由主循环解析蓝牙包写入；摇杆回中 → 偏移 0 → 锁回水平
			{
				float Motor_Target = ROLL_GAIN * (ROLL_TARGET + g_JoyRollOff - IMU_Roll) * DEG_TO_RAD;
				M0_Set_Angle(Motor_Target);
			}
//			// ★ 俯仰自稳（M1）
//			// ★★★ TEMP 诊断：临时改成 ±0.5rad（约±28°）三角波扫掠，测试俯仰电机能否转起来。
//			//   看串口两列：
//			//     M1Angle 列  能画出 ±0.5 的三角波 → 电机能转 → 问题在传感器/控制
//			//     M1Angle 列  平线不动或嗡嗡 → 电机缺相/相序/接线问题
//			//     IMU_Pitch 列 跟随三角波 → 传感器和轴映射正常；不跟 → MPU 装歪
//			//   测完恢复为下面被注释掉的正常自稳。
//			{
//				static int32_t m1_sweep = 0;
//				static int32_t m1_sweep_dir = 1;
//				M1_Set_Angle(m1_sweep * 0.01f);
//				m1_sweep += m1_sweep_dir;
//				if (m1_sweep > 50) m1_sweep_dir = -1;
//				else if (m1_sweep < -50) m1_sweep_dir = 1;
//			}
			 {
			   float Motor_Target = PITCH_GAIN * (PITCH_TARGET + g_JoyPitchOff - PitchF) * DEG_TO_RAD;
			 	 M1_Set_Angle(Motor_Target);
			 }

			if (++AngleTick >= 10)		// 每 100ms
			{
				AngleTick = 0;
				AngleSendFlag = 1;
			}
		}
	}
}
