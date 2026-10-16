#include "stm32f10x.h"
#include <math.h>
#include <stdlib.h>
#include "PWM.h"
#include "AS5600.h"
#include "Delay.h"
#include "Serial.h"
#include "Lowpass_Filter.h"
#include "PID_Control.h"
#include "Inlinecurrent.h"
#include "DFOC.h"

#define PI         3.14159265359f
#define _2PI       6.28318530718f
#define _3PI_2     4.71238898f
#define DEG_TO_RAD 0.0174532925f

// 保持点死区消除摆动 Uq output deadband (V): below this, no motor drive -> kills hold-point limit cycle
#define UQ_DEADBAND   0.05f   // ★ Level 1: 0.12 → 0.05，减小回中位"松垮/极限环"

// ★ Level 1 扭矩级角速度阻尼系数 (V per rad/s)：
//   直接抵抗相机转动，吸收振荡能量。
//   ★ 符号约定（已按各自增益方向定好，勿一起改）：
//     M1(俯仰) PITCH_GAIN=+1 → 用 `Uq -= ...`
//     M0(横滚) ROLL_GAIN=-1  → 用 `Uq += ...`（符号相反）
//   若某轴出现"锁死/跑飞"且注释不符 → 只把那轴那行符号反过来即可。
//   应急兜底：某轴仍异常 → 把该轴阻尼临时设 0，恢复原始行为再排查。
#define M0_RATE_DAMP   0.15f
#define M1_RATE_DAMP   0.15f

// ★ 上电锁位策略（防缠线）：
//   不再强制转子转到编码器绝对 0（那会造成最多一整圈的回摆、缠线），
//   而是把当前转子位置直接捕获为自稳参考零点（见 tripod_Iint），原地保持。

struct Motor_ M0 = {0};
struct Motor_ M1 = {1};

PID_t M0_ANG_PID;
PID_t M1_ANG_PID;
PID_t M0_VEL_PID;
PID_t M1_VEL_PID;

// ★ Level 1 相机角速度（deg/s）：ISR 从 IMU 复制到此，角度环读取做扭矩级阻尼。
//   tripod 锁位阶段为 0，不影响上电锁位。
volatile float g_IMU_RollRate  = 0.0f;
volatile float g_IMU_PitchRate = 0.0f;

struct AS5600_Sensor Angle_Sensor0 = {0};
struct AS5600_Sensor Angle_Sensor1 = {1};

float voltage_limit = 12;
float voltage_power_supply = 0;
float M0_zero_elc_Angle = 0, M1_zero_elc_Angle = 0;
extern int M0_PP , M0_DIR ;
extern int M1_PP , M1_DIR ;

// ★ 软启动：进入自稳后的前 START_RAMP_CYCLES 个周期，Uq 按 0→1 斜坡递增
//   上电默认 1.0（tripod 锁位/对齐需要满输出），FOC_SoftStart_Begin() 后从 0 开始爬升
#define START_RAMP_CYCLES   50              // 50 × 10ms = 0.5s
static float g_StartRamp = 1.0f;

// ★ 上电自稳参考零点：tripod_Iint 捕获当前转子多圈角，自稳误差相对它计算。
//   配合 AngleErr() 的最短路径，转子永远从当前位置出发、最多转动目标所需角度，
//   杜绝上电/自稳时多圈旋转缠线。
static float M0_angle_ref = 0.0f;
static float M1_angle_ref = 0.0f;

// ★ 传感器故障锁存（ISR 检测到 I2C 持续失败后置位）
volatile uint8_t FOC_Fault_Flag   = 0;      // 1 = 已停机（锁存，需重新上电）
volatile uint8_t FOC_Fault_Notify = 0;      // 1 = 待主循环串口上报一次

//电机使能
void Motor_en(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	GPIO_SetBits(GPIOA, GPIO_Pin_8);
}

//限幅函数
float constrain(float amt, float low, float high)
{
	 return ((amt<low)?(low):((amt)>(high)?(high):(amt)));
}

//角度归一化，避免角度错乱一直转
float normalizeAngle(float angle)
{
	float a = fmod(angle, 2*PI);
	return ((a>=0) ? a : (a + 2*PI));
}

// ★ 角度误差归一化到 (-π, π]：取最短路径。
//   即使多圈累计角 / 参考零点相差 2π 的整数倍，误差也不会超过半圈，
//   转子最多转动目标所需的角度，绝无整圈旋转。
static float AngleErr(float err)
{
	float a = fmodf(err + PI, _2PI);
	if (a < 0.0f) a += _2PI;
	return a - PI;
}

//电角度计算函数：机械角度*极对数
float M0_electricAngle(void)
{
	return normalizeAngle((GetAngle_NoTrack(&Angle_Sensor0) * M0_PP * M0_DIR) - M0_zero_elc_Angle);
}

float M1_electricAngle(void)
{
	return normalizeAngle((GetAngle_NoTrack(&Angle_Sensor1) * M1_PP * M1_DIR) - M1_zero_elc_Angle);
}

//根据三相电压输出三相PWM驱动电机
void SetPwm(int Mot_num, float Ua, float Ub, float Uc)
{
	float U_a=0.0;
	float U_b=0.0;
	float U_c=0.0;

	U_a = constrain(Ua, 0.0f, voltage_limit);
	U_b = constrain(Ub, 0.0f, voltage_limit);
	U_c = constrain(Uc, 0.0f, voltage_limit);

	if(Mot_num == 0)
	{
		M0.dc_a = constrain(U_a / voltage_power_supply, 0.0f, 1.0f);
		M0.dc_b = constrain(U_b / voltage_power_supply, 0.0f, 1.0f);
		M0.dc_c = constrain(U_c / voltage_power_supply, 0.0f, 1.0f);

		M0_PWM_A(M0.dc_a * 4800.0f);  // 频率15k
		M0_PWM_B(M0.dc_b * 4800.0f);
		M0_PWM_C(M0.dc_c * 4800.0f);
	}
	else if(Mot_num == 1)
	{
		M1.dc_a = constrain(U_a / voltage_power_supply, 0.0f, 1.0f);
		M1.dc_b = constrain(U_b / voltage_power_supply, 0.0f, 1.0f);
		M1.dc_c = constrain(U_c / voltage_power_supply, 0.0f, 1.0f);

		M1_PWM_A(M1.dc_a * 4800.0f);  // 频率15k
		M1_PWM_B(M1.dc_b * 4800.0f);
		M1_PWM_C(M1.dc_c * 4800.0f);
	}
}

//FOC算法核心：逆帕克+反克拉克，根据需要的力矩Uq计算三相电压
void SetPhaseVoltage(struct Motor_ *Motor, float Uq, float angle_el)
{
	Motor->Ualpha = -Uq*sin(angle_el);
	Motor->Ubeta = Uq*cos(angle_el);

	Motor->Ua = Motor->Ualpha + voltage_power_supply / 2;
	Motor->Ub = (sqrt(3)*Motor->Ubeta - Motor->Ualpha) / 2 + voltage_power_supply / 2;
	Motor->Uc = -(Motor->Ualpha + sqrt(3)*Motor->Ubeta) / 2 + voltage_power_supply / 2;

	SetPwm(Motor->Mot_num, Motor->Ua,Motor->Ub,Motor->Uc);
}

//校准传感器（编码器）：M0 与 M1 依次通电对齐电角零位
//注意：本函数约 7s，期间串口打印进度（Serial_Init 需在 FOC_Init 之前调用）
void Check_Sensor(void)
{
	Serial_SendString("ALGN M0\r\n");
	SetPhaseVoltage(&M0, 3, _3PI_2);
	delay_ms(3000);
	M0_zero_elc_Angle = M0_electricAngle();
	SetPhaseVoltage(&M0, 0, _3PI_2);
	delay_ms(500);

	Serial_SendString("ALGN M1\r\n");
	SetPhaseVoltage(&M1, 3, _3PI_2);
	delay_ms(3000);
	M1_zero_elc_Angle = M1_electricAngle();
	SetPhaseVoltage(&M1, 0, _3PI_2);
	delay_ms(500);

	Serial_SendString("ALGN DONE\r\n");
}

void FOC_Init(float power)
{
	voltage_power_supply = power;

	// ★ 上电时序修正：先初始化 PWM（配置 PA0~PA7 为复用推挽输出），
	//   再使能电机驱动（PA8 拉高）——避免使能瞬间 PWM 引脚仍是浮空输入导致驱动误动作。
	//   电角对齐需要通电，所以使能必须在 Check_Sensor 之前。
	PWM_Init();
	Motor_en();

	AS5600_Init();

	// 角度环 / 速度环 PID：每轴独立实例（误差单位为弧度，Kp 按 V/rad）
	// ★ 位置环 Kp 分轴：M0(横滚) 5.0 保持原刚度（只加阻尼、不动刚度）；
	//   M1(俯仰) 3.0 防俯仰位置环振铃（阻尼已由 M1_RATE_DAMP 承担）
	PID_Init(&M0_ANG_PID, 5.0f, 0.0f, 0.0f, 6.7f);
	PID_Init(&M1_ANG_PID, 3.0f, 0.0f, 0.0f, 6.7f);
	PID_Init(&M0_VEL_PID, 0.5f, 0.0f, 0.0f, 6.7f);
	PID_Init(&M1_VEL_PID, 0.5f, 0.0f, 0.0f, 6.7f);

	Check_Sensor();
}

// ★ M0（横滚）角度环：Target 单位 rad，用 AS5600 多圈绝对角做位置闭环
void M0_Set_Angle(float Target)
{
	float angle = GetAngle(&Angle_Sensor0);
	M0.Enc_Angle = angle;
	// ★ 防缠线：误差相对上电参考零点计算，并归一化到 (-π,π] 取最短路径。
	//   转子最多转动 |Target|（≤180°），不会因多圈累计/零点偏差绕一整圈。
	float err = AngleErr(Target - M0_DIR * (angle - M0_angle_ref));
	float Uq = PID_Controller(&M0_ANG_PID, err, 0.01f);
	// ★ Level 1 扭矩级角速度阻尼：直接抵抗相机横滚转动，吸收振荡能量。
	//   ★ 符号已翻转：横滚外环 ROLL_GAIN=-1 与俯仰 PITCH_GAIN=+1 符号相反，
	//     陀螺速率对横滚轴的符号关系也相反，所以阻尼项必须用 `Uq +=`（否则正反馈锁死）。
	//   再保险：若翻转后横滚改为"跑飞/越转越快"，说明原 `Uq -=` 才是对的，改回去即可。
	Uq += M0_RATE_DAMP * (g_IMU_RollRate * DEG_TO_RAD);
	Uq *= g_StartRamp;							// ★ 软启动斜坡：0→1 递增限幅
	if (Uq > -UQ_DEADBAND && Uq < UQ_DEADBAND) Uq = 0.0f;
	SetPhaseVoltage(&M0, Uq, M0_electricAngle());
}

// ★ M1（俯仰）角度环
void M1_Set_Angle(float Target)
{
	float angle = GetAngle(&Angle_Sensor1);
	M1.Enc_Angle = angle;
	// ★ 防缠线：同上（相对上电参考零点 + 最短路径）
	float err = AngleErr(Target - M1_DIR * (angle - M1_angle_ref));
	float Uq = PID_Controller(&M1_ANG_PID, err, 0.01f);
	// ★ Level 1 扭矩级角速度阻尼：直接抵抗相机俯仰转动，吸收振荡能量。
	//   符号判据：拨动俯仰若更快收敛→对；若更凶/发散→把 `Uq -=` 改成 `Uq +=`
	Uq -= M1_RATE_DAMP * (g_IMU_PitchRate * DEG_TO_RAD);
	Uq *= g_StartRamp;							// ★ 软启动斜坡：0→1 递增限幅
	if (Uq > -UQ_DEADBAND && Uq < UQ_DEADBAND) Uq = 0.0f;
	SetPhaseVoltage(&M1, Uq, M1_electricAngle());
}

// M0 速度环（Target 单位 rad/s，当前自稳未使用）
void M0_Set_Velocity(float Target)
{
	float velocity = GetVelocity(&Angle_Sensor0);
	float Uq = PID_Controller(&M0_VEL_PID, Target - velocity, 0.01f);
	Uq *= g_StartRamp;							// 软启动斜坡
	if (Uq > -UQ_DEADBAND && Uq < UQ_DEADBAND) Uq = 0.0f;
	SetPhaseVoltage(&M0, Uq, M0_electricAngle());
}

void M1_Set_Velocity(float Target)
{
	float velocity = GetVelocity(&Angle_Sensor1);
	float Uq = PID_Controller(&M1_VEL_PID, Target - velocity, 0.01f);
	Uq *= g_StartRamp;							// 软启动斜坡
	if (Uq > -UQ_DEADBAND && Uq < UQ_DEADBAND) Uq = 0.0f;
	SetPhaseVoltage(&M1, Uq, M1_electricAngle());
}

// 开环力矩（Target 即 Uq，单位 V）
void M0_Set_CurTorque(float Target)
{
	SetPhaseVoltage(&M0, Target, M0_electricAngle());
}

void M1_Set_CurTorque(float Target)
{
	SetPhaseVoltage(&M1, Target, M1_electricAngle());
}

// ★ 软启动：进入自稳前调用，清零斜坡（对齐/锁位阶段保持满输出）
void FOC_SoftStart_Begin(void)
{
	g_StartRamp = 0.0f;
}

// ★ 软启动：每个控制周期推进斜坡（0→1）
void FOC_SoftStart_Tick(void)
{
	if (g_StartRamp < 1.0f)
	{
		g_StartRamp += 1.0f / START_RAMP_CYCLES;
		if (g_StartRamp > 1.0f) g_StartRamp = 1.0f;
	}
}

// ★ 诊断：读取软启动斜坡系数（0~1），串口确认斜坡是否爬满
float FOC_GetStartRamp(void)
{
	return g_StartRamp;
}

// ★ 直接切断电机驱动（可重复调用，不重复上报）
void FOC_CutPower(void)
{
	GPIO_ResetBits(GPIOA, GPIO_Pin_8);   // 拉低驱动使能，切断三相
	M0_PWM_A(0); M0_PWM_B(0); M0_PWM_C(0);
	M1_PWM_A(0); M1_PWM_B(0); M1_PWM_C(0);
}

// ★ 故障停机：锁存 + 通知主循环上报一次 + 切断驱动（需重新上电恢复）
void FOC_Disable(void)
{
	FOC_Fault_Flag   = 1;
	FOC_Fault_Notify = 1;
	FOC_CutPower();
}

// ★ 电机上电初始化（防缠线）：把当前转子位置捕获为自稳零点并原地保持。
//   不再强制转到编码器绝对 0（避免上电时转子大角度回摆、多圈旋转缠线）。
void tripod_Iint(void)
{
	uint16_t i;
	float err0, err1;

	M0_angle_ref = GetAngle(&Angle_Sensor0);   // 当前多圈角作为 M0 零点
	M1_angle_ref = GetAngle(&Angle_Sensor1);   // 当前多圈角作为 M1 零点

	for (i = 0; i < 300; i ++)          // max ~3s (300 x 10ms)
	{
		M0_Set_Angle(0.0f);             // 目标 0（相对参考零点）= 原地保持
		M1_Set_Angle(0.0f);
		delay_ms(10);

		// 收敛判据：转子未漂移（相对参考零点误差 < 0.05 rad ≈ 2.9°）
		err0 = fabsf(AngleErr(-M0_DIR * (GetAngle(&Angle_Sensor0) - M0_angle_ref)));
		err1 = fabsf(AngleErr(-M1_DIR * (GetAngle(&Angle_Sensor1) - M1_angle_ref)));
		if (err0 < 0.05f && err1 < 0.05f)
			break;
	}
	M0_Set_Angle(0.0f);
	M1_Set_Angle(0.0f);
}
