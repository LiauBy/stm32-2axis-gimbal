#ifndef  __INLINECURRENT_H
#define  __INLINECURRENT_H


struct Current_Sensor{
	int Sen_Num;
	float I_a;
	float I_b;
	float offset_ia;
	float offset_ib;
	float current_a;
	float current_b;
	float current_c;
};

void AD_Init(void);
void Set_Cur_Sensor(int Mot_num);
void DriftOffsets(struct Current_Sensor *Sensor);
void CurrSense_Init(struct Current_Sensor *Sensor);
void GetPhaseCurrent(struct Current_Sensor *Sensor);


#endif

