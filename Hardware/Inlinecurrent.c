#include "stm32f10x.h"
#include "Serial.h"
#include "Inlinecurrent.h"

uint16_t Samp_volts[4];

void AD_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	DMA_InitTypeDef DMA_InitStruct;
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);
	
	GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 1, ADC_SampleTime_7Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 2, ADC_SampleTime_7Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 3, ADC_SampleTime_7Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 4, ADC_SampleTime_7Cycles5);
	
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_NbrOfChannel = 4;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_Init(ADC1,&ADC_InitStructure);
	
	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)Samp_volts;
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStruct.DMA_BufferSize = 4;
	DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;
	DMA_InitStruct.DMA_Priority = DMA_Priority_Medium;
	DMA_Init(DMA1_Channel1,&DMA_InitStruct);
	
	DMA_Cmd(DMA1_Channel1,ENABLE);
	ADC_DMACmd(ADC1, ENABLE);
	ADC_Cmd(ADC1, ENABLE);
	
	ADC_ResetCalibration(ADC1);
  while (ADC_GetResetCalibrationStatus(ADC1) == SET);
  ADC_StartCalibration(ADC1);
  while (ADC_GetCalibrationStatus(ADC1) == SET);
	
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

#define _ADC_CONV    0.00080586f

float _shunt_resistor = 0;
float amp_gain = 0;
float vlots_to_amps = 0;
float gain_a, gain_b, gain_c;

int A, B;
void Set_Cur_Sensor(int Mot_num)
{
	if(Mot_num == 0)
	{
		A = 0;
		B = 1;
	}
	else if(Mot_num == 1)
	{
		A = 2;
		B = 3;
	}
}
	
// ¡„∆ÆºÏ≤‚
void DriftOffsets(struct Current_Sensor *Sensor)
{
	uint16_t detect_rounds = 1000;
	for(int i = 0; i < detect_rounds; i++)
	{
		Sensor->offset_ia += Samp_volts[A]*_ADC_CONV;
		Sensor->offset_ib += Samp_volts[B]*_ADC_CONV;
	}
	
	Sensor->offset_ia = Sensor->offset_ia / detect_rounds;
	Sensor->offset_ib = Sensor->offset_ib / detect_rounds;

}

void CurrSense_Init(struct Current_Sensor *Sensor)
{
	Set_Cur_Sensor(Sensor->Sen_Num);
	DriftOffsets(Sensor);
	
	_shunt_resistor = 0.01; 
	amp_gain = 50; 
	
	vlots_to_amps = 1.0f / _shunt_resistor / amp_gain;
	
	gain_a = vlots_to_amps * -1;
	gain_b = vlots_to_amps * -1;
	gain_c = vlots_to_amps;
	
}

void GetPhaseCurrent(struct Current_Sensor *Sensor)
{
//  struct Current_Sensor current;
	Set_Cur_Sensor(Sensor->Sen_Num);
	float tran_vol_a = (float)Samp_volts[A]*_ADC_CONV;
	float tran_vol_b = (float)Samp_volts[B]*_ADC_CONV;
	
	Sensor->I_a = (tran_vol_a - Sensor->offset_ia)*gain_a;
	Sensor->I_b = (tran_vol_b - Sensor->offset_ib)*gain_b;
}



