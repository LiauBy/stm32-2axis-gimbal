#include "stm32f10x.h"

/**
  * @brief  微秒级延时
  * @param  xus 延时时长，范围：0~233015
  * @retval 无
  */
void Delay_us(uint32_t xus)
{
	SysTick->LOAD = 72 * xus;				//设置定时器重装值
	SysTick->VAL = 0x00;					//清空当前计数值
	SysTick->CTRL = 0x00000005;				//设置时钟源为HCLK，启动定时器
	while(!(SysTick->CTRL & 0x00010000));	//等待计数到0
	SysTick->CTRL = 0x00000004;				//关闭定时器
}

/**
  * @brief  毫秒级延时
  * @param  xms 延时时长，范围：0~4294967295
  * @retval 无
  */
void Delay_ms(uint32_t xms)
{
	while(xms--)
	{
		Delay_us(1000);
	}
}
 
/**
  * @brief  秒级延时
  * @param  xs 延时时长，范围：0~4294967295
  * @retval 无
  */
void Delay_s(uint32_t xs)
{
	while(xs--)
	{
		Delay_ms(1000);
	}
} 
#include "stm32f10x.h"

void Systick_CountMode(void)
{
	SysTick->LOAD = 0xFFFFFF-1;
	SysTick->VAL  = 0;
	SysTick->CTRL = SysTick_CTRL_ENABLE_Msk;
}


/*******非嘀嗒定时器延时********/
void delay_us(uint16_t time)
{    
   uint16_t i=0;  
   while(time--)
   {
      i=10;
      while(i--) ;    
   }
}
//毫秒级的延时
void delay_ms(uint16_t time)
{    
   uint16_t i=0;  
   while(time--)
   {
      i=12000;
      while(i--) ;    
   }
}

