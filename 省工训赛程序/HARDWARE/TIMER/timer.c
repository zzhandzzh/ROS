#include "timer.h"
#include "led.h"
//基本定时器TIM6配置
//溢出时间t=(7200*500)/72000000s=5ms


void TIM6_init(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE); 
	TIM_TimeBaseStructure.TIM_Period = 50-1;  
	TIM_TimeBaseStructure.TIM_Prescaler = 7200-1;   
	TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;  
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);
	TIM_ITConfig(TIM6,TIM_IT_Update, ENABLE);     
	TIM_Cmd(TIM6, ENABLE); 
	MY_NVIC_Init(2,1,TIM6_IRQn,2);  	//抢占2，子优先级1，组2	
}









