#include "motor.h"

void MiniBalance_Motor_Init(void)
{		
		/*定义一个GPIO_InitTypeDef类型的结构体*/
	GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF,ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
  GPIO_Init(GPIOF, &GPIO_InitStructure); 
	
	RCC->APB2ENR|=1<<4;          //PORTC时钟使能     
	GPIOC->CRH&=0XFFFFFF00;  //PORTC8复用输出
	GPIOC->CRH|=0X000000BB;    //PORTC8复用输出
	
	GPIOC->CRL&=0X00FFFFFF;  //PORTC6 7复用输出
	GPIOC->CRL|=0XBB000000;    //PORTC6 7复用输出
}
void MiniBalance_PWM_Init(u16 arr,u16 psc)
{		 			
	GPIO_InitTypeDef GPIO_InitStructure;
	MiniBalance_Motor_Init();  //初始化电机控制所需IO
	RCC->APB2ENR|=1<<13;       //使能TIM8时钟   

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);// 
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE , ENABLE);  
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);                                                                    	
  GPIO_PinRemapConfig(GPIO_FullRemap_TIM1,ENABLE);                                                                  	  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_11|GPIO_Pin_13|GPIO_Pin_14; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	TIM8->ARR=arr;             //设定计数器自动重装值 
	TIM8->PSC=psc;             //预分频器不分频
	TIM8->CCMR1|=6<<4;         //CH1 PWM1模式	
	TIM8->CCMR1|=6<<12;        //CH2 PWM1模式	
	TIM8->CCMR2|=6<<4;         //CH3 PWM1模式	
	TIM8->CCMR2|=6<<12;        //CH4 PWM1模式	
	
	TIM8->CCMR1|=1<<3;         //CH1预装载使能	  
	TIM8->CCMR1|=1<<11;        //CH2预装载使能	 
	TIM8->CCMR2|=1<<3;         //CH3预装载使能	  
	TIM8->CCMR2|=1<<11;        //CH4预装载使能	  
	
	TIM8->CCER|=1<<0;         //CH1输出使能	
	TIM8->CCER|=1<<4;         //CH2输出使能	   
	TIM8->CCER|=1<<8;         //CH3输出使能	 
	TIM8->CCER|=1<<12;        //CH4输出使能
	TIM8->BDTR |= 1<<15;       //TIM必须要这句话才能输出PWM
	TIM8->CR1=0x8000;          //ARPE使能 
	TIM8->CR1|=0x01;          //使能定时器 			
} 

