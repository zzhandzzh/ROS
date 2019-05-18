#include "motor.h"

void MiniBalance_Motor_Init(void)
{		
		/*����һ��GPIO_InitTypeDef���͵Ľṹ��*/
	GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF,ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
  GPIO_Init(GPIOF, &GPIO_InitStructure); 
	
	RCC->APB2ENR|=1<<4;          //PORTCʱ��ʹ��     
	GPIOC->CRH&=0XFFFFFF00;  //PORTC8�������
	GPIOC->CRH|=0X000000BB;    //PORTC8�������
	
	GPIOC->CRL&=0X00FFFFFF;  //PORTC6 7�������
	GPIOC->CRL|=0XBB000000;    //PORTC6 7�������
}
void MiniBalance_PWM_Init(u16 arr,u16 psc)
{		 			
	GPIO_InitTypeDef GPIO_InitStructure;
	MiniBalance_Motor_Init();  //��ʼ�������������IO
	RCC->APB2ENR|=1<<13;       //ʹ��TIM8ʱ��   

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);// 
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE , ENABLE);  
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);                                                                    	
  GPIO_PinRemapConfig(GPIO_FullRemap_TIM1,ENABLE);                                                                  	  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_11|GPIO_Pin_13|GPIO_Pin_14; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	TIM8->ARR=arr;             //�趨�������Զ���װֵ 
	TIM8->PSC=psc;             //Ԥ��Ƶ������Ƶ
	TIM8->CCMR1|=6<<4;         //CH1 PWM1ģʽ	
	TIM8->CCMR1|=6<<12;        //CH2 PWM1ģʽ	
	TIM8->CCMR2|=6<<4;         //CH3 PWM1ģʽ	
	TIM8->CCMR2|=6<<12;        //CH4 PWM1ģʽ	
	
	TIM8->CCMR1|=1<<3;         //CH1Ԥװ��ʹ��	  
	TIM8->CCMR1|=1<<11;        //CH2Ԥװ��ʹ��	 
	TIM8->CCMR2|=1<<3;         //CH3Ԥװ��ʹ��	  
	TIM8->CCMR2|=1<<11;        //CH4Ԥװ��ʹ��	  
	
	TIM8->CCER|=1<<0;         //CH1���ʹ��	
	TIM8->CCER|=1<<4;         //CH2���ʹ��	   
	TIM8->CCER|=1<<8;         //CH3���ʹ��	 
	TIM8->CCER|=1<<12;        //CH4���ʹ��
	TIM8->BDTR |= 1<<15;       //TIM����Ҫ��仰�������PWM
	TIM8->CR1=0x8000;          //ARPEʹ�� 
	TIM8->CR1|=0x01;          //ʹ�ܶ�ʱ�� 			
} 

