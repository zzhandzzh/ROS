#include "exti.h"
#include "key.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK��ӢSTM32������
//�ⲿ�ж� ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/9/3
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////   
//�ⲿ�ж�0�������
void EXTIX_Init(void)
{
 	RCC->APB2ENR|=1<<6;     //ʹ��PORTEʱ��
	RCC->APB2ENR|=1<<0;     //��������ʱ��	
//	AFIO->MAPR&=0XF8FFFFFF; //���MAPR��[26:24]
//	AFIO->MAPR|=0X04000000; //�ر�JTAG
	GPIOE->CRL&=0XFFFFFFF0;//PE0���ó�����	  
	GPIOE->CRL|=0X00000008; 	
	GPIOE->ODR|=1<<0;
 	Ex_NVIC_Config(4,0,1);//�½��ش���
	MY_NVIC_Init(2,1,EXTI0_IRQn,2);//��ռ2�������ȼ�1����2	   
}


void EXTI0_IRQHandler(void)
{
		if(Sensor==1)
	{
		xx++;
	}
			if(Sensor==2)
	{
		yy++;	//printf("{#%d}$",yy);
	}

	EXTI_ClearITPendingBit(EXTI_Line0);  //���LINE3�ϵ��жϱ�־λ  
}
