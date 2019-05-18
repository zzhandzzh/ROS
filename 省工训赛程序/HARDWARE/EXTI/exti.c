#include "exti.h"
#include "key.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK精英STM32开发板
//外部中断 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2012/9/3
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////   
//外部中断0服务程序
void EXTIX_Init(void)
{
 	RCC->APB2ENR|=1<<6;     //使能PORTE时钟
	RCC->APB2ENR|=1<<0;     //开启辅助时钟	
//	AFIO->MAPR&=0XF8FFFFFF; //清除MAPR的[26:24]
//	AFIO->MAPR|=0X04000000; //关闭JTAG
	GPIOE->CRL&=0XFFFFFFF0;//PE0设置成输入	  
	GPIOE->CRL|=0X00000008; 	
	GPIOE->ODR|=1<<0;
 	Ex_NVIC_Config(4,0,1);//下降沿触发
	MY_NVIC_Init(2,1,EXTI0_IRQn,2);//抢占2，子优先级1，组2	   
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

	EXTI_ClearITPendingBit(EXTI_Line0);  //清除LINE3上的中断标志位  
}
