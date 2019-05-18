#include "sensor.h"	

void OPT_GPIO_Config(void)
{		
	/*定义一个GPIO_InitTypeDef类型的结构体*/
//	GPIO_InitTypeDef GPIO_InitStructure;
//  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE);
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz; 
//  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;	
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//  GPIO_Init(GPIOG, &GPIO_InitStructure); 
	GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOB, &GPIO_InitStructure); 
}
void DUOJICHUSHIHUA(void)
{		
	/*定义一个GPIO_InitTypeDef类型的结构体*/
//	GPIO_InitTypeDef GPIO_InitStructure;
//  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE);
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz; 
//  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;	
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//  GPIO_Init(GPIOG, &GPIO_InitStructure); 
	GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOE, &GPIO_InitStructure); 
	GPIO_SetBits(GPIOE, GPIO_Pin_12);
	GPIO_ResetBits(GPIOE, GPIO_Pin_13);
}
void READ_SENSOR(int sen)
{
	switch(sen)
	{
		case 1:	LEFT=GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12);break;	
		case 2:RIGHT=GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13);break;	
		case 3:LEFT1=GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14);break;	
		case 4:RIGHT1=GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15);break;	
		default:break;
	}
}

