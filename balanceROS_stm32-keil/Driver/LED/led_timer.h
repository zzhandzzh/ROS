#ifdef __cplusplus
extern "C" {
#endif
#ifndef __LED_H
#define __LED_H	
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ���������ɣ��������������κ���;
//ALIENTEK��ӢSTM32F103������V1
//LED��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2015/1/9
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) �������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 

//LED�˿ڶ���
#define LED0 PBout(5)	// DS0
#define LED1 PEout(5)	// DS1	

void LED_Init(void);	//��ʼ��		 				    
#endif


#ifdef __cplusplus
}
#endif













