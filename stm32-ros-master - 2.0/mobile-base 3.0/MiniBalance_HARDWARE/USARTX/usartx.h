#ifndef __USRAT3_H
#define __USRAT3_H 
#include "sys.h"	  	

extern u8 Usart2_Receive;
void uart2_init(u32 bound);
void USART2_IRQHandler(void);
#endif

