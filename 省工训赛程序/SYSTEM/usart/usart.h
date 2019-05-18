#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "sys.h" 
int fputc(int ch,FILE *p);  
void uart1_init(u32 bt);  
void uart2_init(u32 bt);  
void uart3_init(u32 bt);  
void Usart_SendByte( USART_TypeDef * pUSARTx, uint8_t ch);
void Usart_SendString( USART_TypeDef * pUSARTx, char *str);
void Usart_SendHalfWord( USART_TypeDef * pUSARTx, uint16_t ch);
void uart_init(u32 pclk2,u32 bound);
void USART2_Init(u32 My_BaudRate);
void uartWriteBuf(uint8_t *buf, uint8_t len);
#endif /* __USART_H */



