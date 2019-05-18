#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h" 

#define USART_REC_LEN  			200  	//定义最大接收字节数 200
#define EN_USART1_RX 			1		//使能（1）/禁止（0）串口1接收
	  	
extern u8  USART_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern u16 USART_RX_STA;         		//接收状态标记	
#define		DINT()		__disable_irq()
#define		EINT()		__enable_irq()
#define USARTzTxBufferSize   64
#define USARTz										USART1
#define USARTz_GPIO								GPIOA
#define USARTz_RxPin							GPIO_Pin_10
#define USARTz_TxPin							GPIO_Pin_9
#define USARTz_IRQn								USART1_IRQn
#define USARTz_DR_Base						(uint32_t)(&USART1->DR)
//#define USARTz_DR_Base						USART3_BASE+0x04
#define USARTz_Tx_DMA_Channe			DMA2_Stream7
#define USARTz_Tx_DMA_FLAG				DMA1_FLAG_GL4 //DMA1_FLAG_TC2|DMA1_FLAG_TE2
#define UASRTz_TX_DMA_IRQ					DMA1_Channel4_IRQn

#define USARTz_Rx_DMA_Channe			DMA2_Stream5
#define USARTz_Rx_DMA_FLAG				DMA1_FLAG_GL5 //DMA1_FLAG_TC3 |DMA1_FLAG_TE3  //前者为DMA1通道3全局标志位，后者为传输完成标志位和传输错误标志位
#define UASRTz_RX_DMA_IRQ					DMA1_Channel5_IRQn

//浮点数与HEX快速获取
typedef	union{
		float 	fv;
		uint8_t cv[4];
}float_union;

//接收数据结构
typedef	struct{

		float_union		linear_vx;//线速度x
		float_union		linear_vy;//线速度y
		float_union		angular_v;//角速度
		
}rcv_data;

//发送数据结构
typedef	struct{
		
		float_union	x_pos;//x方向坐标
		float_union	y_pos;//y方向坐标
		float_union	x_v;//x方向速度
		float_union	y_v;//y方向速度
		float_union	angular_v;//角速度
		float_union	pose_angular;//角度
	
}send_data;

void com_x_usart_dma_read(void);
void com_x_usart_dma_start_tx(uint8_t size);
void DMA_Use_USART1_Rx_Init(void);
void DMA_Use_USART1_Tx_Init(void);
void uart_init(u32 bound);
extern send_data	com_x_send_data;//数据发送
extern rcv_data		com_x_rcv_data;//数据接收
				
//****************外部调用函数************************
int8_t get_data_analyze(uint8_t	*pdata);//接收数据分析
void data_pack(void);//数据打包并发送

#endif


