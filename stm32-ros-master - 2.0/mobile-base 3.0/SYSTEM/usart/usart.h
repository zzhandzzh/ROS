#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h" 

#define USART_REC_LEN  			200  	//�����������ֽ��� 200
#define EN_USART1_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����
	  	
extern u8  USART_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u16 USART_RX_STA;         		//����״̬���	
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
#define USARTz_Rx_DMA_FLAG				DMA1_FLAG_GL5 //DMA1_FLAG_TC3 |DMA1_FLAG_TE3  //ǰ��ΪDMA1ͨ��3ȫ�ֱ�־λ������Ϊ������ɱ�־λ�ʹ�������־λ
#define UASRTz_RX_DMA_IRQ					DMA1_Channel5_IRQn

//��������HEX���ٻ�ȡ
typedef	union{
		float 	fv;
		uint8_t cv[4];
}float_union;

//�������ݽṹ
typedef	struct{

		float_union		linear_vx;//���ٶ�x
		float_union		linear_vy;//���ٶ�y
		float_union		angular_v;//���ٶ�
		
}rcv_data;

//�������ݽṹ
typedef	struct{
		
		float_union	x_pos;//x��������
		float_union	y_pos;//y��������
		float_union	x_v;//x�����ٶ�
		float_union	y_v;//y�����ٶ�
		float_union	angular_v;//���ٶ�
		float_union	pose_angular;//�Ƕ�
	
}send_data;

void com_x_usart_dma_read(void);
void com_x_usart_dma_start_tx(uint8_t size);
void DMA_Use_USART1_Rx_Init(void);
void DMA_Use_USART1_Tx_Init(void);
void uart_init(u32 bound);
extern send_data	com_x_send_data;//���ݷ���
extern rcv_data		com_x_rcv_data;//���ݽ���
				
//****************�ⲿ���ú���************************
int8_t get_data_analyze(uint8_t	*pdata);//�������ݷ���
void data_pack(void);//���ݴ��������

#endif


