#include "sys.h"
#include "usart.h"	
////////////////////////////////////////////////////////////////////////////////// 	 
//���ʹ��ucos,����������ͷ�ļ�����.
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos ʹ��	  
#endif


//////////////////////////////////////////////////////////////////
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
_sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{ 	
	if(Flag_Show==0)
	{	
	while((USART2->SR&0X40)==0);//Flag_Show=0  ʹ�ô���2
	USART2->DR = (u8) ch;      
	}
	/* ����1 ���Σ�������ݮ��ͨ��
	else
	{	
	while((USART1->SR&0X40)==0);//Flag_Show!=0  ʹ�ô���1   
	USART1->DR = (u8) ch;      
	}	
	*/
	return ch;
}
#endif
 
uint8_t USARTzTxBuffer[USARTzTxBufferSize];
uint8_t USARTzRxBuffer[USARTzTxBufferSize];
uint8_t USARTzRxBufferD[USARTzTxBufferSize];

send_data	com_x_send_data;//���ݷ���
rcv_data		com_x_rcv_data;//���ݽ���


//����1�жϷ������//�ĳɴ��ڼ�DMA��ȡ27�ֽ�һ֡������
//bound:������
void uart_init(u32 bound){
   //GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //ʹ��GPIOAʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//ʹ��USART1ʱ��
 
	//����1��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9����ΪUSART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10����ΪUSART1
	
	//USART1�˿�����
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9��GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOA,&GPIO_InitStructure); //��ʼ��PA9��PA10

   //USART1 ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(USART1, &USART_InitStructure); //��ʼ������1

	/* Enable USART1 Receive  interrupts */
  USART_ITConfig(USARTz, USART_IT_IDLE, ENABLE); //�������ڿ���IDLE�ж�
  /* Enable USARTz DMA TX request */
  USART_DMACmd(USARTz, USART_DMAReq_Tx, ENABLE);
	/* Enable USARTz DMA RX request */
	USART_DMACmd(USARTz, USART_DMAReq_Rx, ENABLE);
  /* Enable USARTz */
  USART_Cmd(USARTz, ENABLE);
  /* Enable USARTz DMA TX Channel */
	USART_DMACmd(USARTz, USART_DMAReq_Tx,ENABLE);// DMA_Cmd(USARTz_Tx_DMA_Channe, ENABLE);
	/* Enable USARTz DMA RX Channel */
	USART_DMACmd(USARTz, USART_DMAReq_Rx,ENABLE);//DMA_Cmd(USARTz_Rx_DMA_Channe, ENABLE);
	
	/* Enable the DMA Interrupt */
//	NVIC_InitStructure.NVIC_IRQChannel = (uint8_t)UASRTz_TX_DMA_IRQ;  
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;     
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
	/* Enable the USARTz Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USARTz_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
void DMA_Use_USART1_Tx_Init(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

    DMA_DeInit(DMA2_Stream7); 

    DMA_InitStructure.DMA_Channel             = DMA_Channel_4;
    DMA_InitStructure.DMA_PeripheralBaseAddr  = (uint32_t)(&(USART1->DR));
    DMA_InitStructure.DMA_Memory0BaseAddr     = (uint32_t)USARTzTxBuffer;
    DMA_InitStructure.DMA_DIR                 = DMA_DIR_MemoryToPeripheral;
    DMA_InitStructure.DMA_BufferSize          = USARTzTxBufferSize;
    DMA_InitStructure.DMA_PeripheralInc       = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc           = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize  = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize      = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode                = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority            = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_FIFOMode            = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold       = DMA_FIFOThreshold_HalfFull;
    DMA_InitStructure.DMA_MemoryBurst         = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst     = DMA_PeripheralBurst_Single;
 
    DMA_Init(DMA2_Stream7, &DMA_InitStructure);		//USART1����ӳ��DMA2Channel4������7	//����DMA
    DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, ENABLE);//ʹ��DMA�ж�
    USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);//ʹ�ܴ��ڵ�DMA���ͽӿ�

  
    NVIC_InitStructure.NVIC_IRQChannel                   = DMA2_Stream7_IRQn;           
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;          
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1; 
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

		//DMA_ClearFlag(DMA2_Stream7,USARTz_Tx_DMA_FLAG);
    DMA_Cmd(DMA2_Stream7, ENABLE);
}

void DMA_Use_USART1_Rx_Init(void)
{
    DMA_InitTypeDef DMA_InitStructure;
    //tUART1_Rx.dwUART1RxLen = 0;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

    DMA_DeInit(DMA2_Stream5); 

    DMA_InitStructure.DMA_Channel             = DMA_Channel_4;              
    DMA_InitStructure.DMA_PeripheralBaseAddr  = (uint32_t)(&(USART1->DR));  
    DMA_InitStructure.DMA_Memory0BaseAddr     = (uint32_t)USARTzRxBuffer;            
    DMA_InitStructure.DMA_DIR                 = DMA_DIR_PeripheralToMemory;    
    DMA_InitStructure.DMA_BufferSize          = USARTzTxBufferSize;                                  
    DMA_InitStructure.DMA_PeripheralInc       = DMA_PeripheralInc_Disable;    
    DMA_InitStructure.DMA_MemoryInc           = DMA_MemoryInc_Enable;        
    DMA_InitStructure.DMA_PeripheralDataSize  = DMA_MemoryDataSize_Byte;      
    DMA_InitStructure.DMA_MemoryDataSize      = DMA_MemoryDataSize_Byte;     
    DMA_InitStructure.DMA_Mode                = DMA_Mode_Normal;              
    DMA_InitStructure.DMA_Priority            = DMA_Priority_High;       
    DMA_InitStructure.DMA_FIFOMode            = DMA_FIFOMode_Disable;         
    DMA_InitStructure.DMA_FIFOThreshold       = DMA_FIFOThreshold_HalfFull; 
    DMA_InitStructure.DMA_MemoryBurst         = DMA_MemoryBurst_Single;       
    DMA_InitStructure.DMA_PeripheralBurst     = DMA_PeripheralBurst_Single;

    DMA_Init(DMA2_Stream5, &DMA_InitStructure);
		//���ղ���ҪDMA�ж�
    USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);
		DMA_Cmd(DMA2_Stream5,ENABLE);
}


//************************************DMA����********************************
void DMA2_Stream7_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA2_Stream7,DMA_IT_TCIF7))
    {
      DMA_ClearFlag(DMA2_Stream7,DMA_IT_TCIF7|DMA_IT_TEIF7);    //���DMA���б�־    
			DMA_Cmd(DMA2_Stream7, DISABLE);  //�ر�DMA����ͨ��
      //USART_ITConfig(USART1,USART_IT_TC,ENABLE);
    }
}

void com_x_usart_dma_start_tx(uint8_t size)
{
    USARTz_Tx_DMA_Channe->NDTR = (uint16_t)size; //���¸�ֵ ָ�����ͻ��泤��
    DMA_Cmd(DMA2_Stream7, ENABLE);  //����DMA����      
}


void USART1_IRQHandler(void)                	//����1�жϷ������
{
  
	if(USART_GetITStatus(USARTz, USART_IT_IDLE) != RESET)  
    {
        com_x_usart_dma_read();
				USART_ReceiveData( USARTz );
    }
} 

/*�жϽ���*/
//void USART1_IRQFuc(void)   
//{  

//    deal_irq_tx_end();   

//    tUART1_Rx.dwUART1RxLen = deal_irq_rx_end(tUART1_Rx.UART1RxBuf); 
//} 
/*�����жϽ���*/
//static void deal_irq_tx_end(void)  
//{  
//    if(USART_GetITStatus(USART1, USART_IT_TXE) == RESET)  
//    {  
//        
//        USART_ITConfig(USART1,USART_IT_TC,DISABLE);  
//       
//        UART1_Use_DMA_Tx_Flag = 0;  
//    }    
//} 

//static uint8_t deal_irq_rx_end(uint8_t *buf)  
//{     
//    uint16_t len = 0;  

//    if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)  
//    {  
//        USART1->SR;  
//        USART1->DR; 

//        DMA_Cmd(DMA2_Stream2,DISABLE);  

//        DMA_ClearFlag(DMA2_Stream2,DMA_FLAG_TCIF2);  


//        len = RX_BUF_LEN - DMA_GetCurrDataCounter(DMA2_Stream2);  
//        memcpy(buf,Rx_Buf,len);  


//        DMA_SetCurrDataCounter(DMA2_Stream2,RX_BUF_LEN);  

//        DMA_Cmd(DMA2_Stream2,ENABLE);  

//        return len;  
//    }   

//    return 0;  
//}  

void com_x_usart_dma_read(void)
{
    uint8_t rxcounter;
		uint8_t i;
		DMA_Cmd(DMA2_Stream5, DISABLE);    //�ر�DMA��ֹ����   
    DMA_ClearFlag(DMA2_Stream5,DMA_IT_TCIF5|DMA_IT_TEIF5);   //�����־λ       
    rxcounter= USARTzTxBufferSize - DMA_GetCurrDataCounter(DMA2_Stream5);//��ȡ���յ����ֽ��� 
    DMA2_Stream5->NDTR = USARTzTxBufferSize; //���¸�ֵ����ֵ   
    memset(USARTzRxBufferD,0,sizeof(USARTzRxBufferD));
		printf("Received data:");
	  for(i=0;i<rxcounter;i++){
			USARTzRxBufferD[i] = USARTzRxBuffer[i];//��ȡ���յ������ݣ���������RxBufferD��
			printf("0x%02x",USARTzRxBufferD[i]);
		}
		printf("\n\r");
		//�����յ������ݰ�
		//DINT();//��ֹ�ж�
		if(get_data_analyze(USARTzRxBufferD) != 0)
			printf("data analyze error\n\r");
		//EINT();//�����ж�
		for(i=0;i<rxcounter;i++)
			USARTzRxBuffer[i] = 0;//clear Rx buffer
		DMA_Cmd(DMA2_Stream5, ENABLE);  //DMA���� �ȴ���һ֡����     

}
//void com_x_usart_rcc_config(void)
//{
///* DMA clock enable */
//  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
//	
//  RCC_APB2PeriphClockCmd( USARTz_GPIO_CLK  |RCC_APB2Periph_AFIO, ENABLE);
//	RCC_APB2PeriphClockCmd(USARTz_CLK,ENABLE);

//}
//void com_x_usart_Init(void)
//{
//	
//	com_x_usart_rcc_config();
//	com_x_uasrt_nvic_config();
//	com_x_usart_gpio_config();
//	com_x_usart_dma_config();
//	com_x_usart_config();
//}
//����
void data_pack(void)
{
	
	com_x_send_data.x_pos.fv = 2.68;//x����
	com_x_send_data.y_pos.fv = 3.96;//y����
	com_x_send_data.x_v.fv	= 0.6;//x�����ٶ�
	com_x_send_data.y_v.fv = 0.0;//y �����ٶ�
	com_x_send_data.angular_v.fv = 2.0;//���ٶ� ��z��
	com_x_send_data.pose_angular.fv = 1.0;//yawƫ����
	
	USARTzTxBuffer[0] = 0xaa;
	USARTzTxBuffer[1] = 0xaa;
	
	USARTzTxBuffer[2] = com_x_send_data.x_pos.cv[0];
	USARTzTxBuffer[3] = com_x_send_data.x_pos.cv[1];
	USARTzTxBuffer[4] = com_x_send_data.x_pos.cv[2];
	USARTzTxBuffer[5] = com_x_send_data.x_pos.cv[3];
	
	USARTzTxBuffer[6] = com_x_send_data.y_pos.cv[0];
	USARTzTxBuffer[7] = com_x_send_data.y_pos.cv[1];
	USARTzTxBuffer[8] = com_x_send_data.y_pos.cv[2];
	USARTzTxBuffer[9] = com_x_send_data.y_pos.cv[3];
	
	USARTzTxBuffer[10] = com_x_send_data.x_v.cv[0];
	USARTzTxBuffer[11] = com_x_send_data.x_v.cv[1];
	USARTzTxBuffer[12] = com_x_send_data.x_v.cv[2];
	USARTzTxBuffer[13] = com_x_send_data.x_v.cv[3];
	
	USARTzTxBuffer[14] = com_x_send_data.y_v.cv[0];
	USARTzTxBuffer[15] = com_x_send_data.y_v.cv[1];
	USARTzTxBuffer[16] = com_x_send_data.y_v.cv[2];
	USARTzTxBuffer[17] = com_x_send_data.y_v.cv[3];
	
	USARTzTxBuffer[18] = com_x_send_data.angular_v.cv[0];
	USARTzTxBuffer[19] = com_x_send_data.angular_v.cv[1];
	USARTzTxBuffer[20] = com_x_send_data.angular_v.cv[2];
	USARTzTxBuffer[21] = com_x_send_data.angular_v.cv[3];
	
	USARTzTxBuffer[22] = com_x_send_data.pose_angular.cv[0];
	USARTzTxBuffer[23] = com_x_send_data.pose_angular.cv[1];
	USARTzTxBuffer[24] = com_x_send_data.pose_angular.cv[2];
	USARTzTxBuffer[25] = com_x_send_data.pose_angular.cv[3];
	
	USARTzTxBuffer[26] = 	USARTzTxBuffer[2]^USARTzTxBuffer[3]^USARTzTxBuffer[4]^USARTzTxBuffer[5]^USARTzTxBuffer[6]^
												USARTzTxBuffer[7]^USARTzTxBuffer[8]^USARTzTxBuffer[9]^USARTzTxBuffer[10]^USARTzTxBuffer[11]^
												USARTzTxBuffer[12]^USARTzTxBuffer[13]^USARTzTxBuffer[14]^USARTzTxBuffer[15]^USARTzTxBuffer[16]^
												USARTzTxBuffer[17]^USARTzTxBuffer[18]^USARTzTxBuffer[19]^USARTzTxBuffer[20]^USARTzTxBuffer[21]^
												USARTzTxBuffer[22]^USARTzTxBuffer[23]^USARTzTxBuffer[24]^USARTzTxBuffer[25];                        //CRC
	
	
	com_x_usart_dma_start_tx(27);	//���ݰ�����
}
//���ݽ��շ���
int8_t get_data_analyze(uint8_t	*pdata)
{
	
	int8_t 	ret=0;
	int8_t	crc = 0;
	int8_t  p_crc = 0;
	if((*(pdata + 0) == 0xff) && (*(pdata + 1) == 0xff)){
		crc = (*(pdata + 2))^(*(pdata + 3))^(*(pdata + 4))^(*(pdata + 5))^(*(pdata + 6))^(*(pdata + 7))^(*(pdata + 8))^(*(pdata + 9))^(*(pdata + 10))^(*(pdata + 11))^(*(pdata + 12))^(*(pdata + 13));
		p_crc = (int8_t)(*(pdata + 14));//����������ת��������������
	}
	else{
		ret = -1;
		return ret;
	}
	if(p_crc != crc ){//У��ͷ�������
		ret = -1;
		return ret;
	}
	//���ݰ�������ȷ����ȡ����
	memset(&com_x_rcv_data,0,sizeof(com_x_rcv_data));
	com_x_rcv_data.linear_vx.cv[0] = *(pdata + 2);
	com_x_rcv_data.linear_vx.cv[1] = *(pdata + 3);
	com_x_rcv_data.linear_vx.cv[2] = *(pdata + 4);
	com_x_rcv_data.linear_vx.cv[3] = *(pdata + 5);
	
	com_x_rcv_data.linear_vy.cv[0] = *(pdata + 6);
	com_x_rcv_data.linear_vy.cv[1] = *(pdata + 7);
	com_x_rcv_data.linear_vy.cv[2] = *(pdata + 8);
	com_x_rcv_data.linear_vy.cv[3] = *(pdata + 9);
	
	com_x_rcv_data.angular_v.cv[0] = *(pdata + 10);
	com_x_rcv_data.angular_v.cv[1] = *(pdata + 11);
	com_x_rcv_data.angular_v.cv[2] = *(pdata + 12);
	com_x_rcv_data.angular_v.cv[3] = *(pdata + 13);
	
	return ret;
	
}




