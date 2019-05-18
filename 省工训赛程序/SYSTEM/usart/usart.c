#include "sys.h"
#include "usart.h"	  

int fputc(int ch,FILE *p)       //��ʹ��printfʱϵͳ�Զ����ô˺���    
{    
    USART_SendData(USART1,(u8)ch);      
    while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);    
    return ch;    

}    

bool isUartRxCompleted = false;

//int fputc(int ch,FILE *p)       //��ʹ��printfʱϵͳ�Զ����ô˺���    
//{    
//    USART_SendData(USART1,(u8)ch);      
//    while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);    
//    return ch;    

//}    
void uartWriteBuf(uint8_t *buf, uint8_t len)
{
	while (len--) {
		while ((USART2->SR & 0x40) == 0);
		USART_SendData(USART2,*buf++);
	}
}



/*******************************************************************************  
* �� �� ��         : uart_init  
* ��������         : IO�˿ڼ�����1��ʱ�ӳ�ʼ������    A9,A10    
* ��    ��         : ��  
* ��    ��         : ��  
*******************************************************************************/    
void uart1_init(u32 bt)    
{    
    GPIO_InitTypeDef GPIO_InitStructure;    //����һ���ṹ�������������ʼ��GPIO    
    NVIC_InitTypeDef NVIC_InitStructure;     //�жϽṹ�嶨��    
    USART_InitTypeDef  USART_InitStructure;   //���ڽṹ�嶨��    

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_USART1|RCC_APB2Periph_AFIO,ENABLE);    

    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9;//TX    
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;    
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;    
    GPIO_Init(GPIOA,&GPIO_InitStructure);    
    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10;//RX    
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;    
    GPIO_Init(GPIOA,&GPIO_InitStructure);    


    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);     
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;     
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;     
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;     
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;     
    NVIC_Init(&NVIC_InitStructure);    


    USART_InitStructure.USART_BaudRate=bt;   //����������Ϊbt    
    USART_InitStructure.USART_WordLength=USART_WordLength_8b;    
    USART_InitStructure.USART_StopBits=USART_StopBits_1;    
    USART_InitStructure.USART_Parity=USART_Parity_No;    
    USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;    
    USART_InitStructure.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;    
    USART_Init(USART1,&USART_InitStructure);    
    USART_Cmd(USART1, ENABLE);    
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//ʹ�ܻ���ʧ��ָ����USART�ж� �����ж�    
    USART_ClearFlag(USART1,USART_FLAG_TC);//���USARTx�Ĵ������־λ     
}    

/*******************************************************************************  
* �� �� ��         : uart2_init  
* ��������         : IO�˿ڼ�����2��ʱ�ӳ�ʼ������     A2,A3   
* ��    ��         : ��  
* ��    ��         : ��  
*******************************************************************************/    
void uart2_init(u32 bt)    
{    
     USART_InitTypeDef USART_InitStructure;    
  NVIC_InitTypeDef NVIC_InitStructure;     
    GPIO_InitTypeDef GPIO_InitStructure;    //����һ���ṹ�������������ʼ��GPIO    
   //ʹ�ܴ��ڵ�RCCʱ��    
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE); //ʹ��UART3����GPIOB��ʱ��    
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);    

   //����ʹ�õ�GPIO������    
   // Configure USART2 Rx (PB.11) as input floating      
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;    
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;    
   GPIO_Init(GPIOA, &GPIO_InitStructure);    

   // Configure USART2 Tx (PB.10) as alternate function push-pull    
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;    
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;    
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;    
   GPIO_Init(GPIOA, &GPIO_InitStructure);    

   //���ô���    
   USART_InitStructure.USART_BaudRate = bt;    
   USART_InitStructure.USART_WordLength = USART_WordLength_8b;    
   USART_InitStructure.USART_StopBits = USART_StopBits_1;    
   USART_InitStructure.USART_Parity = USART_Parity_No;    
   USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;    
   USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;    


   // Configure USART3     
   USART_Init(USART2, &USART_InitStructure);//���ô���3    

  // Enable USART1 Receive interrupts ʹ�ܴ��ڽ����ж�    
   USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);    
   //���ڷ����ж��ڷ�������ʱ����    
   //USART_ITConfig(USART2, USART_IT_TXE, ENABLE);    

   // Enable the USART3     
   USART_Cmd(USART2, ENABLE);//ʹ�ܴ���3    

   //�����ж�����    
   //Configure the NVIC Preemption Priority Bits       
   NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);    

   // Enable the USART3 Interrupt     
   NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;    
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;    
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;    
   NVIC_Init(&NVIC_InitStructure);    
}    


/*******************************************************************************  
* �� �� ��         : uart3_init  
* ��������         : IO�˿ڼ�����3��ʱ�ӳ�ʼ������   B10,B11     
* ��    ��         : ��  
* ��    ��         : ��  
*******************************************************************************/    
void uart3_init(u32 bt)    
{    
    USART_InitTypeDef USART_InitStructure;    
  NVIC_InitTypeDef NVIC_InitStructure;     
    GPIO_InitTypeDef GPIO_InitStructure;    //����һ���ṹ�������������ʼ��GPIO    
   //ʹ�ܴ��ڵ�RCCʱ��    
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE); //ʹ��UART3����GPIOB��ʱ��    
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);    

   //����ʹ�õ�GPIO������    
   // Configure USART2 Rx (PB.11) as input floating      
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;    
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;    
   GPIO_Init(GPIOB, &GPIO_InitStructure);    

   // Configure USART2 Tx (PB.10) as alternate function push-pull    
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;    
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;    
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;    
   GPIO_Init(GPIOB, &GPIO_InitStructure);    

   //���ô���    
   USART_InitStructure.USART_BaudRate = bt;    
   USART_InitStructure.USART_WordLength = USART_WordLength_8b;    
   USART_InitStructure.USART_StopBits = USART_StopBits_1;    
   USART_InitStructure.USART_Parity = USART_Parity_No;    
   USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;    
   USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;    


   // Configure USART3     
   USART_Init(USART3, &USART_InitStructure);//���ô���3    

  // Enable USART1 Receive interrupts ʹ�ܴ��ڽ����ж�    
   USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);    
   //���ڷ����ж��ڷ�������ʱ����    
   //USART_ITConfig(USART2, USART_IT_TXE, ENABLE);    

   // Enable the USART3     
   USART_Cmd(USART3, ENABLE);//ʹ�ܴ���3    

   //�����ж�����    
   //Configure the NVIC Preemption Priority Bits       
   NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);    

   // Enable the USART3 Interrupt     
   NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;    
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;    
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;    
   NVIC_Init(&NVIC_InitStructure);    


}    

//��ʼ��IO ����1
//pclk2:PCLK2ʱ��Ƶ��(Mhz)
//bound:������ 
void uart_init(u32 pclk2,u32 bound)
{  	 
	float temp;
	u16 mantissa;
	u16 fraction;	   
	temp=(float)(pclk2*1000000)/(bound*16);//�õ�USARTDIV
	mantissa=temp;				 //�õ���������
	fraction=(temp-mantissa)*16; //�õ�С������	 
    mantissa<<=4;
	mantissa+=fraction; 
	RCC->APB2ENR|=1<<2;   //ʹ��PORTA��ʱ��  
	RCC->APB2ENR|=1<<14;  //ʹ�ܴ���ʱ�� 
	GPIOA->CRH&=0XFFFFF00F;//IO״̬����
	GPIOA->CRH|=0X000008B0;//IO״̬���� 
	RCC->APB2RSTR|=1<<14;   //��λ����1
	RCC->APB2RSTR&=~(1<<14);//ֹͣ��λ	   	   
	//����������
 	USART1->BRR=mantissa; // ����������	 
	USART1->CR1|=0X200C;  //1λֹͣ,��У��λ.
//#if EN_USART1_RX		  //���ʹ���˽���
	//ʹ�ܽ����ж� 
	USART1->CR1|=1<<5;    //���ջ������ǿ��ж�ʹ��	    	
	MY_NVIC_Init(3,3,USART1_IRQn,2);//��2��������ȼ� 
//#endif
}

void USART2_Init(u32 My_BaudRate)
{
	GPIO_InitTypeDef GPIO_InitStrue;
	USART_InitTypeDef USART_InitStrue;
	NVIC_InitTypeDef NVIC_InitStrue;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
	USART_DeInit(USART2);  
	
	GPIO_InitStrue.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_InitStrue.GPIO_Pin=GPIO_Pin_2;
	GPIO_InitStrue.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStrue);
	
	GPIO_InitStrue.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	GPIO_InitStrue.GPIO_Pin=GPIO_Pin_3;
  GPIO_Init(GPIOA,&GPIO_InitStrue);
	
	USART_InitStrue.USART_BaudRate=My_BaudRate; 
	USART_InitStrue.USART_HardwareFlowControl=USART_HardwareFlowControl_None; 
	USART_InitStrue.USART_Mode=USART_Mode_Tx|USART_Mode_Rx; 
	USART_InitStrue.USART_Parity=USART_Parity_No; 
	USART_InitStrue.USART_StopBits=USART_StopBits_1; 
	USART_InitStrue.USART_WordLength=USART_WordLength_8b; 
	USART_Init(USART2,&USART_InitStrue);
	
	USART_Cmd(USART2,ENABLE);
	USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);
	
	NVIC_InitStrue.NVIC_IRQChannel=USART2_IRQn;
	NVIC_InitStrue.NVIC_IRQChannelCmd=DISABLE; //��ʹ���ж�
	NVIC_InitStrue.NVIC_IRQChannelPreemptionPriority=1;
	NVIC_InitStrue.NVIC_IRQChannelSubPriority=1;
	NVIC_Init(&NVIC_InitStrue);
}



/*****************  ����һ���ֽ� **********************/
void Usart_SendByte( USART_TypeDef * pUSARTx, uint8_t ch)
{
	/* ����һ���ֽ����ݵ�USART */
	USART_SendData(pUSARTx,ch);
		
	/* �ȴ��������ݼĴ���Ϊ�� */
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);	
}

/****************** ����8λ������ ************************/
void Usart_SendArray( USART_TypeDef * pUSARTx, uint8_t *array, uint16_t num)
{
  uint8_t i;
	
	for(i=0; i<num; i++)
  {
	    /* ����һ���ֽ����ݵ�USART */
	    Usart_SendByte(pUSARTx,array[i]);	
  
  }
	/* �ȴ�������� */
	while(USART_GetFlagStatus(pUSARTx,USART_FLAG_TC)==RESET);
}

/*****************  �����ַ��� **********************/
void Usart_SendString( USART_TypeDef * pUSARTx, char *str)
{
	unsigned int k=0;
  do 
  {
      Usart_SendByte( pUSARTx, *(str + k) );
      k++;
  } while(*(str + k)!='\0');
  
  /* �ȴ�������� */
  while(USART_GetFlagStatus(pUSARTx,USART_FLAG_TC)==RESET)
  {}
}

/*****************  ����һ��16λ�� **********************/
void Usart_SendHalfWord( USART_TypeDef * pUSARTx, uint16_t ch)
{
	uint8_t temp_h, temp_l;
	
	/* ȡ���߰�λ */
	temp_h = (ch&0XFF00)>>8;
	/* ȡ���Ͱ�λ */
	temp_l = ch&0XFF;
	
	/* ���͸߰�λ */
	USART_SendData(pUSARTx,temp_h);	
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);
	
	/* ���͵Ͱ�λ */
	USART_SendData(pUSARTx,temp_l);	
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);	
}
