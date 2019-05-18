#include "sys.h"
#include "usart.h"	  

int fputc(int ch,FILE *p)       //在使用printf时系统自动条用此函数    
{    
    USART_SendData(USART1,(u8)ch);      
    while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);    
    return ch;    

}    

bool isUartRxCompleted = false;

//int fputc(int ch,FILE *p)       //在使用printf时系统自动条用此函数    
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
* 函 数 名         : uart_init  
* 函数功能         : IO端口及串口1，时钟初始化函数    A9,A10    
* 输    入         : 无  
* 输    出         : 无  
*******************************************************************************/    
void uart1_init(u32 bt)    
{    
    GPIO_InitTypeDef GPIO_InitStructure;    //声明一个结构体变量，用来初始化GPIO    
    NVIC_InitTypeDef NVIC_InitStructure;     //中断结构体定义    
    USART_InitTypeDef  USART_InitStructure;   //串口结构体定义    

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


    USART_InitStructure.USART_BaudRate=bt;   //波特率设置为bt    
    USART_InitStructure.USART_WordLength=USART_WordLength_8b;    
    USART_InitStructure.USART_StopBits=USART_StopBits_1;    
    USART_InitStructure.USART_Parity=USART_Parity_No;    
    USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;    
    USART_InitStructure.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;    
    USART_Init(USART1,&USART_InitStructure);    
    USART_Cmd(USART1, ENABLE);    
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//使能或者失能指定的USART中断 接收中断    
    USART_ClearFlag(USART1,USART_FLAG_TC);//清除USARTx的待处理标志位     
}    

/*******************************************************************************  
* 函 数 名         : uart2_init  
* 函数功能         : IO端口及串口2，时钟初始化函数     A2,A3   
* 输    入         : 无  
* 输    出         : 无  
*******************************************************************************/    
void uart2_init(u32 bt)    
{    
     USART_InitTypeDef USART_InitStructure;    
  NVIC_InitTypeDef NVIC_InitStructure;     
    GPIO_InitTypeDef GPIO_InitStructure;    //声明一个结构体变量，用来初始化GPIO    
   //使能串口的RCC时钟    
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE); //使能UART3所在GPIOB的时钟    
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);    

   //串口使用的GPIO口配置    
   // Configure USART2 Rx (PB.11) as input floating      
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;    
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;    
   GPIO_Init(GPIOA, &GPIO_InitStructure);    

   // Configure USART2 Tx (PB.10) as alternate function push-pull    
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;    
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;    
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;    
   GPIO_Init(GPIOA, &GPIO_InitStructure);    

   //配置串口    
   USART_InitStructure.USART_BaudRate = bt;    
   USART_InitStructure.USART_WordLength = USART_WordLength_8b;    
   USART_InitStructure.USART_StopBits = USART_StopBits_1;    
   USART_InitStructure.USART_Parity = USART_Parity_No;    
   USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;    
   USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;    


   // Configure USART3     
   USART_Init(USART2, &USART_InitStructure);//配置串口3    

  // Enable USART1 Receive interrupts 使能串口接收中断    
   USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);    
   //串口发送中断在发送数据时开启    
   //USART_ITConfig(USART2, USART_IT_TXE, ENABLE);    

   // Enable the USART3     
   USART_Cmd(USART2, ENABLE);//使能串口3    

   //串口中断配置    
   //Configure the NVIC Preemption Priority Bits       
   NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);    

   // Enable the USART3 Interrupt     
   NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;    
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;    
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;    
   NVIC_Init(&NVIC_InitStructure);    
}    


/*******************************************************************************  
* 函 数 名         : uart3_init  
* 函数功能         : IO端口及串口3，时钟初始化函数   B10,B11     
* 输    入         : 无  
* 输    出         : 无  
*******************************************************************************/    
void uart3_init(u32 bt)    
{    
    USART_InitTypeDef USART_InitStructure;    
  NVIC_InitTypeDef NVIC_InitStructure;     
    GPIO_InitTypeDef GPIO_InitStructure;    //声明一个结构体变量，用来初始化GPIO    
   //使能串口的RCC时钟    
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE); //使能UART3所在GPIOB的时钟    
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);    

   //串口使用的GPIO口配置    
   // Configure USART2 Rx (PB.11) as input floating      
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;    
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;    
   GPIO_Init(GPIOB, &GPIO_InitStructure);    

   // Configure USART2 Tx (PB.10) as alternate function push-pull    
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;    
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;    
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;    
   GPIO_Init(GPIOB, &GPIO_InitStructure);    

   //配置串口    
   USART_InitStructure.USART_BaudRate = bt;    
   USART_InitStructure.USART_WordLength = USART_WordLength_8b;    
   USART_InitStructure.USART_StopBits = USART_StopBits_1;    
   USART_InitStructure.USART_Parity = USART_Parity_No;    
   USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;    
   USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;    


   // Configure USART3     
   USART_Init(USART3, &USART_InitStructure);//配置串口3    

  // Enable USART1 Receive interrupts 使能串口接收中断    
   USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);    
   //串口发送中断在发送数据时开启    
   //USART_ITConfig(USART2, USART_IT_TXE, ENABLE);    

   // Enable the USART3     
   USART_Cmd(USART3, ENABLE);//使能串口3    

   //串口中断配置    
   //Configure the NVIC Preemption Priority Bits       
   NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);    

   // Enable the USART3 Interrupt     
   NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;    
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;    
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;    
   NVIC_Init(&NVIC_InitStructure);    


}    

//初始化IO 串口1
//pclk2:PCLK2时钟频率(Mhz)
//bound:波特率 
void uart_init(u32 pclk2,u32 bound)
{  	 
	float temp;
	u16 mantissa;
	u16 fraction;	   
	temp=(float)(pclk2*1000000)/(bound*16);//得到USARTDIV
	mantissa=temp;				 //得到整数部分
	fraction=(temp-mantissa)*16; //得到小数部分	 
    mantissa<<=4;
	mantissa+=fraction; 
	RCC->APB2ENR|=1<<2;   //使能PORTA口时钟  
	RCC->APB2ENR|=1<<14;  //使能串口时钟 
	GPIOA->CRH&=0XFFFFF00F;//IO状态设置
	GPIOA->CRH|=0X000008B0;//IO状态设置 
	RCC->APB2RSTR|=1<<14;   //复位串口1
	RCC->APB2RSTR&=~(1<<14);//停止复位	   	   
	//波特率设置
 	USART1->BRR=mantissa; // 波特率设置	 
	USART1->CR1|=0X200C;  //1位停止,无校验位.
//#if EN_USART1_RX		  //如果使能了接收
	//使能接收中断 
	USART1->CR1|=1<<5;    //接收缓冲区非空中断使能	    	
	MY_NVIC_Init(3,3,USART1_IRQn,2);//组2，最低优先级 
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
	NVIC_InitStrue.NVIC_IRQChannelCmd=DISABLE; //不使用中断
	NVIC_InitStrue.NVIC_IRQChannelPreemptionPriority=1;
	NVIC_InitStrue.NVIC_IRQChannelSubPriority=1;
	NVIC_Init(&NVIC_InitStrue);
}



/*****************  发送一个字节 **********************/
void Usart_SendByte( USART_TypeDef * pUSARTx, uint8_t ch)
{
	/* 发送一个字节数据到USART */
	USART_SendData(pUSARTx,ch);
		
	/* 等待发送数据寄存器为空 */
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);	
}

/****************** 发送8位的数组 ************************/
void Usart_SendArray( USART_TypeDef * pUSARTx, uint8_t *array, uint16_t num)
{
  uint8_t i;
	
	for(i=0; i<num; i++)
  {
	    /* 发送一个字节数据到USART */
	    Usart_SendByte(pUSARTx,array[i]);	
  
  }
	/* 等待发送完成 */
	while(USART_GetFlagStatus(pUSARTx,USART_FLAG_TC)==RESET);
}

/*****************  发送字符串 **********************/
void Usart_SendString( USART_TypeDef * pUSARTx, char *str)
{
	unsigned int k=0;
  do 
  {
      Usart_SendByte( pUSARTx, *(str + k) );
      k++;
  } while(*(str + k)!='\0');
  
  /* 等待发送完成 */
  while(USART_GetFlagStatus(pUSARTx,USART_FLAG_TC)==RESET)
  {}
}

/*****************  发送一个16位数 **********************/
void Usart_SendHalfWord( USART_TypeDef * pUSARTx, uint16_t ch)
{
	uint8_t temp_h, temp_l;
	
	/* 取出高八位 */
	temp_h = (ch&0XFF00)>>8;
	/* 取出低八位 */
	temp_l = ch&0XFF;
	
	/* 发送高八位 */
	USART_SendData(pUSARTx,temp_h);	
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);
	
	/* 发送低八位 */
	USART_SendData(pUSARTx,temp_l);	
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);	
}
