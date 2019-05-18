#include "sys.h"
#include "usart.h"	
////////////////////////////////////////////////////////////////////////////////// 	 
//如果使用ucos,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos 使用	  
#endif


//////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
_sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	
	if(Flag_Show==0)
	{	
	while((USART2->SR&0X40)==0);//Flag_Show=0  使用串口2
	USART2->DR = (u8) ch;      
	}
	/* 串口1 屏蔽，用作树莓派通信
	else
	{	
	while((USART1->SR&0X40)==0);//Flag_Show!=0  使用串口1   
	USART1->DR = (u8) ch;      
	}	
	*/
	return ch;
}
#endif
 
uint8_t USARTzTxBuffer[USARTzTxBufferSize];
uint8_t USARTzRxBuffer[USARTzTxBufferSize];
uint8_t USARTzRxBufferD[USARTzTxBufferSize];

send_data	com_x_send_data;//数据发送
rcv_data		com_x_rcv_data;//数据接收


//串口1中断服务程序//改成串口加DMA读取27字节一帧的数据
//bound:波特率
void uart_init(u32 bound){
   //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//使能USART1时钟
 
	//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9复用为USART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10复用为USART1
	
	//USART1端口配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA9，PA10

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART1, &USART_InitStructure); //初始化串口1

	/* Enable USART1 Receive  interrupts */
  USART_ITConfig(USARTz, USART_IT_IDLE, ENABLE); //开启串口空闲IDLE中断
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
 
    DMA_Init(DMA2_Stream7, &DMA_InitStructure);		//USART1请求映射DMA2Channel4数据流7	//配置DMA
    DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, ENABLE);//使能DMA中断
    USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);//使能串口的DMA发送接口

  
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
		//接收不需要DMA中断
    USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);
		DMA_Cmd(DMA2_Stream5,ENABLE);
}


//************************************DMA发送********************************
void DMA2_Stream7_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA2_Stream7,DMA_IT_TCIF7))
    {
      DMA_ClearFlag(DMA2_Stream7,DMA_IT_TCIF7|DMA_IT_TEIF7);    //清除DMA所有标志    
			DMA_Cmd(DMA2_Stream7, DISABLE);  //关闭DMA发送通道
      //USART_ITConfig(USART1,USART_IT_TC,ENABLE);
    }
}

void com_x_usart_dma_start_tx(uint8_t size)
{
    USARTz_Tx_DMA_Channe->NDTR = (uint16_t)size; //重新赋值 指定发送缓存长度
    DMA_Cmd(DMA2_Stream7, ENABLE);  //开启DMA发送      
}


void USART1_IRQHandler(void)                	//串口1中断服务程序
{
  
	if(USART_GetITStatus(USARTz, USART_IT_IDLE) != RESET)  
    {
        com_x_usart_dma_read();
				USART_ReceiveData( USARTz );
    }
} 

/*中断解析*/
//void USART1_IRQFuc(void)   
//{  

//    deal_irq_tx_end();   

//    tUART1_Rx.dwUART1RxLen = deal_irq_rx_end(tUART1_Rx.UART1RxBuf); 
//} 
/*接收中断解析*/
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
		DMA_Cmd(DMA2_Stream5, DISABLE);    //关闭DMA防止干扰   
    DMA_ClearFlag(DMA2_Stream5,DMA_IT_TCIF5|DMA_IT_TEIF5);   //清除标志位       
    rxcounter= USARTzTxBufferSize - DMA_GetCurrDataCounter(DMA2_Stream5);//获取接收到的字节数 
    DMA2_Stream5->NDTR = USARTzTxBufferSize; //重新赋值计数值   
    memset(USARTzRxBufferD,0,sizeof(USARTzRxBufferD));
		printf("Received data:");
	  for(i=0;i<rxcounter;i++){
			USARTzRxBufferD[i] = USARTzRxBuffer[i];//获取接收到的数据，存入数组RxBufferD中
			printf("0x%02x",USARTzRxBufferD[i]);
		}
		printf("\n\r");
		//分析收到的数据包
		//DINT();//禁止中断
		if(get_data_analyze(USARTzRxBufferD) != 0)
			printf("data analyze error\n\r");
		//EINT();//允许中断
		for(i=0;i<rxcounter;i++)
			USARTzRxBuffer[i] = 0;//clear Rx buffer
		DMA_Cmd(DMA2_Stream5, ENABLE);  //DMA开启 等待下一帧数据     

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
//测试
void data_pack(void)
{
	
	com_x_send_data.x_pos.fv = 2.68;//x坐标
	com_x_send_data.y_pos.fv = 3.96;//y坐标
	com_x_send_data.x_v.fv	= 0.6;//x方向速度
	com_x_send_data.y_v.fv = 0.0;//y 方向速度
	com_x_send_data.angular_v.fv = 2.0;//角速度 绕z轴
	com_x_send_data.pose_angular.fv = 1.0;//yaw偏航角
	
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
	
	
	com_x_usart_dma_start_tx(27);	//数据包发送
}
//数据接收分析
int8_t get_data_analyze(uint8_t	*pdata)
{
	
	int8_t 	ret=0;
	int8_t	crc = 0;
	int8_t  p_crc = 0;
	if((*(pdata + 0) == 0xff) && (*(pdata + 1) == 0xff)){
		crc = (*(pdata + 2))^(*(pdata + 3))^(*(pdata + 4))^(*(pdata + 5))^(*(pdata + 6))^(*(pdata + 7))^(*(pdata + 8))^(*(pdata + 9))^(*(pdata + 10))^(*(pdata + 11))^(*(pdata + 12))^(*(pdata + 13));
		p_crc = (int8_t)(*(pdata + 14));//不进行类型转换，负数不正常
	}
	else{
		ret = -1;
		return ret;
	}
	if(p_crc != crc ){//校验和分析有误
		ret = -1;
		return ret;
	}
	//数据包分析正确，提取数据
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




