/**
  ******************************************************************************
  * @file    GPIO/IOToggle/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and peripherals
  *          interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h" 
#include "sys.h"

u8  Uart1_Buffer[18];
u16 Uart1_Rx=0;
u8 Receive[50];
int Usart_Receive;

void NMI_Handler(void)
{
}
 
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}
 
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

 
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}
 
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}
 
void SVC_Handler(void)
{
}
 
void DebugMon_Handler(void)
{
}
 
void PendSV_Handler(void)
{
}
 
void SysTick_Handler(void)
{
}
// 串口中断服务函数
void USART1_IRQHandler(void)  
{ 
if(USART_GetITStatus(USART1,USART_IT_RXNE) != RESET) //中断产生 
	{
		USART_ClearITPendingBit(USART1,USART_IT_RXNE); //清除中断标志
			 
		Uart1_Buffer[Uart1_Rx] = USART_ReceiveData(USART1);     //接收串口1数据到buff缓冲区
		
		Usart_Receive=Uart1_Buffer[Uart1_Rx] ;
		Uart1_Rx++; 
		
						
		
								if(Usart_Receive>=0x41&&Usart_Receive<=0x48)  
						{	
							Flag_Direction=Usart_Receive-0x40;
						}
						else	if(Usart_Receive<=8)   
						{			
							Flag_Direction=Usart_Receive;
						}	
						else  Flag_Direction=0;
     		 
		if(Uart1_Buffer[Uart1_Rx-1] == 'E' || Uart1_Rx == 18)    //如果接收到尾标识是换行符（或者等于最大接受数就清空重新接收）
		{
			if(Uart1_Buffer[0] == 'B')                      //检测到头标识是我们需要的 
			{
				DelayUs(50);
				K[0]=Uart1_Buffer[1]-48;
				K[1]=Uart1_Buffer[2]-48;
				K[2]=Uart1_Buffer[3]-48;
				T=1;				
				//printf("%d	%d	%d	%d",K[0],K[1],K[2],T);				//读取完成标志位
				DelayUs(50);
				OLED_ShowCHinese(36,0,0);//垂
		        OLED_ShowCHinese(54,0,1);//死
		        OLED_ShowCHinese(72,0,2);//挣
		        OLED_ShowCHinese(90,0,3);//扎
	            OLED_ShowNum(0,3,K[0],2,12);//显示ASCII字符
                OLED_ShowNum(18,3,K[1],2,12);
                OLED_ShowNum(36,3,K[2],2,12);                            
			} 
			else
			{
				Uart1_Rx=0;                                   //不是我们需要的数据或者达到最大接收数则开始重新接收
			}
		}
	
		
				if((T==1&&Uart1_Buffer[Uart1_Rx-1] == 'S' )|| Uart1_Rx == 18)    //如果接收到尾标识是换行符（或者等于最大接受数就清空重新接收）
		{
			if(Uart1_Buffer[0] == 'A')                      //检测到头标识是我们需要的 
			{
				DelayUs(50);
				Target[0]=Uart1_Buffer[1];
				Target[1]=Uart1_Buffer[2];
				Target[2]=Uart1_Buffer[3];
				T=2;				
//				printf("%d	%d	%d	%d",K[0],K[1],K[2],T);				//读取完成标志位
				DelayUs(50);
				OLED_ShowCHinese(36,0,0);//垂
				OLED_ShowCHinese(54,0,1);//死
		        OLED_ShowCHinese(72,0,2);//挣
		        OLED_ShowCHinese(90,0,3);//扎
	            OLED_ShowNum(0,15,Target[0]-48,2,12);//显示ASCII字符
                OLED_ShowNum(18,15,Target[1]-48,2,12);
                OLED_ShowNum(36,15,Target[2]-48,2,12);                            
			} 
			else
			{
				Uart1_Rx=0;                   //不是我们需要的数据或者达到最大接收数则开始重新接收
			}
		}
	 }
}  

void USART2_IRQHandler(void)  
{  
  uint8_t ucTemp;
	if(USART_GetITStatus(USART2,USART_IT_RXNE)!=RESET)
	{		
		ucTemp = USART_ReceiveData(USART2);
    USART_SendData(USART2,ucTemp);    
	}	   
}  

void USART3_IRQHandler(void)  
{  
  uint8_t ucTemp;
	if(USART_GetITStatus(USART3,USART_IT_RXNE)!=RESET)
	{		
		ucTemp = USART_ReceiveData(USART3);
    USART_SendData(USART3,ucTemp);    
	}	    
}  

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/
