#include "timer.h"
/**************************************************************************
�������ܣ���ʱ��3ͨ��3���벶���ʼ��
��ڲ�������ڲ�����arr���Զ���װֵ  psc��ʱ��Ԥ��Ƶ�� 
����  ֵ����
**************************************************************************/

void TIM3_Cap_Init(u16 arr,u16 psc)	
{	 	
	TIM_ICInitTypeDef  TIM3_ICInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  	//TIM3ʱ��ʹ��    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOC, ENABLE); 	//ʹ��PORTB Cʱ��	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; //GPIOA0
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; //����
	GPIO_Init(GPIOB,&GPIO_InitStructure); //��ʼ��PB0
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;  //PC2���
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOC, &GPIO_InitStructure);//��ʼ��
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource0,GPIO_AF_TIM3); //PB0����λ��ʱ��3
  
	  
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period=arr;   //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);
	

	//��ʼ��TIM3���벶�����
	TIM3_ICInitStructure.TIM_Channel = TIM_Channel_3; //	ѡ������� IC3ӳ�䵽TI3��
  TIM3_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
  TIM3_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
  TIM3_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //���������Ƶ,����Ƶ 
  TIM3_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 ���������˲��� ���˲�
  TIM_ICInit(TIM3, &TIM3_ICInitStructure);
		
	TIM_ITConfig(TIM3,TIM_IT_Update|TIM_IT_CC3,ENABLE);//���������ж� ,����CC3IE�����ж�	
	
  TIM_Cmd(TIM3,ENABLE ); 	//ʹ�ܶ�ʱ��3

 
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =2;		//�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����
}
/**************************************************************************
�������ܣ����������ջز�����
��ڲ�������
����  ֵ����
**************************************************************************/
u16 TIM3CH3_CAPTURE_STA,TIM3CH3_CAPTURE_VAL;
void Read_Distane(void)
{   
	 PCout(2)=1;
	 delay_us(15);  
	 PCout(2)=0;	
			if(TIM3CH3_CAPTURE_STA&0X80)//�ɹ�������һ�θߵ�ƽ
		{
			Distance=TIM3CH3_CAPTURE_STA&0X3F;
			Distance*=65536;					        //���ʱ���ܺ�
			Distance+=TIM3CH3_CAPTURE_VAL;		//�õ��ܵĸߵ�ƽʱ��
			Distance=Distance*170/1000;
		//	printf("%d \r\n",Distance);
			TIM3CH3_CAPTURE_STA=0;			//������һ�β���
		}				
}
/**************************************************************************
�������ܣ��������ز�������ȡ�ж�
��ڲ�������
����  ֵ����
��    �ߣ�**
**************************************************************************/
void TIM3_IRQHandler(void)
{ 		    		  			    
	if((TIM3CH3_CAPTURE_STA&0X80)==0)//��δ�ɹ�����	
				{
								if(TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)//���
								{	    
										if(TIM3CH3_CAPTURE_STA&0X40)//�Ѿ����񵽸ߵ�ƽ��
										{
											if((TIM3CH3_CAPTURE_STA&0X3F)==0X3F)//�ߵ�ƽ̫����
											{
												TIM3CH3_CAPTURE_STA|=0X80;//��ǳɹ�������һ��
												TIM3CH3_CAPTURE_VAL=0XFFFF;
											}else TIM3CH3_CAPTURE_STA++;
										}	 
								}
						if(TIM_GetITStatus(TIM3, TIM_IT_CC3) != RESET)//����3���������¼�
				    	{	
											if(TIM3CH3_CAPTURE_STA&0X40)		//����һ���½��� 		
											{	  			
											TIM3CH3_CAPTURE_STA|=0X80;		//��ǳɹ�����һ�θߵ�ƽ����
											TIM3CH3_CAPTURE_VAL=TIM_GetCapture3(TIM3);	//��ȡ��ǰ�Ĳ���ֵ.
											TIM_OC3PolarityConfig(TIM3,TIM_ICPolarity_Rising); //����Ϊ�����ز���
									  	}else  								//��δ��ʼ,��һ�β���������
											{
											TIM3CH3_CAPTURE_STA=0;			//���
											TIM3CH3_CAPTURE_VAL=0;
											TIM3CH3_CAPTURE_STA|=0X40;		//��ǲ�����������
											TIM_Cmd(TIM3,DISABLE ); 	//�رն�ʱ��
											TIM_SetCounter(TIM3,0);
											TIM_OC3PolarityConfig(TIM3,TIM_ICPolarity_Falling);				// ����Ϊ�½��ز���
											TIM_Cmd(TIM3,ENABLE ); 	//ʹ�ܶ�ʱ��
											}		    
					    	}			     	    					   
		   }
	TIM_ClearITPendingBit(TIM3, TIM_IT_CC3|TIM_IT_Update); //����жϱ�־λ    
}



u8 TIM8CH1_CAPTURE_STA = 0;	//ͨ��1���벶���־������λ�������־����6λ�������־		
u16 TIM8CH1_CAPTURE_UPVAL;
u16 TIM8CH1_CAPTURE_DOWNVAL;

u8 TIM8CH2_CAPTURE_STA = 0;	//ͨ��2���벶���־������λ�������־����6λ�������־		
u16 TIM8CH2_CAPTURE_UPVAL;
u16 TIM8CH2_CAPTURE_DOWNVAL;

u8 TIM8CH3_CAPTURE_STA = 0;	//ͨ��3���벶���־������λ�������־����6λ�������־		
u16 TIM8CH3_CAPTURE_UPVAL;
u16 TIM8CH3_CAPTURE_DOWNVAL;

u8 TIM8CH4_CAPTURE_STA = 0;	//ͨ��1���벶���־������λ�������־����6λ�������־		
u16 TIM8CH4_CAPTURE_UPVAL;
u16 TIM8CH4_CAPTURE_DOWNVAL;


u32 TIM8_T1;
u32 TIM8_T2;
u32 TIM8_T3;
u32 TIM8_T4;


/**************************************************************************
�������ܣ���ģң�س�ʼ������
��ڲ�����arr���Զ���װֵ  psc��ʱ��Ԥ��Ƶ�� 
����  ֵ����
**************************************************************************/
void TIM8_Cap_Init(u16 arr, u16 psc)
{
	TIM_ICInitTypeDef  TIM8_ICInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);  	//TIM8ʱ��ʹ��    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); 	//ʹ��PORT Cʱ��	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7; //GPIOA0
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; //����
	GPIO_Init(GPIOC,&GPIO_InitStructure); //��ʼ��PB0

	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_TIM8); //PC6����λ��ʱ��8
  GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_TIM8); //PC7����λ��ʱ��8
	  
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period=arr;   //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM8,&TIM_TimeBaseStructure);
		//��ʼ��TIM���벶�����
	TIM8_ICInitStructure.TIM_Channel = TIM_Channel_1; //	CH1 CH2
  TIM8_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
  TIM8_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
  TIM8_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //���������Ƶ,����Ƶ 
  TIM8_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 ���������˲��� ���˲�
  TIM_ICInit(TIM8, &TIM8_ICInitStructure);
	TIM8_ICInitStructure.TIM_Channel = TIM_Channel_2; //	CH1 CH2	
	TIM_ICInit(TIM8,&TIM8_ICInitStructure); 

  NVIC_InitStructure.NVIC_IRQChannel = TIM8_CC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����
	NVIC_InitStructure.NVIC_IRQChannel = TIM8_UP_TIM13_IRQn ; 
  NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����
	TIM_ClearITPendingBit(TIM8,TIM_IT_Update|TIM_IT_CC1|TIM_IT_CC2); 
	TIM_ITConfig(TIM8,TIM_IT_Update|TIM_IT_CC1|TIM_IT_CC2,ENABLE);//���������ж� ,���������ж�
	TIM_Cmd(TIM8,ENABLE ); 	//ʹ�ܶ�ʱ��
}

/**************************************************************************
�������ܣ���ģң�ؽ����ж�
��ڲ�������
����  ֵ����
**************************************************************************/
void TIM8_CC_IRQHandler(void)
{
	if ((TIM8CH1_CAPTURE_STA & 0X80) == 0) 		//��δ�ɹ�����	
	{
				if (TIM_GetITStatus(TIM8, TIM_IT_CC1) != RESET) 		//����1���������¼�
				{
								TIM_ClearITPendingBit(TIM8, TIM_IT_CC1); 		//����жϱ�־λ
								if (TIM8CH1_CAPTURE_STA & 0X40)		//����һ���½���
								{
									TIM8CH1_CAPTURE_DOWNVAL = TIM_GetCapture1(TIM8);//��¼�´�ʱ�Ķ�ʱ������ֵ
									if (TIM8CH1_CAPTURE_DOWNVAL < TIM8CH1_CAPTURE_UPVAL)
									{
										TIM8_T1 = 65535;
									}
									else
										TIM8_T1 = 0;
									Remoter_Ch1 = TIM8CH1_CAPTURE_DOWNVAL - TIM8CH1_CAPTURE_UPVAL
											+ TIM8_T1;		//�õ��ܵĸߵ�ƽ��ʱ��
									TIM8CH1_CAPTURE_STA = 0;		//�����־λ����
									TIM_OC1PolarityConfig(TIM8, TIM_ICPolarity_Rising); //����Ϊ�����ز���		  
								}
								else //��������ʱ�䵫�����½��أ���һ�β��������أ���¼��ʱ�Ķ�ʱ������ֵ
								{
									TIM8CH1_CAPTURE_UPVAL = TIM_GetCapture1(TIM8);		//��ȡ����������
									TIM8CH1_CAPTURE_STA |= 0X40;		//����Ѳ���������
									TIM_OC1PolarityConfig(TIM8, TIM_ICPolarity_Falling);//����Ϊ�½��ز���
								}
				}
	}

	if ((TIM8CH2_CAPTURE_STA & 0X80) == 0)		//��δ�ɹ�����	
	{
		if (TIM_GetITStatus(TIM8, TIM_IT_CC2) != RESET)		//����2���������¼�
		{
			TIM_ClearITPendingBit(TIM8, TIM_IT_CC2);		//����жϱ�־λ
			if (TIM8CH2_CAPTURE_STA & 0X40)		//����һ���½���
			{
				TIM8CH2_CAPTURE_DOWNVAL = TIM_GetCapture2(TIM8);//��¼�´�ʱ�Ķ�ʱ������ֵ
				if (TIM8CH2_CAPTURE_DOWNVAL < TIM8CH2_CAPTURE_UPVAL)
				{
					TIM8_T2 = 65535;
				}
				else
					TIM8_T2 = 0;
				Remoter_Ch2 = TIM8CH2_CAPTURE_DOWNVAL - TIM8CH2_CAPTURE_UPVAL
						+ TIM8_T2;		//�õ��ܵĸߵ�ƽ��ʱ��
				TIM8CH2_CAPTURE_STA = 0;		//�����־λ����
				TIM_OC2PolarityConfig(TIM8, TIM_ICPolarity_Rising); //����Ϊ�����ز���		  
			}
			else //��������ʱ�䵫�����½��أ���һ�β��������أ���¼��ʱ�Ķ�ʱ������ֵ
			{
				TIM8CH2_CAPTURE_UPVAL = TIM_GetCapture2(TIM8);		//��ȡ����������
				TIM8CH2_CAPTURE_STA |= 0X40;		//����Ѳ���������
				TIM_OC2PolarityConfig(TIM8, TIM_ICPolarity_Falling);//����Ϊ�½��ز���
			}
		}
	}

//	if ((TIM2CH3_CAPTURE_STA & 0X80) == 0)		//��δ�ɹ�����	
//	{
//		if (TIM_GetITStatus(TIM2, TIM_IT_CC3) != RESET)		//����3���������¼�
//		{
//			TIM_ClearITPendingBit(TIM2, TIM_IT_CC3);		//����жϱ�־λ
//			if (TIM2CH3_CAPTURE_STA & 0X40)		//����һ���½���
//			{
//				TIM2CH3_CAPTURE_DOWNVAL = TIM_GetCapture3(TIM2);//��¼�´�ʱ�Ķ�ʱ������ֵ
//				if (TIM2CH3_CAPTURE_DOWNVAL < TIM2CH3_CAPTURE_UPVAL)
//				{
//					TIM2_T3 = 65535;
//				}
//				else
//					TIM2_T3 = 0;
//				Remoter_Ch3 = TIM2CH3_CAPTURE_DOWNVAL - TIM2CH3_CAPTURE_UPVAL
//						+ TIM2_T3;		//�õ��ܵĸߵ�ƽ��ʱ��
//				TIM2CH3_CAPTURE_STA = 0;		//�����־λ����
//				TIM_OC3PolarityConfig(TIM2, TIM_ICPolarity_Rising); //����Ϊ�����ز���		  
//			}
//			else //��������ʱ�䵫�����½��أ���һ�β��������أ���¼��ʱ�Ķ�ʱ������ֵ
//			{
//				TIM2CH3_CAPTURE_UPVAL = TIM_GetCapture3(TIM2);		//��ȡ����������
//				TIM2CH3_CAPTURE_STA |= 0X40;		//����Ѳ���������
//				TIM_OC3PolarityConfig(TIM2, TIM_ICPolarity_Falling);//����Ϊ�½��ز���
//			}
//		}
//	}

//	if ((TIM2CH4_CAPTURE_STA & 0X80) == 0)		//��δ�ɹ�����	
//	{
//		if (TIM_GetITStatus(TIM2, TIM_IT_CC4) != RESET)		//����4���������¼�
//		{
//			TIM_ClearITPendingBit(TIM2, TIM_IT_CC4);		//����жϱ�־λ
//			if (TIM2CH4_CAPTURE_STA & 0X40)		//����һ���½���
//			{
//				TIM2CH4_CAPTURE_DOWNVAL = TIM_GetCapture4(TIM2);//��¼�´�ʱ�Ķ�ʱ������ֵ
//				if (TIM2CH4_CAPTURE_DOWNVAL < TIM2CH4_CAPTURE_UPVAL)
//				{
//					TIM2_T4 = 65535;
//				}
//				else
//					TIM2_T4 = 0;
//				Remoter_Ch4 = TIM2CH4_CAPTURE_DOWNVAL - TIM2CH4_CAPTURE_UPVAL
//						+ TIM2_T4;		//�õ��ܵĸߵ�ƽ��ʱ��
//				TIM2CH4_CAPTURE_STA = 0;		//�����־λ����
//				TIM_OC4PolarityConfig(TIM2, TIM_ICPolarity_Rising); //����Ϊ�����ز���		  
//			}
//			else //��������ʱ�䵫�����½��أ���һ�β��������أ���¼��ʱ�Ķ�ʱ������ֵ
//			{
//				TIM2CH4_CAPTURE_UPVAL = TIM_GetCapture4(TIM2);		//��ȡ����������
//				TIM2CH4_CAPTURE_STA |= 0X40;		//����Ѳ���������
//				TIM_OC4PolarityConfig(TIM2, TIM_ICPolarity_Falling);//����Ϊ�½��ز���
//			}
//		}
//	}
}

void TIM8_UP_TIM13_IRQHandler(void) 
{ 
// 
  TIM_ClearITPendingBit(TIM8,TIM_IT_Update);  
}

