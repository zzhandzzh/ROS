#include "IOI2C.h"
#include "delay.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_Init(void)
*��������:		��ʼ��I2C��Ӧ�Ľӿ����š�
*******************************************************************************/

void Delay(u32 count)//���ڲ���400KHzIIC�ź�����Ҫ����ʱ
{
	while (count--);
}
void jy_IIC_Init(void)
{			
	GPIO_InitTypeDef GPIO_InitStructure;
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);			     
 	//����PB6 PB7 Ϊ��©���  ˢ��Ƶ��Ϊ10Mhz
 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;       
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	//Ӧ�����õ�GPIOB 
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	jy_SDA_OUT();     //sda�����
	jy_IIC_SDA=1;	  	  
	jy_IIC_SCL=1;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_Start(void)
*��������:		����IIC��ʼ�ź�
*******************************************************************************/
void jy_IIC_Start(void)
{
	jy_SDA_OUT();     //sda�����
	jy_IIC_SDA=1;	  	  
	jy_IIC_SCL=1;
	
	Delay(5);
 	jy_IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	
	Delay(5);
	jy_IIC_SCL=0;//ǯסI2C���ߣ�׼�����ͻ�������� 
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_Stop(void)
*��������:	    //����IICֹͣ�ź�
*******************************************************************************/	  
void jy_IIC_Stop(void)
{
	jy_SDA_OUT();//sda�����
	jy_IIC_SCL=0;
	jy_IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	
		Delay(5);
	jy_IIC_SCL=1; 
	jy_IIC_SDA=1;//����I2C���߽����ź�
	
		Delay(5);							   	
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IIC_Wait_Ack(void)
*��������:	    �ȴ�Ӧ���źŵ��� 
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
*******************************************************************************/
u8 jy_IIC_Wait_Ack(void)
{
	u8 ucErrTime=0; 
	jy_SDA_IN();      //SDA����Ϊ����  
	jy_IIC_SDA=1;
		Delay(5);	  
	while(jy_READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>50)
		{
			jy_IIC_Stop();
			return 1;
		}
		Delay(5);
	}  
	jy_IIC_SCL=1;
	Delay(5); 
	jy_IIC_SCL=0;//ʱ�����0  
	return 0;  
} 

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_Ack(void)
*��������:	    ����ACKӦ��
*******************************************************************************/
void jy_IIC_Ack(void)
{
	jy_IIC_SCL=0;
	jy_SDA_OUT();
	jy_IIC_SDA=0;
		Delay(5);
	jy_IIC_SCL=1;
		Delay(5);
	jy_IIC_SCL=0;
}
	
/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_NAck(void)
*��������:	    ����NACKӦ��
*******************************************************************************/	    
void jy_IIC_NAck(void)
{
	jy_IIC_SCL=0;
	jy_SDA_OUT();
	jy_IIC_SDA=1;
	
		Delay(5);
	jy_IIC_SCL=1;
		Delay(5);
	jy_IIC_SCL=0;
}					 				     

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_Send_Byte(u8 txd)
*��������:	    IIC����һ���ֽ�
*******************************************************************************/		  
void jy_IIC_Send_Byte(u8 txd)
{                        
    u8 t; 
		jy_SDA_OUT(); 	    
    jy_IIC_SCL=0;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        jy_IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
			
		Delay(2);   
		jy_IIC_SCL=1;
		Delay(5);
		jy_IIC_SCL=0;	
		Delay(3);
    }	 
} 	 
   
/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IIC_Read_Byte(unsigned char ack)
*��������:	    //��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK 
*******************************************************************************/  
u8 jy_IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	jy_SDA_IN();//SDA����Ϊ����
    for(i=0;i<8;i++ )
	{
        jy_IIC_SCL=0; 
        
		Delay(5);
		jy_IIC_SCL=1;
        receive<<=1;
        if(jy_READ_SDA)receive++;   
		
		Delay(5); 
    }					 
    if (ack)
        jy_IIC_Ack(); //����ACK 
    else
        jy_IIC_NAck();//����nACK  
    return receive;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data)
*��������:	    ��ȡָ���豸 ָ���Ĵ����� length��ֵ
����	dev  Ŀ���豸��ַ
		reg	  �Ĵ�����ַ
		length Ҫ�����ֽ���
		*data  ���������ݽ�Ҫ��ŵ�ָ��
����   ���������ֽ�����
*******************************************************************************/ 
u8 jy_IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data){
    u8 count = 0;
	
	jy_IIC_Start();
	jy_IIC_Send_Byte(dev<<1);	   //����д����
	jy_IIC_Wait_Ack();
	jy_IIC_Send_Byte(reg);   //���͵�ַ
  jy_IIC_Wait_Ack();	  
	jy_IIC_Start();
	jy_IIC_Send_Byte((dev<<1)+1);  //�������ģʽ	
	jy_IIC_Wait_Ack();
	
    for(count=0;count<length;count++){
		 
		 if(count!=length-1)data[count]=jy_IIC_Read_Byte(1);  //��ACK�Ķ�����
		 	else  data[count]=jy_IIC_Read_Byte(0);	 //���һ���ֽ�NACK
	}
    jy_IIC_Stop();//����һ��ֹͣ����
    return count;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data)
*��������:	    ������ֽ�д��ָ���豸 ָ���Ĵ���
����	dev  Ŀ���豸��ַ
		reg	  �Ĵ�����ַ
		length Ҫд���ֽ���
		*data  ��Ҫд�����ݵ��׵�ַ
����   �����Ƿ�ɹ�
*******************************************************************************/ 
u8 jy_IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data){
  
 	u8 count = 0;
	jy_IIC_Start();
	jy_IIC_Send_Byte(dev<<1);	   //����д����
	jy_IIC_Wait_Ack();
	jy_IIC_Send_Byte(reg);   //���͵�ַ
	jy_IIC_Wait_Ack();	  
	for(count=0;count<length;count++){
		jy_IIC_Send_Byte(data[count]); 
		jy_IIC_Wait_Ack(); 
 }
	jy_IIC_Stop();//����һ��ֹͣ����

    return 1; //status == 0;
	
}
