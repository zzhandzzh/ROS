#ifndef __IOI2C_H
#define __IOI2C_H
#include "sys.h"  	   		   
//IO��������
#define jy_SDA_IN()  {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=0x00008000;}
#define jy_SDA_OUT() {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=0x00003000;}
//#define SDA_IN()  {GPIOC->CRH&=0XFFF0FFFF;GPIOC->CRH|=0x00080000;}
//#define SDA_OUT() {GPIOC->CRH&=0XFFF0FFFF;GPIOC->CRH|=0x00030000;}
//IO��������	 
#define jy_IIC_SCL    PBout(10) //SCL
#define jy_IIC_SDA    PBout(11) //SDA
#define jy_READ_SDA   PBin(11)  //����SDA
//#define IIC_SCL    PCout(11) //SCL
//#define IIC_SDA    PCout(12) //SDA
//#define READ_SDA   PCin(12)  //����SDA

//IIC���в�������
void jy_IIC_Init(void);                //��ʼ��IIC��IO��				 
void jy_IIC_Start(void);				//����IIC��ʼ�ź�
void jy_IIC_Stop(void);	  			//����IICֹͣ�ź�
void jy_IIC_Send_Byte(u8 txd);			//IIC����һ���ֽ�
u8 jy_IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
u8 jy_IIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
void jy_IIC_Ack(void);					//IIC����ACK�ź�
void jy_IIC_NAck(void);				//IIC������ACK�ź�

void jy_IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 jy_IIC_Read_One_Byte(u8 daddr,u8 addr);	 
unsigned char jy_I2C_Readkey(unsigned char I2C_Addr);

unsigned char jy_I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr);
unsigned char jy_IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data);
unsigned char jy_IICwriteCmd(unsigned char dev, unsigned char cmd);
u8 jy_IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data);
u8 jy_IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data);
u8 jy_IICwriteBit(u8 dev,u8 reg,u8 bitNum,u8 data);
u8 jy_IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data);

#endif

//------------------End of File----------------------------
