#ifndef __IOI2C_H
#define __IOI2C_H
#include "sys.h"  	   		   
//IO方向设置
#define jy_SDA_IN()  {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=0x00008000;}
#define jy_SDA_OUT() {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=0x00003000;}
//#define SDA_IN()  {GPIOC->CRH&=0XFFF0FFFF;GPIOC->CRH|=0x00080000;}
//#define SDA_OUT() {GPIOC->CRH&=0XFFF0FFFF;GPIOC->CRH|=0x00030000;}
//IO操作函数	 
#define jy_IIC_SCL    PBout(10) //SCL
#define jy_IIC_SDA    PBout(11) //SDA
#define jy_READ_SDA   PBin(11)  //输入SDA
//#define IIC_SCL    PCout(11) //SCL
//#define IIC_SDA    PCout(12) //SDA
//#define READ_SDA   PCin(12)  //输入SDA

//IIC所有操作函数
void jy_IIC_Init(void);                //初始化IIC的IO口				 
void jy_IIC_Start(void);				//发送IIC开始信号
void jy_IIC_Stop(void);	  			//发送IIC停止信号
void jy_IIC_Send_Byte(u8 txd);			//IIC发送一个字节
u8 jy_IIC_Read_Byte(unsigned char ack);//IIC读取一个字节
u8 jy_IIC_Wait_Ack(void); 				//IIC等待ACK信号
void jy_IIC_Ack(void);					//IIC发送ACK信号
void jy_IIC_NAck(void);				//IIC不发送ACK信号

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
