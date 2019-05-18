#ifndef __SYS_H
#define __SYS_H	
#include "stm32f10x.h"
#include "stm32f10x_it.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32开发板		   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2012/8/18
//版本：V1.7
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved
////////////////////////////////////////////////////////////////////////////////// 	 

//0,不支持ucos
//1,支持ucos
#define SYSTEM_SUPPORT_OS		0		//定义系统文件夹是否支持UCOS
																	    
	 
//位带操作,实现51类似的GPIO控制功能
//具体实现思想,参考<<CM3权威指南>>第五章(87页~92页).
//IO口操作宏定义
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
//IO口地址映射
#define GPIOA_ODR_Addr    (GPIOA_BASE+12) //0x4001080C 
#define GPIOB_ODR_Addr    (GPIOB_BASE+12) //0x40010C0C 
#define GPIOC_ODR_Addr    (GPIOC_BASE+12) //0x4001100C 
#define GPIOD_ODR_Addr    (GPIOD_BASE+12) //0x4001140C 
#define GPIOE_ODR_Addr    (GPIOE_BASE+12) //0x4001180C 
#define GPIOF_ODR_Addr    (GPIOF_BASE+12) //0x40011A0C    
#define GPIOG_ODR_Addr    (GPIOG_BASE+12) //0x40011E0C    

#define GPIOA_IDR_Addr    (GPIOA_BASE+8) //0x40010808 
#define GPIOB_IDR_Addr    (GPIOB_BASE+8) //0x40010C08 
#define GPIOC_IDR_Addr    (GPIOC_BASE+8) //0x40011008 
#define GPIOD_IDR_Addr    (GPIOD_BASE+8) //0x40011408 
#define GPIOE_IDR_Addr    (GPIOE_BASE+8) //0x40011808 
#define GPIOF_IDR_Addr    (GPIOF_BASE+8) //0x40011A08 
#define GPIOG_IDR_Addr    (GPIOG_BASE+8) //0x40011E08 
 
//IO口操作,只对单一的IO口!
//确保n的值小于16!
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //输出 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //输入 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //输出 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //输入 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //输出 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //输入 

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //输出 
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //输入 

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //输出 
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //输入

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //输出 
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //输入

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //输出 
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //输入


void Stm32_Clock_Init(u8 PLL);  //时钟初始化  
void Sys_Soft_Reset(void);      //系统软复位
void Sys_Standby(void);         //待机模式 	
void MY_NVIC_SetVectorTable(u32 NVIC_VectTab, u32 Offset);//设置偏移地址
void MY_NVIC_PriorityGroupConfig(u8 NVIC_Group);//设置NVIC分组
void MY_NVIC_Init(u8 NVIC_PreemptionPriority,u8 NVIC_SubPriority,u8 NVIC_Channel,u8 NVIC_Group);//设置中断
void Ex_NVIC_Config(u8 GPIOx,u8 BITx,u8 TRIM);//外部中断配置函数(只对GPIOA~G)
//void JTAG_Set(u8 mode);



//以下为汇编函数
void WFI_SET(void);		//执行WFI指令
void INTX_DISABLE(void);//关闭所有中断
void INTX_ENABLE(void);	//开启所有中断
void MSR_MSP(u32 addr);	//设置堆栈地址


extern int Encoder_A,Encoder_B,Encoder_C,Encoder_D;         	//=====编码器的脉冲计数
extern long int Position_A,Position_B,Position_C,Position_D,Rate_A,Rate_B,Rate_C,Rate_D; //=====PID控制相关变量                 
extern int Motor_A,Motor_B,Motor_C,Motor_D;        											//=====电机PWM变量
extern int Target_A,Target_B,Target_C,Target_D;    										  //=====电机目标值                   
extern float Pitch,Roll,Yaw,Move_X,Move_Y,Move_Z;  									  //=====三轴角度和XYZ轴目标速度
extern float	Position_KP,Position_KI,Position_KD; 		  //=====位置控制PID参数
extern float Velocity_KP,Velocity_KI;	         												  //=====速度控制PID参数
extern float Angle_KP,Angle_KI;
extern int RC_Velocity,RC_Position;        										  //=====设置遥控的速度和位置值
extern u8 State;																																							  //=====运行状态标志位
extern int T;																																											  //=====延时计数位
extern u32  Distance;																																						//=====一号超声波距离
extern u32  Distance_1;
extern u32  Distance1;																																					//=====二号超声波距离
extern u32  Distance2;																																					//=====三号超声波距离
extern u8 ReadValuea,ReadValueb;																																			//=====红外对管标志位
extern u8 ReadValuec,ReadValued;																																			//=====红外对管标志位
extern int q;																																										  	//=====超声波状态标志位
extern u8 res;
extern int  LEFT,RIGHT,LEFT1,RIGHT1;
extern u8 Flag,ReadValue,Sensor;
extern int xx,yy;
extern int Flag_Velocity;
extern int Flag_Angle;
extern int run;
extern int Flag_Direction;
extern float Target_Z;
extern uint8_t LobotRxBuf[16];
extern u8  K[3]; //二维码信息
extern char Target[3];


extern	float pitch,roll,yaw; 		//欧拉角
extern	short aacx,aacy,aacz;		//加速度传感器原始数据
extern	short gyrox,gyroy,gyroz;	//陀螺仪原始数据

#include "usart.h"
#include <jansson.h>
#include "oled.h"
#include "delay.h"
#include "sensor.h"
#include "usart.h"
#include "timer.h"
#include "motor.h"
#include "encoder.h"							   
#include "control.h"	
#include "move.h"
#include "SysTick.h"
#include "LobotServoController.h"
#include "INIT.h"
#include "exti.h"	
#include "key.h"
#include "IOI2C.h"
#include "mpu6050.h" 
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "pstwo.h" 
#endif
