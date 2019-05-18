#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"
  /**************************************************************************
作者：平衡小车之家
我的淘宝小店：http://shop114407458.taobao.com/
**************************************************************************/
#define PI 3.14159265
#define ZHONGZHI 0 
#define DIFFERENCE 100
extern	int Balance_Pwm,Velocity_Pwm,Turn_Pwm;
void Kinematic_Analysis(float Vx,float Vy,float Vz);
void Kinematic_Analysis2(float Vy,float Vz);
void Xianfu_Pwm(int amplitude);
void Xianfu_Velocity(int amplitude_A,int amplitude_B,int amplitude_C,int amplitude_D);
u32 myabs(long int a);
int Incremental_PI_A (int Encoder,int Target);
int Incremental_PI_B (int Encoder,int Target);
int Incremental_PI_C (int Encoder,int Target);
int Incremental_PI_D (int Encoder,int Target);

int Position_PID_A (int Encoder,int Target);
int Position_PID_B (int Encoder,int Target);
int Position_PID_C (int Encoder,int Target);
int Position_PID_D (int Encoder,int Target);
void Get_RC(void);
void Count_Velocity(void);
void CAN_N_Usart_Control(void);
int angle_PID(float angle,float Target);//角度pid
void motor_pid(void);
void Duty_10ms(void);
#endif
