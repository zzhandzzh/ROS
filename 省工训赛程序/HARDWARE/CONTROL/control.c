#include "control.h"	


u8 Flag_Target,Flag_Change;                             //相关标志位
u8 temp1;                                               								 //临时变量
float Voltage_Count,Voltage_All; 											   //电压采样相关变量
float Gyro_K=0.004;     				  											         //陀螺仪比例系数

#define a_PARAMETER          (0.095f)               
#define b_PARAMETER          (0.086f)                    
/**************************************************************************
函数功能：小车运动数学模型
入口参数：X Y Z 三轴速度或者位置
返回  值：无
**************************************************************************/

void Kinematic_Analysis(float Vx,float Vy,float Vz)
{
        Target_A   = -Vx-Vy+Vz*(a_PARAMETER+b_PARAMETER);
        Target_B   = -Vx+Vy+Vz*(a_PARAMETER+b_PARAMETER);
	      Target_C   = +Vx-Vy+Vz*(a_PARAMETER+b_PARAMETER);
				Target_D   = +Vx+Vy+Vz*(a_PARAMETER+b_PARAMETER);
}
/**************************************************************************
函数功能：获取位置控制过程速度值
入口参数：X Y Z 三轴位置变化量
返回  值：无
**************************************************************************/
void Kinematic_Analysis2(float Vy,float Vz)
{
	      Rate_A   = Vy-Vz*(a_PARAMETER)*10;
        Rate_B   = Vy-Vz*(a_PARAMETER)*10;
	      Rate_C   = Vy+Vz*(a_PARAMETER)*10;
				Rate_D   = Vy+Vz*(a_PARAMETER)*10;
}
/**************************************************************************
函数功能：所有的控制代码都在这里面
         5ms定时中断由MPU6050的INT引脚触发
         严格保证采样和数据处理的时间同步				 
**************************************************************************/
void Get_RC(void)
{
	float step=0.5;  //设置速度控制步进值。
		 		if((mpu_dmp_get_data(&pitch,&roll,&yaw)==0)&&(Flag_Angle==1))Move_Z=angle_PID( yaw, Target_Z)*10;//角度pid
	
				 switch(Flag_Direction)   //方向控制
				 {
					 //Move_Z=0;	
				 case 0: 	Move_X=0;			      Move_Y=0;							     break;
				 case 1:  Move_X=0;           Move_Y+=step;              break;
				 case 2:  Move_X+=step;       Move_Y+=step;              break;
				 case 3:  Move_X+=step;       Move_Y=0;               	 break;
				 case 4:  Move_X+=step;       Move_Y-=step;              break;
				 case 5:  Move_X=0;           Move_Y-=step;              break;
				 case 6:  Move_X-=step;       Move_Y-=step;              break;
				 case 7:  Move_X-=step;       Move_Y=0;                  break;
				 case 8:  Move_X-=step;       Move_Y+=step;              break;
case 9:  Move_X=0;           Move_Y=0;      Move_Z=-100 ;        break;					 
			 }
				 
				if(Move_X<-RC_Velocity) Move_X=-RC_Velocity;	   //速度控制限幅
				if(Move_X>RC_Velocity)  Move_X=RC_Velocity;	     
				if(Move_Y<-RC_Velocity) Move_Y=-RC_Velocity;	
				if(Move_Y>RC_Velocity)  Move_Y=RC_Velocity;	 
				if(Move_Z<-90) Move_Z=-90;	
				if(Move_Z>90)  Move_Z=90;	 
			 

		 Kinematic_Analysis(Move_X,Move_Y,Move_Z);//得到控制目标值，进行运动学分析
}

void Duty_10ms()
{
	Get_RC();
  motor_pid();	 
}

void Duty_50ms()
{
	//printf("%.2f\r\n",angle1);
			printf ("%f	\t%f\r\n",yaw,Target_Z);
		ReadValuea = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12);	
		ReadValueb = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15);	
		ReadValuec = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13);	
		ReadValued = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14);	
	
}


void motor_pid(void)
{
	if(Flag_Velocity==0)
		{
			Encoder_A=Read_Encoder(2);                                          	//===读取编码器的值
			Encoder_B=Read_Encoder(3);                                          		//===读取编码器的值
			Encoder_C=Read_Encoder(4);                                         		//===读取编码器的值
			Encoder_D=Read_Encoder(5); 			 																			//===读取编码器的值	
			if(RC_Velocity>0&&RC_Velocity<15)  RC_Velocity=15;
				
			//printf("%d  \r\n  ",Encoder_C);
			Motor_A=Incremental_PI_A(Encoder_A,Target_A);         //===速度闭环控制计算电机A最终PWM
			Motor_B=Incremental_PI_B(Encoder_B,Target_B);         //===速度闭环控制计算电机B最终PWM
			Motor_C=Incremental_PI_C(Encoder_C,Target_C);         //===速度闭环控制计算电机C最终PWM
			Motor_D=Incremental_PI_D(Encoder_D,Target_D);         //===速度闭环控制计算电机C最终PWM
				
			Xianfu_Pwm(6900);                        																				  //===PWM限幅				 
			if( Motor_A<0){PWMA=-Motor_A;AF;}
			else {PWMA=Motor_A;AZ;}
			if( Motor_B<0){PWMB=-Motor_B;BF;}
			else {PWMB=Motor_B;BZ;}
			if( Motor_C<0){PWMC=-Motor_C;CF;}
			else {PWMC=Motor_C;CZ;}
			if( Motor_D<0){PWMD=-Motor_D;DF;}
			else {PWMD=Motor_D;DZ;}
		}
		if(Flag_Velocity==1)
		{
			Encoder_A=Read_Encoder(2);                                          	//===读取编码器的值
			Encoder_B=Read_Encoder(3);                                          		//===读取编码器的值
			Encoder_C=Read_Encoder(4);                                         		//===读取编码器的值
			Encoder_D=Read_Encoder(5); 			 																			//===读取编码器的值	
			if(RC_Velocity>0&&RC_Velocity<10)  RC_Velocity=10;
				
			//printf("%d    ",Encoder_C);
			Motor_A=Incremental_PI_A(Encoder_A,Target_A);         //===速度闭环控制计算电机A最终PWM
			Motor_B=Incremental_PI_B(Encoder_B,Target_B);         //===速度闭环控制计算电机B最终PWM
			Motor_C=Incremental_PI_C(Encoder_C,Target_C);         //===速度闭环控制计算电机C最终PWM
			Motor_D=Incremental_PI_D(Encoder_D,Target_D);         //===速度闭环控制计算电机C最终PWM
				
			Xianfu_Pwm(6900);                        																				  //===PWM限幅				 
			if( Motor_A<0){PWMA=-Motor_A;AZ;}
			else {PWMA=Motor_A;AF;}
			if( Motor_B<0){PWMB=-Motor_B;BZ;}
			else {PWMB=Motor_B;BF;}
			if( Motor_C<0){PWMC=-Motor_C;CZ;}
			else {PWMC=Motor_C;CF;}
			if( Motor_D<0){PWMD=-Motor_D;DZ;}
			else {PWMD=Motor_D;DF;}
		}
}


int TIM6_IRQHandler(void) 
{    
if(TIM6->SR&0X0001)																																		//溢出中断
{
	static u8 ms10,ms50;
	TIM6->SR&=~(1<<0);																																	//清除中断标志位 	  				
		ms10++;
		ms50++;
		if(ms10==1)
		{
		ms10=0;
		Duty_10ms();	
		}		
		if(ms50==3)
		{
		ms50=0;
    Duty_50ms();
		}


}
return 0;	 
} 
/**************************************************************************
函数功能：限制PWM赋值 
入口参数：幅值
返回  值：无
**************************************************************************/
void Xianfu_Pwm(int amplitude)
{	
    if(Motor_A<-amplitude) Motor_A=-amplitude;	
		if(Motor_A>amplitude)  Motor_A=amplitude;	
	  if(Motor_B<-amplitude) Motor_B=-amplitude;	
		if(Motor_B>amplitude)  Motor_B=amplitude;		
	  if(Motor_C<-amplitude) Motor_C=-amplitude;	
		if(Motor_C>amplitude)  Motor_C=amplitude;		
	  if(Motor_D<-amplitude) Motor_D=-amplitude;	
	  if(Motor_D>amplitude)  Motor_D=amplitude;		
}
/**************************************************************************
函数功能：位置PID控制过程中速度的设置
入口参数：无、幅值
返回  值：无
**************************************************************************/
void Xianfu_Velocity(int amplitude_A,int amplitude_B,int amplitude_C,int amplitude_D)
{	
    if(Motor_A<-amplitude_A) Motor_A=-amplitude_A;	//位置控制模式中，A电机的运行速度
		if(Motor_A>amplitude_A)  Motor_A=amplitude_A;	  //位置控制模式中，A电机的运行速度
	  if(Motor_B<-amplitude_B) Motor_B=-amplitude_B;	//位置控制模式中，B电机的运行速度
		if(Motor_B>amplitude_B)  Motor_B=amplitude_B;		//位置控制模式中，B电机的运行速度
	  if(Motor_C<-amplitude_C) Motor_C=-amplitude_C;	//位置控制模式中，C电机的运行速度
		if(Motor_C>amplitude_C)  Motor_C=amplitude_C;		//位置控制模式中，C电机的运行速度
	  if(Motor_D<-amplitude_D) Motor_D=-amplitude_D;	//位置控制模式中，D电机的运行速度
		if(Motor_D>amplitude_D)  Motor_D=amplitude_D;		//位置控制模式中，D电机的运行速度
}
/**************************************************************************
函数功能：绝对值函数
入口参数：long int
返回  值：unsigned int
**************************************************************************/
u32 myabs(long int a)
{ 		   
	  u32 temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}
/**************************************************************************
函数功能：增量PI控制器
入口参数：编码器测量值，目标速度
返回  值：电机PWM
根据增量式离散PID公式 
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  以此类推 
pwm代表增量输出
在我们的速度控制闭环系统里面，只使用PI控制
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI_A (int Encoder,int Target)
{ 	
	 static int Bias,Pwm,Last_bias;
	Bias=Encoder-Target;                //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias ;   //增量式PI控制器     
	 if(Pwm>7200)Pwm=7200;
	 if(Pwm<-7200)Pwm=-7200;
	 Last_bias=Bias;	                   //保存上一次偏差 
	 return Pwm;                         //增量输出
}
int Incremental_PI_B (int Encoder,int Target)
{ 	
	 static int Bias,Pwm,Last_bias;
	Bias=Encoder-Target;                //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias ;   //增量式PI控制器     
	 if(Pwm>7200)Pwm=7200;
	 if(Pwm<-7200)Pwm=-7200;
	 Last_bias=Bias;	                   //保存上一次偏差 
	 return Pwm;                         //增量输出
}
int Incremental_PI_C (int Encoder,int Target)
{ 	
	 static int Bias,Pwm,Last_bias;
	 Bias=Encoder-Target;                //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //增量式PI控制器
	 if(Pwm>7200)Pwm=7200;
	 if(Pwm<-7200)Pwm=-7200;
	 Last_bias=Bias;	                   //保存上一次偏差 
	 return Pwm;                         //增量输出
}
int Incremental_PI_D (int Encoder,int Target)
{ 	
	 static int Bias,Pwm,Last_bias;
	 Bias=Encoder-Target;                //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //增量式PI控制器
	 if(Pwm>7200)Pwm=7200;
	 if(Pwm<-7200)Pwm=-7200;
	 Last_bias=Bias;	                   //保存上一次偏差 
	 return Pwm;                         //增量输出
}


//角度PID
int angle_PID(float angle,float Target)//角度pid
{
	 static int Bias,angle_speed,Last_bias;	
	 Bias=angle-Target;                //计算偏差
	 angle_speed+=Angle_KP*(Bias-Last_bias)+Angle_KI*Bias;   //增量式PI控制器
	 if(angle_speed>2)angle_speed=2;
	 if(angle_speed<-2)angle_speed=-2;
	 Last_bias=Bias;	                   //保存上一次偏差 
	 return angle_speed;  
}
/**************************************************************************
函数功能：位置式PID控制器
入口参数：编码器测量位置信息，目标位置
返回  值：电机PWM
根据位置式离散PID公式 
pwm=Kp*e(k)+Ki*∑e(k)+Kd[e（k）-e(k-1)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  
∑e(k)代表e(k)以及之前的偏差的累积和;其中k为1,2,,k;
pwm代表输出
**************************************************************************/
int Position_PID_A (int Encoder,int Target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Encoder-Target;                                  //计算偏差
	 Integral_bias+=Bias;	                                 //求出偏差的积分
	 if(Integral_bias>100000)Integral_bias=100000;
	 if(Integral_bias<-100000)Integral_bias=-100000;
	 Pwm=Position_KP*Bias+Position_KI/100*Integral_bias+Position_KD*(Bias-Last_Bias);       //位置式PID控制器
	 Last_Bias=Bias;                                       //保存上一次偏差 
	 return Pwm;                                           //增量输出
}
int Position_PID_B (int Encoder,int Target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Encoder-Target;                                  //计算偏差
	 Integral_bias+=Bias;	                                 //求出偏差的积分
	 if(Integral_bias>100000)Integral_bias=100000;
	 if(Integral_bias<-100000)Integral_bias=-100000;
	 Pwm=Position_KP*Bias+Position_KI/100*Integral_bias+Position_KD*(Bias-Last_Bias);       //位置式PID控制器
	 Last_Bias=Bias;                                       //保存上一次偏差 
	 return Pwm;                                           //增量输出
}
int Position_PID_C (int Encoder,int Target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Encoder-Target;                                  //计算偏差
	 Integral_bias+=Bias;	                                 //求出偏差的积分
	 if(Integral_bias>100000)Integral_bias=100000;
	 if(Integral_bias<-100000)Integral_bias=-100000;
	 Pwm=Position_KP*Bias+Position_KI/100*Integral_bias+Position_KD*(Bias-Last_Bias);       //位置式PID控制器
	 Last_Bias=Bias;                                       //保存上一次偏差 
	 return Pwm;                                           //增量输出
}
int Position_PID_D (int Encoder,int Target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Encoder-Target;                                  //计算偏差
	 Integral_bias+=Bias;	                                 //求出偏差的积分
	 if(Integral_bias>100000)Integral_bias=100000;
	 if(Integral_bias<-100000)Integral_bias=-100000;
	 Pwm=Position_KP*Bias+Position_KI/100*Integral_bias+Position_KD*(Bias-Last_Bias);       //位置式PID控制器
	 Last_Bias=Bias;                                       //保存上一次偏差 
	 return Pwm;                                           //增量输出
}
/**************************************************************************
函数功能：每个电机位置控制过程速度计算
入口参数：无
返回  值：无
**************************************************************************/
