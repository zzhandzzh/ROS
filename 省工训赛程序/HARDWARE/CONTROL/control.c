#include "control.h"	


u8 Flag_Target,Flag_Change;                             //��ر�־λ
u8 temp1;                                               								 //��ʱ����
float Voltage_Count,Voltage_All; 											   //��ѹ������ر���
float Gyro_K=0.004;     				  											         //�����Ǳ���ϵ��

#define a_PARAMETER          (0.095f)               
#define b_PARAMETER          (0.086f)                    
/**************************************************************************
�������ܣ�С���˶���ѧģ��
��ڲ�����X Y Z �����ٶȻ���λ��
����  ֵ����
**************************************************************************/

void Kinematic_Analysis(float Vx,float Vy,float Vz)
{
        Target_A   = -Vx-Vy+Vz*(a_PARAMETER+b_PARAMETER);
        Target_B   = -Vx+Vy+Vz*(a_PARAMETER+b_PARAMETER);
	      Target_C   = +Vx-Vy+Vz*(a_PARAMETER+b_PARAMETER);
				Target_D   = +Vx+Vy+Vz*(a_PARAMETER+b_PARAMETER);
}
/**************************************************************************
�������ܣ���ȡλ�ÿ��ƹ����ٶ�ֵ
��ڲ�����X Y Z ����λ�ñ仯��
����  ֵ����
**************************************************************************/
void Kinematic_Analysis2(float Vy,float Vz)
{
	      Rate_A   = Vy-Vz*(a_PARAMETER)*10;
        Rate_B   = Vy-Vz*(a_PARAMETER)*10;
	      Rate_C   = Vy+Vz*(a_PARAMETER)*10;
				Rate_D   = Vy+Vz*(a_PARAMETER)*10;
}
/**************************************************************************
�������ܣ����еĿ��ƴ��붼��������
         5ms��ʱ�ж���MPU6050��INT���Ŵ���
         �ϸ�֤���������ݴ����ʱ��ͬ��				 
**************************************************************************/
void Get_RC(void)
{
	float step=0.5;  //�����ٶȿ��Ʋ���ֵ��
		 		if((mpu_dmp_get_data(&pitch,&roll,&yaw)==0)&&(Flag_Angle==1))Move_Z=angle_PID( yaw, Target_Z)*10;//�Ƕ�pid
	
				 switch(Flag_Direction)   //�������
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
				 
				if(Move_X<-RC_Velocity) Move_X=-RC_Velocity;	   //�ٶȿ����޷�
				if(Move_X>RC_Velocity)  Move_X=RC_Velocity;	     
				if(Move_Y<-RC_Velocity) Move_Y=-RC_Velocity;	
				if(Move_Y>RC_Velocity)  Move_Y=RC_Velocity;	 
				if(Move_Z<-90) Move_Z=-90;	
				if(Move_Z>90)  Move_Z=90;	 
			 

		 Kinematic_Analysis(Move_X,Move_Y,Move_Z);//�õ�����Ŀ��ֵ�������˶�ѧ����
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
			Encoder_A=Read_Encoder(2);                                          	//===��ȡ��������ֵ
			Encoder_B=Read_Encoder(3);                                          		//===��ȡ��������ֵ
			Encoder_C=Read_Encoder(4);                                         		//===��ȡ��������ֵ
			Encoder_D=Read_Encoder(5); 			 																			//===��ȡ��������ֵ	
			if(RC_Velocity>0&&RC_Velocity<15)  RC_Velocity=15;
				
			//printf("%d  \r\n  ",Encoder_C);
			Motor_A=Incremental_PI_A(Encoder_A,Target_A);         //===�ٶȱջ����Ƽ�����A����PWM
			Motor_B=Incremental_PI_B(Encoder_B,Target_B);         //===�ٶȱջ����Ƽ�����B����PWM
			Motor_C=Incremental_PI_C(Encoder_C,Target_C);         //===�ٶȱջ����Ƽ�����C����PWM
			Motor_D=Incremental_PI_D(Encoder_D,Target_D);         //===�ٶȱջ����Ƽ�����C����PWM
				
			Xianfu_Pwm(6900);                        																				  //===PWM�޷�				 
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
			Encoder_A=Read_Encoder(2);                                          	//===��ȡ��������ֵ
			Encoder_B=Read_Encoder(3);                                          		//===��ȡ��������ֵ
			Encoder_C=Read_Encoder(4);                                         		//===��ȡ��������ֵ
			Encoder_D=Read_Encoder(5); 			 																			//===��ȡ��������ֵ	
			if(RC_Velocity>0&&RC_Velocity<10)  RC_Velocity=10;
				
			//printf("%d    ",Encoder_C);
			Motor_A=Incremental_PI_A(Encoder_A,Target_A);         //===�ٶȱջ����Ƽ�����A����PWM
			Motor_B=Incremental_PI_B(Encoder_B,Target_B);         //===�ٶȱջ����Ƽ�����B����PWM
			Motor_C=Incremental_PI_C(Encoder_C,Target_C);         //===�ٶȱջ����Ƽ�����C����PWM
			Motor_D=Incremental_PI_D(Encoder_D,Target_D);         //===�ٶȱջ����Ƽ�����C����PWM
				
			Xianfu_Pwm(6900);                        																				  //===PWM�޷�				 
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
if(TIM6->SR&0X0001)																																		//����ж�
{
	static u8 ms10,ms50;
	TIM6->SR&=~(1<<0);																																	//����жϱ�־λ 	  				
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
�������ܣ�����PWM��ֵ 
��ڲ�������ֵ
����  ֵ����
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
�������ܣ�λ��PID���ƹ������ٶȵ�����
��ڲ������ޡ���ֵ
����  ֵ����
**************************************************************************/
void Xianfu_Velocity(int amplitude_A,int amplitude_B,int amplitude_C,int amplitude_D)
{	
    if(Motor_A<-amplitude_A) Motor_A=-amplitude_A;	//λ�ÿ���ģʽ�У�A����������ٶ�
		if(Motor_A>amplitude_A)  Motor_A=amplitude_A;	  //λ�ÿ���ģʽ�У�A����������ٶ�
	  if(Motor_B<-amplitude_B) Motor_B=-amplitude_B;	//λ�ÿ���ģʽ�У�B����������ٶ�
		if(Motor_B>amplitude_B)  Motor_B=amplitude_B;		//λ�ÿ���ģʽ�У�B����������ٶ�
	  if(Motor_C<-amplitude_C) Motor_C=-amplitude_C;	//λ�ÿ���ģʽ�У�C����������ٶ�
		if(Motor_C>amplitude_C)  Motor_C=amplitude_C;		//λ�ÿ���ģʽ�У�C����������ٶ�
	  if(Motor_D<-amplitude_D) Motor_D=-amplitude_D;	//λ�ÿ���ģʽ�У�D����������ٶ�
		if(Motor_D>amplitude_D)  Motor_D=amplitude_D;		//λ�ÿ���ģʽ�У�D����������ٶ�
}
/**************************************************************************
�������ܣ�����ֵ����
��ڲ�����long int
����  ֵ��unsigned int
**************************************************************************/
u32 myabs(long int a)
{ 		   
	  u32 temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}
/**************************************************************************
�������ܣ�����PI������
��ڲ���������������ֵ��Ŀ���ٶ�
����  ֵ�����PWM
��������ʽ��ɢPID��ʽ 
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)������ƫ�� 
e(k-1)������һ�ε�ƫ��  �Դ����� 
pwm�����������
�����ǵ��ٶȿ��Ʊջ�ϵͳ���棬ֻʹ��PI����
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI_A (int Encoder,int Target)
{ 	
	 static int Bias,Pwm,Last_bias;
	Bias=Encoder-Target;                //����ƫ��
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias ;   //����ʽPI������     
	 if(Pwm>7200)Pwm=7200;
	 if(Pwm<-7200)Pwm=-7200;
	 Last_bias=Bias;	                   //������һ��ƫ�� 
	 return Pwm;                         //�������
}
int Incremental_PI_B (int Encoder,int Target)
{ 	
	 static int Bias,Pwm,Last_bias;
	Bias=Encoder-Target;                //����ƫ��
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias ;   //����ʽPI������     
	 if(Pwm>7200)Pwm=7200;
	 if(Pwm<-7200)Pwm=-7200;
	 Last_bias=Bias;	                   //������һ��ƫ�� 
	 return Pwm;                         //�������
}
int Incremental_PI_C (int Encoder,int Target)
{ 	
	 static int Bias,Pwm,Last_bias;
	 Bias=Encoder-Target;                //����ƫ��
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //����ʽPI������
	 if(Pwm>7200)Pwm=7200;
	 if(Pwm<-7200)Pwm=-7200;
	 Last_bias=Bias;	                   //������һ��ƫ�� 
	 return Pwm;                         //�������
}
int Incremental_PI_D (int Encoder,int Target)
{ 	
	 static int Bias,Pwm,Last_bias;
	 Bias=Encoder-Target;                //����ƫ��
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //����ʽPI������
	 if(Pwm>7200)Pwm=7200;
	 if(Pwm<-7200)Pwm=-7200;
	 Last_bias=Bias;	                   //������һ��ƫ�� 
	 return Pwm;                         //�������
}


//�Ƕ�PID
int angle_PID(float angle,float Target)//�Ƕ�pid
{
	 static int Bias,angle_speed,Last_bias;	
	 Bias=angle-Target;                //����ƫ��
	 angle_speed+=Angle_KP*(Bias-Last_bias)+Angle_KI*Bias;   //����ʽPI������
	 if(angle_speed>2)angle_speed=2;
	 if(angle_speed<-2)angle_speed=-2;
	 Last_bias=Bias;	                   //������һ��ƫ�� 
	 return angle_speed;  
}
/**************************************************************************
�������ܣ�λ��ʽPID������
��ڲ���������������λ����Ϣ��Ŀ��λ��
����  ֵ�����PWM
����λ��ʽ��ɢPID��ʽ 
pwm=Kp*e(k)+Ki*��e(k)+Kd[e��k��-e(k-1)]
e(k)������ƫ�� 
e(k-1)������һ�ε�ƫ��  
��e(k)����e(k)�Լ�֮ǰ��ƫ����ۻ���;����kΪ1,2,,k;
pwm�������
**************************************************************************/
int Position_PID_A (int Encoder,int Target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Encoder-Target;                                  //����ƫ��
	 Integral_bias+=Bias;	                                 //���ƫ��Ļ���
	 if(Integral_bias>100000)Integral_bias=100000;
	 if(Integral_bias<-100000)Integral_bias=-100000;
	 Pwm=Position_KP*Bias+Position_KI/100*Integral_bias+Position_KD*(Bias-Last_Bias);       //λ��ʽPID������
	 Last_Bias=Bias;                                       //������һ��ƫ�� 
	 return Pwm;                                           //�������
}
int Position_PID_B (int Encoder,int Target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Encoder-Target;                                  //����ƫ��
	 Integral_bias+=Bias;	                                 //���ƫ��Ļ���
	 if(Integral_bias>100000)Integral_bias=100000;
	 if(Integral_bias<-100000)Integral_bias=-100000;
	 Pwm=Position_KP*Bias+Position_KI/100*Integral_bias+Position_KD*(Bias-Last_Bias);       //λ��ʽPID������
	 Last_Bias=Bias;                                       //������һ��ƫ�� 
	 return Pwm;                                           //�������
}
int Position_PID_C (int Encoder,int Target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Encoder-Target;                                  //����ƫ��
	 Integral_bias+=Bias;	                                 //���ƫ��Ļ���
	 if(Integral_bias>100000)Integral_bias=100000;
	 if(Integral_bias<-100000)Integral_bias=-100000;
	 Pwm=Position_KP*Bias+Position_KI/100*Integral_bias+Position_KD*(Bias-Last_Bias);       //λ��ʽPID������
	 Last_Bias=Bias;                                       //������һ��ƫ�� 
	 return Pwm;                                           //�������
}
int Position_PID_D (int Encoder,int Target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Encoder-Target;                                  //����ƫ��
	 Integral_bias+=Bias;	                                 //���ƫ��Ļ���
	 if(Integral_bias>100000)Integral_bias=100000;
	 if(Integral_bias<-100000)Integral_bias=-100000;
	 Pwm=Position_KP*Bias+Position_KI/100*Integral_bias+Position_KD*(Bias-Last_Bias);       //λ��ʽPID������
	 Last_Bias=Bias;                                       //������һ��ƫ�� 
	 return Pwm;                                           //�������
}
/**************************************************************************
�������ܣ�ÿ�����λ�ÿ��ƹ����ٶȼ���
��ڲ�������
����  ֵ����
**************************************************************************/
