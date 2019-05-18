#include "sys.h"

int Encoder_A,Encoder_B,Encoder_C,Encoder_D;         	//=====编码器的脉冲计数
long int Position_A,Position_B,Position_C,Position_D,Rate_A,Rate_B,Rate_C,Rate_D; //=====PID控制相关变量                 
int Motor_A,Motor_B,Motor_C,Motor_D;        											//=====电机PWM变量
int Target_A,Target_B,Target_C,Target_D;    										  //=====电机目标值                   
float Pitch,Roll,Yaw,Move_X=0,Move_Y=0,Move_Z=0;  									  //=====三轴角度和XYZ轴目标速度
float	Position_KP=20,Position_KI=0,Position_KD=20; 		  //=====位置控制PID参数
float Angle_KP=2,Angle_KI=20;         							//=====速度控制PID参数
float Velocity_KP=7,Velocity_KI=7;	         												  //=====速度控制PID参数
int RC_Velocity=40,RC_Position=10000;        										  //=====设置遥控的速度和位置值
u8 State=0;																																							  //=====运行状态标志位
int T=0;																																											  //=====扫码标志位
int Flag_Velocity=0;																										//启动速度标志位
int Flag_Angle=0;																										//启动位置PID标志位
u8 ReadValuea=0,ReadValuec=0;																																			//=====红外对管标志位
u8 ReadValueb=0,ReadValued=0;																																			//=====红外对管标志位
int q=1;																																										  	//=====超声波状态标志位
u8 res;
int run=0;
int Flag_Direction;																										//方向标志位
//json_t *red,*green,*blue,*code;
//char *outR,*outG,*outB,*outC;
u8  K[3];
int xx=0,yy=0;
int  LEFT=0,RIGHT=0,LEFT1=0,RIGHT1=0;
//u8 ReadValue;                                           //巡线传感器读取变量
//u8 Flag;                                                //传感器状态变量
u8 Sensor;                                              //传感器控制变量
char Target[3];
	float pitch,roll,yaw; 		//欧拉角
	short aacx,aacy,aacz;		//加速度传感器原始数据
	short gyrox,gyroy,gyroz;	//陀螺仪原始数据

float Target_Z;
//int PS2_KEY=9;
int main(void)
{	
		uart_init(72,9600);																																			//=====串口初始化为115200
//	  USART2_Init(9600);
    DelayInit();                 //=====延时初始化
		OLED_Init();
		OLED_Clear();
	
		MiniBalance_PWM_Init(7199,0);   
		Encoder_Init_TIM2();   
		Encoder_Init_TIM3();           																												//=====编码器接口
		Encoder_Init_TIM4();           																												//=====编码器接口
		Encoder_Init_TIM5(); 																																	//=====编码器接口
	
	  TIM6_init();	
		JSON_Init();                  //=====JSON参数初始化
		EXTIX_Init();
		OPT_GPIO_Config();
		DUOJICHUSHIHUA();
//		MPU_Init();					//初始化MPU6050
//	PS2_Init();
//	PS2_SetInit();
		OLED_ShowCHinese(36,0,0);//垂
		OLED_ShowCHinese(54,0,1);//死
		OLED_ShowCHinese(72,0,2);//挣
		OLED_ShowCHinese(90,0,3);//扎
		Flag_Angle=1;//开启角度PID
			while(mpu_dmp_init())
 	{
		printf("MPU6050 Error");
 		DelayMs(200);
	}  
 		printf("MPU6050 ok");
////	
		Target_Z=0;
	while(1)
	{		 
		
		
		Flag_Direction=1;
//		DelayMs(1800);
//		DelayMs(1800);
//		PS2_KEY=PS2_DataKey();
//				 switch(PS2_KEY)   //方向控制
//				 {
//					 //Move_Z=0;	
//				 case 0: 	Flag_Direction=1;							     break;
//				 case 1:  Flag_Direction=2;              break;
//				 case 2:  Flag_Direction=3;              break;
//				 case 3:  Flag_Direction=4;               	 break;
//				 case 4:  Flag_Direction=5;              break;
//				 case 5:  Flag_Direction=6;              break;
//				 case 6:  Flag_Direction=7;              break;
//				 case 7:  Flag_Direction=8;                  break;
//				 case 8:  Flag_Direction=9;              break;
//				 
//			 }
				 DelayMs(20);

//		  move();
		  //Target_Z=0;
	}
//while(1)
//	{		 
//		move();
//	}

}
