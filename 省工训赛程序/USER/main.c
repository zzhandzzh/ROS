#include "sys.h"

int Encoder_A,Encoder_B,Encoder_C,Encoder_D;         	//=====���������������
long int Position_A,Position_B,Position_C,Position_D,Rate_A,Rate_B,Rate_C,Rate_D; //=====PID������ر���                 
int Motor_A,Motor_B,Motor_C,Motor_D;        											//=====���PWM����
int Target_A,Target_B,Target_C,Target_D;    										  //=====���Ŀ��ֵ                   
float Pitch,Roll,Yaw,Move_X=0,Move_Y=0,Move_Z=0;  									  //=====����ǶȺ�XYZ��Ŀ���ٶ�
float	Position_KP=20,Position_KI=0,Position_KD=20; 		  //=====λ�ÿ���PID����
float Angle_KP=2,Angle_KI=20;         							//=====�ٶȿ���PID����
float Velocity_KP=7,Velocity_KI=7;	         												  //=====�ٶȿ���PID����
int RC_Velocity=40,RC_Position=10000;        										  //=====����ң�ص��ٶȺ�λ��ֵ
u8 State=0;																																							  //=====����״̬��־λ
int T=0;																																											  //=====ɨ���־λ
int Flag_Velocity=0;																										//�����ٶȱ�־λ
int Flag_Angle=0;																										//����λ��PID��־λ
u8 ReadValuea=0,ReadValuec=0;																																			//=====����Թܱ�־λ
u8 ReadValueb=0,ReadValued=0;																																			//=====����Թܱ�־λ
int q=1;																																										  	//=====������״̬��־λ
u8 res;
int run=0;
int Flag_Direction;																										//�����־λ
//json_t *red,*green,*blue,*code;
//char *outR,*outG,*outB,*outC;
u8  K[3];
int xx=0,yy=0;
int  LEFT=0,RIGHT=0,LEFT1=0,RIGHT1=0;
//u8 ReadValue;                                           //Ѳ�ߴ�������ȡ����
//u8 Flag;                                                //������״̬����
u8 Sensor;                                              //���������Ʊ���
char Target[3];
	float pitch,roll,yaw; 		//ŷ����
	short aacx,aacy,aacz;		//���ٶȴ�����ԭʼ����
	short gyrox,gyroy,gyroz;	//������ԭʼ����

float Target_Z;
//int PS2_KEY=9;
int main(void)
{	
		uart_init(72,9600);																																			//=====���ڳ�ʼ��Ϊ115200
//	  USART2_Init(9600);
    DelayInit();                 //=====��ʱ��ʼ��
		OLED_Init();
		OLED_Clear();
	
		MiniBalance_PWM_Init(7199,0);   
		Encoder_Init_TIM2();   
		Encoder_Init_TIM3();           																												//=====�������ӿ�
		Encoder_Init_TIM4();           																												//=====�������ӿ�
		Encoder_Init_TIM5(); 																																	//=====�������ӿ�
	
	  TIM6_init();	
		JSON_Init();                  //=====JSON������ʼ��
		EXTIX_Init();
		OPT_GPIO_Config();
		DUOJICHUSHIHUA();
//		MPU_Init();					//��ʼ��MPU6050
//	PS2_Init();
//	PS2_SetInit();
		OLED_ShowCHinese(36,0,0);//��
		OLED_ShowCHinese(54,0,1);//��
		OLED_ShowCHinese(72,0,2);//��
		OLED_ShowCHinese(90,0,3);//��
		Flag_Angle=1;//�����Ƕ�PID
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
//				 switch(PS2_KEY)   //�������
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
