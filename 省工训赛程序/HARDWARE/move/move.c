#include "move.h"
int i=0;
int j=0;
int S=0;

void move(void)
{
          	if(State==0)
	{
		Sensor=0;
		Flag_Direction=8;
		DelayMs(450);
		runActionGroup(0, 1);
		State++;
	}
	
	
          	if(State==1)
	{
		Sensor=2;
		Flag_Direction=1;
		DelayMs(600);
		State++;
	}

	          if(State==2)
	{
			if(ReadValuea==0&&ReadValueb==0)//同时没线
		{
				DelayMs(10);
				if(ReadValuea==0&&ReadValueb==0)
			{
					Flag_Direction=1;//直走不转
					Move_Z=0;
			}
		}
		if(ReadValuea==1&&ReadValueb==0)//左边检测到线
		{
				DelayMs(10);//延时消抖
				if(ReadValuea==1&&ReadValueb==0)
			{
				Move_Z=11;//左转
			}
		}

		if(ReadValuea==0&&ReadValueb==1)//左边检测到线
		{
				DelayMs(10);//延时消抖
				if(ReadValuea==0&&ReadValueb==1)
			{
				Move_Z=-11;//右转
			}
		}

		if(ReadValuea==1&&ReadValueb==1)//同时有线
		{
				DelayMs(10);
				if(ReadValuea==1&&ReadValueb==1)
			{
				if(yy>=5)//数线是否经过5条
				{	
				DelayMs(200);//延时停车
				Move_X=0;
				Move_Y=0;
				State++;
				Sensor=0;
				yy=0;
				}
				Move_Z=0;
			}
		}
	}	
	
			    if(State==3)//转弯准备读取二维码
	{
		
		Flag_Direction=9;
		DelayMs(680);///
		Flag_Direction=0;
		
		DelayMs(10);
		State++;
	}
	
	//读取二维码
			    if(State==4)
	{
//		if(S==0)
//		{
		Target_Z=-90;
		DelayMs(100);
		Flag_Direction=0;
		Printf_Code();
//			S++;
//		}
//		if(S==1)
//		{
//		   if(T==1)	
//		   {			//读取二维码
//		  	Flag_Direction=1;
//			  DelayMs(200);
					State++;
			
//		   }
//	  }
	}
	//从二维码倒车(斜线)
					if(State==5)
	{
		Flag_Direction=0;
	  DelayMs(10);
		Flag_Direction=4;
		DelayMs(1600);///
		Flag_Direction=8;
	  DelayMs(100);
		Flag_Direction=0;
	  DelayMs(100);
	  Flag_Direction=0;
	  DelayMs(100);///
		State++;
		RIGHT1=0;
		Flag_Velocity=1;
		XUNXIAN(80);///靠近读取颜色
		Flag_Velocity=0;
		Flag_Direction=0;
	  DelayMs(5000);///观察
//		Move_Z=-13;
//		DelayMs(100);
		Move_Z=0;
		runActionGroup(3, 1);
	}
         if(State==6)
	{
//		if(T==1)
//		Printf_Color();
		ZUO();
		RED();
		ZHONG();
		BLUE();
		YOU();
		GREEN();
//		State++;
//		if(T==2)
		State++;
	}
	         if(State==7)
	{
		//ZHUAQU();
		State++;
	}
					if(State==8)
	{
		Flag_Direction=3;
		DelayMs(700);
		Flag_Direction=7;
	  DelayMs(100);
		Flag_Direction=2;
		DelayMs(1400);
		Flag_Direction=8;
	  DelayMs(100);
		State++;
	}
	
					if(State==9)
	{
		Flag_Direction=0;
		//结束
	}
}




void ZHUAQU(void)
{
	static int i=0;
	
	switch(Target[K[i]])
	{	
		case '1':	ZUO();	break;
		case '2':	ZHONG();break;
		case '3':	YOU();break;
	}
	
		switch(K[i])
	{	
		case '1':	RED();break;
		case '2':	GREEN();break;
		case '3':	BLUE();break;
	}
	i++;
	if(i==3)
	{
		State++;
	}
}


void ZUO(void)
{
	RIGHT1=0;
	Flag_Velocity=1;
	XUNXIAN(160);
	Flag_Velocity=0;
// Flag_Direction=1;
//	DelayS(2);
	Flag_Direction=0;
	DelayMs(100);
	//DelayMs(1500);
	Flag_Direction=5;
	DelayMs(100);///
	Flag_Direction=1;//刹车
	DelayMs(100);
	Flag_Direction=0;
	DelayMs(100);
	Flag_Direction=7;
	DelayMs(450);///
	Flag_Direction=3;//刹车
	DelayMs(100);
	Flag_Direction=0;
	DelayMs(100);
	runActionGroup(4,1);
	DelayMs(500);
	Flag_Direction=0;
	DelayMs(100);
	Flag_Direction=3;
	DelayMs(450);///
	Flag_Direction=7;//刹车
	DelayMs(100);
	Flag_Direction=0;
	DelayMs(100);
}
void ZHONG(void)
{
	RIGHT1=0;
	Flag_Velocity=1;
	XUNXIAN(160);
	Flag_Velocity=0;
	//Flag_Direction=1;
	//DelayS(2);
	Flag_Direction=0;
	DelayMs(100);///等待舵机指令时间
	runActionGroup(3,1);
	Flag_Direction=5;
	DelayMs(100);///
	Flag_Direction=1;//刹车
	DelayMs(100);
	Flag_Direction=0;
	DelayMs(100);
	//Flag_Direction=7;
	DelayMs(550);///等待舵机指令时间;
	runActionGroup(4,1);
	DelayMs(500);
	//Flag_Direction=3;
	Flag_Direction=0;
	DelayMs(100);
}
void YOU(void)
{
	RIGHT1=0;
	Flag_Velocity=1;
	XUNXIAN(160);
  Flag_Velocity=0;
//	Flag_Direction=1;
//	//DelayS(2);
//	DelayMs(300);
	Flag_Direction=0;
	DelayMs(100);///等待舵机指令时间
	runActionGroup(3,1);
	DelayMs(100);
	Flag_Direction=0;
	DelayMs(10);
	//Flag_Direction=5;
	DelayMs(100);///
	Flag_Direction=0;
	DelayMs(10);
	Flag_Direction=3;
	DelayMs(450);///
	Flag_Direction=7;//刹车
	DelayMs(100);
	Flag_Direction=0;
	DelayMs(100);
	runActionGroup(4,1);
  DelayMs(500);
	Flag_Direction=7;
	DelayMs(450);///
	Flag_Direction=3;//刹车
	DelayMs(100);
	Flag_Direction=0;
	DelayMs(100);
}
void RED(void)
{
	Flag_Direction=5;
	DelayS(1);
	DelayMs(200);///
	Flag_Direction=3;
	DelayMs(710);///
	Flag_Direction=7;//刹车
	DelayMs(100);
	Flag_Direction=0;
	DelayMs(100);
	runActionGroup(5,1);
	DelayMs(500);
	Flag_Direction=0;
	DelayMs(200);
	Flag_Direction=7;
	DelayMs(710);///
	Flag_Direction=3;//刹车
	DelayMs(100);
	Flag_Direction=0;
	DelayMs(100);
	RIGHT1=0;
	XUNXIAN(60);
	Flag_Direction=0;
	DelayMs(100);
}
void GREEN(void)
{
	Flag_Direction=5;
	DelayS(1);
	DelayMs(200);///
//	Flag_Direction=3;
//	DelayMs(800);
	Flag_Direction=1;//刹车
	DelayMs(100);
	Flag_Direction=0;
	DelayMs(100);
	runActionGroup(5,1);
	DelayMs(500);
	Flag_Direction=0;
	DelayMs(800);///等待舵机指令时间
  RIGHT1=0;
	XUNXIAN(60);
	Flag_Direction=0;
	DelayMs(100);
}
void BLUE(void)
{
	Flag_Direction=5;
	DelayS(1);
	DelayMs(200);///
	Flag_Direction=1;//刹车
	DelayMs(100);
	Flag_Direction=7;
	DelayMs(710);///
	Flag_Direction=3;//刹车
	DelayMs(100);
	Flag_Direction=0;
	DelayMs(100);
	runActionGroup(5,1);
	DelayMs(500);
	Flag_Direction=0;
	DelayMs(800);///等待舵机指令时间
	Flag_Direction=3;
	DelayMs(710);///
	Flag_Direction=7;//刹车
	DelayMs(100);
	RIGHT1=0;
	XUNXIAN(60);
	Flag_Direction=0;
	DelayMs(100);
}
void XUNXIAN(int k)
{
		Sensor=2;
	while(1)
	{
			if(RIGHT1<k)
		 {
				Flag_Direction=1;
		//		DelayMs(1800);
						if(ReadValuea==1&&ReadValueb==0)		//外2灰度传感器左侧检测到黑线
				{
						DelayMs(10);
						if(ReadValuea==1&&ReadValueb==0)
					{
						Move_Z=15;//左转
						LEFT++;
					}
				}
						if(ReadValuea==0&&ReadValueb==1)//外2灰度传感器右侧检测到黑线
				{
						DelayMs(5);
						if(ReadValuea==0&&ReadValueb==1)
					{
						Move_Z=-15;//右转
						RIGHT++;
					}
				}
						if(ReadValuea==0&&ReadValueb==0)//同时没线
				{
						DelayMs(5);
						if(ReadValuea==0&&ReadValueb==0)
					{
								if(ReadValuec==0&&ReadValued==1)//内2灰度传感器左侧检测到黑线
						{
								DelayMs(5);
								if(ReadValuec==0&&ReadValued==1)
							{
								Move_Z=11;//左转
							}
						}
								if(ReadValuec==1&&ReadValued==0)//内2灰度传感器右侧检测到黑线
						{
								DelayMs(5);
								if(ReadValuec==1&&ReadValued==0)
							{
								Move_Z=-11;//右转
							}
						}
								if(ReadValuec==0&&ReadValued==0)//内2灰度传感器没有检测到黑线
						{
								DelayMs(5);
								if(ReadValuec==0&&ReadValued==0)
							{
								Flag_Direction=1;
								Move_Z=0;//直走不转
							}
						}
					}
				}	
						if(ReadValuea==1&&ReadValueb==1)
				{
						DelayMs(5);
						if(ReadValuea==1&&ReadValueb==1)
					{
						Move_Z=0;
					}
				}
				RIGHT1++;
			}
		 else break;
		}
}
//		State++;


