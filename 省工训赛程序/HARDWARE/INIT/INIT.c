#include "INIT.h"


json_t *color,*code;   //JSON变量
char *outRGB,*outC;
	
//=====JSON参数初始化
void JSON_Init()
{
	color = json_pack("{si}", "COLOR", 1);
	code = json_pack("{si}", "CODE", 1);

  outRGB = json_dumps(color, JSON_ENCODE_ANY);
	outC = json_dumps(code, JSON_ENCODE_ANY);
}
//=====OpenMv 找颜色
void Printf_Color()
{
	printf("%s", outRGB);
	
    free(color);
    free(outRGB);
}
//=====OpenMv 找二维码指令
void Printf_Code()
{
	printf("%s", outC);
	
    free(code);
    free(outC);
}

//=====抓取一号位置物体
void Catch_One()
{
	moveServo(1, 2000, 1000); //1秒移动1号舵机至2000位置
	DelayMs(3000);
	moveServos(2, 800, 2,1200,9,2300); //800毫秒2号舵机到1200位置，9号舵机到2300位置
	DelayMs(2000);
	runActionGroup(1, 1); //运行1号动作组1次
	DelayMs(1500);
    stopActionGroup();  //停止动作组运行
    setActionGroupSpeed(1, 200);//将1号动作组运行速度设置为200%
	runActionGroup(1, 1); //运行1号动作组1次
	DelayMs(1000);
	stopActionGroup();  //停止动作组运行
}
//=====抓取二号位置物体
void Catch_Two()
{
	moveServo(1, 2000, 1000); //1秒移动1号舵机至2000位置
	DelayMs(3000);
	moveServos(2, 800, 2,1200,9,2300); //800毫秒2号舵机到1200位置，9号舵机到2300位置
	DelayMs(2000);
	runActionGroup(1, 1); //运行1号动作组1次
	DelayMs(1500);
    stopActionGroup();  //停止动作组运行
    setActionGroupSpeed(1, 200);//将1号动作组运行速度设置为200%
	runActionGroup(1, 1); //运行1号动作组1次
	DelayMs(1000);
	stopActionGroup();  //停止动作组运行
}
//=====抓取三号位置物体
void Catch_Three()
{
	moveServo(1, 2000, 1000); //1秒移动1号舵机至2000位置
	DelayMs(3000);
	moveServos(2, 800, 2,1200,9,2300); //800毫秒2号舵机到1200位置，9号舵机到2300位置
	DelayMs(2000);
	runActionGroup(1, 1); //运行1号动作组1次
	DelayMs(1500);
    stopActionGroup();  //停止动作组运行
    setActionGroupSpeed(1, 200);//将1号动作组运行速度设置为200%
	runActionGroup(1, 1); //运行1号动作组1次
	DelayMs(1000);
	stopActionGroup();  //停止动作组运行
}
//=====放到红色环中
void Put_Red()
{
	moveServo(1, 2000, 1000); //1秒移动1号舵机至2000位置
	DelayMs(3000);
	moveServos(2, 800, 2,1200,9,2300); //800毫秒2号舵机到1200位置，9号舵机到2300位置
	DelayMs(2000);
	runActionGroup(1, 1); //运行1号动作组1次
	DelayMs(1500);
    stopActionGroup();  //停止动作组运行
    setActionGroupSpeed(1, 200);//将1号动作组运行速度设置为200%
	runActionGroup(1, 1); //运行1号动作组1次
	DelayMs(1000);
	stopActionGroup();  //停止动作组运行
}
//=====放到绿色环中
void Put_Green()
{
	moveServo(1, 2000, 1000); //1秒移动1号舵机至2000位置
	DelayMs(3000);
	moveServos(2, 800, 2,1200,9,2300); //800毫秒2号舵机到1200位置，9号舵机到2300位置
	DelayMs(2000);
	runActionGroup(1, 1); //运行1号动作组1次
	DelayMs(1500);
    stopActionGroup();  //停止动作组运行
    setActionGroupSpeed(1, 200);//将1号动作组运行速度设置为200%
	runActionGroup(1, 1); //运行1号动作组1次
	DelayMs(1000);
	stopActionGroup();  //停止动作组运行
}
//=====放到蓝色环中
void Put_Blue()
{
	moveServo(1, 2000, 1000); //1秒移动1号舵机至2000位置
	DelayMs(3000);
	moveServos(2, 800, 2,1200,9,2300); //800毫秒2号舵机到1200位置，9号舵机到2300位置
	DelayMs(2000);
	runActionGroup(1, 1); //运行1号动作组1次
	DelayMs(1500);
    stopActionGroup();  //停止动作组运行
    setActionGroupSpeed(1, 200);//将1号动作组运行速度设置为200%
	runActionGroup(1, 1); //运行1号动作组1次
	DelayMs(1000);
	stopActionGroup();  //停止动作组运行
}
