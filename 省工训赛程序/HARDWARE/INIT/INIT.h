#ifndef __MOVE_H
#define __MOVE_H
#include "sys.h"

void Catch_One(void);//=====抓取一号位置物体
void Catch_Two(void);//=====抓取二号位置物体
void Catch_Three(void);//=====抓取三号位置物体
void Put_Red(void);//=====放入红环
void Put_Green(void);//=====放入绿环
void Put_Blue(void);//=====放入蓝环
void JSON_Init(void);//=====JSON指令初始化
void Printf_Color(void);//=====OpenMv 找圆柱指令
void Printf_Code(void);//=====OpenMv 找二维码指令

#endif
