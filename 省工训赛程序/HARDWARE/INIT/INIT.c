#include "INIT.h"


json_t *color,*code;   //JSON����
char *outRGB,*outC;
	
//=====JSON������ʼ��
void JSON_Init()
{
	color = json_pack("{si}", "COLOR", 1);
	code = json_pack("{si}", "CODE", 1);

  outRGB = json_dumps(color, JSON_ENCODE_ANY);
	outC = json_dumps(code, JSON_ENCODE_ANY);
}
//=====OpenMv ����ɫ
void Printf_Color()
{
	printf("%s", outRGB);
	
    free(color);
    free(outRGB);
}
//=====OpenMv �Ҷ�ά��ָ��
void Printf_Code()
{
	printf("%s", outC);
	
    free(code);
    free(outC);
}

//=====ץȡһ��λ������
void Catch_One()
{
	moveServo(1, 2000, 1000); //1���ƶ�1�Ŷ����2000λ��
	DelayMs(3000);
	moveServos(2, 800, 2,1200,9,2300); //800����2�Ŷ����1200λ�ã�9�Ŷ����2300λ��
	DelayMs(2000);
	runActionGroup(1, 1); //����1�Ŷ�����1��
	DelayMs(1500);
    stopActionGroup();  //ֹͣ����������
    setActionGroupSpeed(1, 200);//��1�Ŷ����������ٶ�����Ϊ200%
	runActionGroup(1, 1); //����1�Ŷ�����1��
	DelayMs(1000);
	stopActionGroup();  //ֹͣ����������
}
//=====ץȡ����λ������
void Catch_Two()
{
	moveServo(1, 2000, 1000); //1���ƶ�1�Ŷ����2000λ��
	DelayMs(3000);
	moveServos(2, 800, 2,1200,9,2300); //800����2�Ŷ����1200λ�ã�9�Ŷ����2300λ��
	DelayMs(2000);
	runActionGroup(1, 1); //����1�Ŷ�����1��
	DelayMs(1500);
    stopActionGroup();  //ֹͣ����������
    setActionGroupSpeed(1, 200);//��1�Ŷ����������ٶ�����Ϊ200%
	runActionGroup(1, 1); //����1�Ŷ�����1��
	DelayMs(1000);
	stopActionGroup();  //ֹͣ����������
}
//=====ץȡ����λ������
void Catch_Three()
{
	moveServo(1, 2000, 1000); //1���ƶ�1�Ŷ����2000λ��
	DelayMs(3000);
	moveServos(2, 800, 2,1200,9,2300); //800����2�Ŷ����1200λ�ã�9�Ŷ����2300λ��
	DelayMs(2000);
	runActionGroup(1, 1); //����1�Ŷ�����1��
	DelayMs(1500);
    stopActionGroup();  //ֹͣ����������
    setActionGroupSpeed(1, 200);//��1�Ŷ����������ٶ�����Ϊ200%
	runActionGroup(1, 1); //����1�Ŷ�����1��
	DelayMs(1000);
	stopActionGroup();  //ֹͣ����������
}
//=====�ŵ���ɫ����
void Put_Red()
{
	moveServo(1, 2000, 1000); //1���ƶ�1�Ŷ����2000λ��
	DelayMs(3000);
	moveServos(2, 800, 2,1200,9,2300); //800����2�Ŷ����1200λ�ã�9�Ŷ����2300λ��
	DelayMs(2000);
	runActionGroup(1, 1); //����1�Ŷ�����1��
	DelayMs(1500);
    stopActionGroup();  //ֹͣ����������
    setActionGroupSpeed(1, 200);//��1�Ŷ����������ٶ�����Ϊ200%
	runActionGroup(1, 1); //����1�Ŷ�����1��
	DelayMs(1000);
	stopActionGroup();  //ֹͣ����������
}
//=====�ŵ���ɫ����
void Put_Green()
{
	moveServo(1, 2000, 1000); //1���ƶ�1�Ŷ����2000λ��
	DelayMs(3000);
	moveServos(2, 800, 2,1200,9,2300); //800����2�Ŷ����1200λ�ã�9�Ŷ����2300λ��
	DelayMs(2000);
	runActionGroup(1, 1); //����1�Ŷ�����1��
	DelayMs(1500);
    stopActionGroup();  //ֹͣ����������
    setActionGroupSpeed(1, 200);//��1�Ŷ����������ٶ�����Ϊ200%
	runActionGroup(1, 1); //����1�Ŷ�����1��
	DelayMs(1000);
	stopActionGroup();  //ֹͣ����������
}
//=====�ŵ���ɫ����
void Put_Blue()
{
	moveServo(1, 2000, 1000); //1���ƶ�1�Ŷ����2000λ��
	DelayMs(3000);
	moveServos(2, 800, 2,1200,9,2300); //800����2�Ŷ����1200λ�ã�9�Ŷ����2300λ��
	DelayMs(2000);
	runActionGroup(1, 1); //����1�Ŷ�����1��
	DelayMs(1500);
    stopActionGroup();  //ֹͣ����������
    setActionGroupSpeed(1, 200);//��1�Ŷ����������ٶ�����Ϊ200%
	runActionGroup(1, 1); //����1�Ŷ�����1��
	DelayMs(1000);
	stopActionGroup();  //ֹͣ����������
}
