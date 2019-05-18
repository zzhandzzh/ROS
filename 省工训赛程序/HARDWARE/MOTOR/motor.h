#ifndef __MOTOR_H
#define __MOTOR_H
#include <sys.h>	 

#define PWMA   TIM8->CCR2  
#define PWMB   TIM8->CCR1 

#define PWMC   TIM8->CCR4
#define PWMD   TIM8->CCR3 

#define AZ  	 GPIO_SetBits(GPIOF,GPIO_Pin_3);GPIO_ResetBits(GPIOF,GPIO_Pin_2);
#define AF  	 GPIO_SetBits(GPIOF,GPIO_Pin_2);GPIO_ResetBits(GPIOF,GPIO_Pin_3);

#define BZ  	 GPIO_SetBits(GPIOF,GPIO_Pin_1);GPIO_ResetBits(GPIOF,GPIO_Pin_0);
#define BF  	 GPIO_SetBits(GPIOF,GPIO_Pin_0);GPIO_ResetBits(GPIOF,GPIO_Pin_1);

#define DZ  	 GPIO_SetBits(GPIOF,GPIO_Pin_5);GPIO_ResetBits(GPIOF,GPIO_Pin_4);
#define DF  	 GPIO_SetBits(GPIOF,GPIO_Pin_4);GPIO_ResetBits(GPIOF,GPIO_Pin_5);

#define CZ  	 GPIO_SetBits(GPIOF,GPIO_Pin_7);GPIO_ResetBits(GPIOF,GPIO_Pin_6);
#define CF 		 GPIO_SetBits(GPIOF,GPIO_Pin_6);GPIO_ResetBits(GPIOF,GPIO_Pin_7);

void MiniBalance_PWM_Init(u16 arr,u16 psc);
void MiniBalance_Motor_Init(void);
#endif
