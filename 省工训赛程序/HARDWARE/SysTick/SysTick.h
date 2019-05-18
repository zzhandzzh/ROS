#ifndef __SYSTICK_H
#define __SYSTICK_H
#include <sys.h>	 
void SysTick_Init( void );
void TimingDelay_Decrement( void );
void Delay_ms ( __IO u32 nTime );
#endif 
