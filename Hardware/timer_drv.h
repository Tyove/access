#ifndef __timer_drv_H
#define __timer_drv_H

#include "stdint.h"

#define sysTickUptime msTicks
//====

//static



//public
void SysTick_Configuration(void);
void Delay(uint32_t);
void Delay_us(uint32_t nTime);
void Delay_ms(uint32_t nTime);	 //µ¥Î»ms

uint32_t GetSysTime_us(void);

extern volatile uint32_t sysTickUptime;

#endif /* __SYSTICK_H */


