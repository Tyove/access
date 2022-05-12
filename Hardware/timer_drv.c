#include "timer_Drv.h"
#include "driverlib.h" //#include "core_cm4.h"
#include "mpu6050.h"
#include "imu.h"
#include "speed_estimator.h"

volatile uint32_t usTicks = 0;
volatile uint32_t msTicks = 0;                      /* counts 1ms timeTicks       */

extern bool KernelRunning;                    
extern bool InitComplete;

void SysTick_Handler(void)
{
    msTicks++;
    KernelRunning = true;
    
    if(InitComplete && msTicks % 3 == 0)
    {
        //��ȡMPU6050��ԭʼ����
        GetMPU6050Data();
        
        //��̬���¡�����Ԫ��
        ATT_Update(&g_MPUManager,&g_Attitude, 0.003f); 
        
        //Z������Ԥ��
        WZ_Est_Calcu(0.003f);
    }
}

void SysTick_Configuration(void)
{
		//
    usTicks = SystemCoreClock / 1000000;
	
	//core_cm4.h
    SysTick_Config(SystemCoreClock / 1000);       /* Systick interrupt each 1ms */
}

uint32_t GetSysTime_us(void) //���λ�ȡ������u32/1000(us),�����β�ֵ���������ȡ
{
    register uint32_t ms;
		uint32_t time_us; //cycle_cnt,
    do
		{
        ms = sysTickUptime;
//        cycle_cnt = SysTick->VAL;
				time_us = ms * 1000 + (usTicks * 1000 - SysTick->VAL) / usTicks;
    } while (ms != sysTickUptime);
    return (time_us);
}

void Delay_us(uint32_t nTime)
{ 
	uint32_t now;
	now = GetSysTime_us();
	while(GetSysTime_us()-now<nTime);
}

void Delay_ms(uint32_t nTime)
{ 
  uint16_t i ;
	for(i=0;i<nTime;i++)
	{
		Delay_us(1000);
	}
}

void Delay(uint32_t nCount)
{
  for(; nCount!= 0;nCount--);
}
