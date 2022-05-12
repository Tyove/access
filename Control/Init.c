
/*==============================================================================
                         ##### How to use this driver #####
==============================================================================
硬件初始化驱动使用时只需要调用Hadrware_Init即可


*/
//外部文件引用
#include "include.h"
#include "gcs.h"
#include "led.h"
#include "spl06.h"
#include "communication.h"
#include "battery.h"
#include "FollowLine.h"
#include "server.h"

//宏定义区
#define HARDWARE_CHECK_LED    LED_STATUS_ON;LED_POWER_ON;Delay_ms(100);\
                              LED_STATUS_OFF;LED_POWER_OFF;Delay_ms(100);\
                              LED_STATUS_ON;LED_POWER_ON;Delay_ms(100);\
                              LED_STATUS_OFF;LED_POWER_OFF;Delay_ms(100);
#ifdef STM32
#define HARDWARE_CHECK        g_MPUManager.Check && \
                              g_SPL06Manager.Check && \
                              g_NRFManager.Check
#endif
//Extern引用
extern bool InitComplete;

//私有函数区
void PID_Init(void); 

//私有变量区
uint32_t SysTick_count; //系统时间计数
Queue_t USB_Send_Queue;

/******************************************************************************
  * 函数名称：Hadrware_Init
  * 函数描述：初始化所有硬件和参数
  * 输    入：void
  * 输    出：void
  * 返    回：void 
  * 备    注：null    
  *    
  *
******************************************************************************/
 void Hadrware_Init(void)
{    
    
    P2DIR |= GPIO_PIN5;

    
    LEDInit();                      //LED闪灯初始化
    MPU6050Init();                  //g_MPUManager初始化
    SPL06Init();                    //SPL06初始化
    PID_Init();                     //PID参数初始化
    server_init();
    Follow_Init();
    gcs_init();
    Remote_init();
    
    //判断MPU6050和SPL06初始化是否成功
    if(g_MPUManager.Check && device_SPL06.Device.bEnable)
    {
        InitComplete = true;
    }
}

/******************************************************************************
  * 函数名称：PID_Init
  * 函数描述：初始化所有PID参数
  * 输    入：void
  * 输    出：void
  * 返    回：void 
  * 备    注：null
  *
  *
******************************************************************************/
void PID_Init(void)
{
    PIDGroup[emPID_Pitch_Pos].kp   = 8.0f;
    PIDGroup[emPID_Pitch_Spd].kp   = 1.1f;
    PIDGroup[emPID_Pitch_Spd].ki   = 1.1f;
    PIDGroup[emPID_Pitch_Spd].kd   = 0.06f;
    PIDGroup[emPID_Pitch_Spd].IntegLimitHigh = 10;
    PIDGroup[emPID_Pitch_Spd].IntegLimitLow = -10;

    PIDGroup[emPID_Roll_Pos].kp    = 8.0f;
    PIDGroup[emPID_Roll_Spd].kp    = 1.1f;
    PIDGroup[emPID_Roll_Spd].ki    = 1.0f;
    PIDGroup[emPID_Roll_Spd].kd    = 0.04f;
    PIDGroup[emPID_Roll_Spd].IntegLimitHigh = 10;
    PIDGroup[emPID_Roll_Spd].IntegLimitLow = -10;

    PIDGroup[emPID_Yaw_Pos].kp     = 8.0f;
    PIDGroup[emPID_Yaw_Spd].kp     = 2.0f;
    PIDGroup[emPID_Yaw_Spd].kd     = 0.00f;
    PIDGroup[emPID_Yaw_Spd].OutLimitHigh = 100;
    PIDGroup[emPID_Yaw_Spd].OutLimitLow = -100;

    PIDGroup[emPID_Roll_Spd].IntegLimitHigh = 50; 
    PIDGroup[emPID_Roll_Spd].IntegLimitLow = -50;
}


/******************* (C) 版权所有 2018 北京中科浩电科技有限公司 *******************/
