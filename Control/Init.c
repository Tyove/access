
/*==============================================================================
                         ##### How to use this driver #####
==============================================================================
Ӳ����ʼ������ʹ��ʱֻ��Ҫ����Hadrware_Init����


*/
//�ⲿ�ļ�����
#include "include.h"
#include "gcs.h"
#include "led.h"
#include "spl06.h"
#include "communication.h"
#include "battery.h"
#include "FollowLine.h"
#include "server.h"

//�궨����
#define HARDWARE_CHECK_LED    LED_STATUS_ON;LED_POWER_ON;Delay_ms(100);\
                              LED_STATUS_OFF;LED_POWER_OFF;Delay_ms(100);\
                              LED_STATUS_ON;LED_POWER_ON;Delay_ms(100);\
                              LED_STATUS_OFF;LED_POWER_OFF;Delay_ms(100);
#ifdef STM32
#define HARDWARE_CHECK        g_MPUManager.Check && \
                              g_SPL06Manager.Check && \
                              g_NRFManager.Check
#endif
//Extern����
extern bool InitComplete;

//˽�к�����
void PID_Init(void); 

//˽�б�����
uint32_t SysTick_count; //ϵͳʱ�����
Queue_t USB_Send_Queue;

/******************************************************************************
  * �������ƣ�Hadrware_Init
  * ������������ʼ������Ӳ���Ͳ���
  * ��    �룺void
  * ��    ����void
  * ��    �أ�void 
  * ��    ע��null    
  *    
  *
******************************************************************************/
 void Hadrware_Init(void)
{    
    
    P2DIR |= GPIO_PIN5;

    
    LEDInit();                      //LED���Ƴ�ʼ��
    MPU6050Init();                  //g_MPUManager��ʼ��
    SPL06Init();                    //SPL06��ʼ��
    PID_Init();                     //PID������ʼ��
    server_init();
    Follow_Init();
    gcs_init();
    Remote_init();
    
    //�ж�MPU6050��SPL06��ʼ���Ƿ�ɹ�
    if(g_MPUManager.Check && device_SPL06.Device.bEnable)
    {
        InitComplete = true;
    }
}

/******************************************************************************
  * �������ƣ�PID_Init
  * ������������ʼ������PID����
  * ��    �룺void
  * ��    ����void
  * ��    �أ�void 
  * ��    ע��null
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


/******************* (C) ��Ȩ���� 2018 �����пƺƵ�Ƽ����޹�˾ *******************/
