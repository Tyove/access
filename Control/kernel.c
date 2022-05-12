/*==============================================================================
                         ##### How to use this driver #####
==============================================================================
循环核心驱动，使用该函数轮训就绪函数


*/
//外部文件引用
#include "include.h"
#include "communication.h"
#include "gcs.h"
#include "speed_estimator.h"
#include "led.h"
#include "battery.h"
#include "stdio.h"
#include "HARDWARE_uart.h"
#include "SPL06.h"
#include "timer_drv.h"
#include "Ano_OF.h"
#include "pos_ctrl.h"
#include "program_ctrl.h"
#include "ANO_GCS_DT.h"
#include "FollowLine.h"
#include "ANO_DT.h"

//宏定义区
#define InitCheck        if(!InitComplete)\
                        {\
                            P2OUT |= GPIO_PIN1;\
                            P2OUT |= GPIO_PIN2;\
                            P2OUT |= GPIO_PIN0;\
                            P1OUT |= GPIO_PIN0;\
                             return ;\
                        }\

#define KernelRunningCheck  if(!KernelRunning)\
                                return;\

//Extern引用



//私有函数区
void Update(void);
void UpdateUSBQueue(void);


//私有变量区
bool KernelRunning = false;
bool InitComplete = false;
int16_t Angle_Int16[3];
uint8_t Buff[20];
int sum = 0;

/******************************************************************************
  * 函数名称：KernelPolling
  * 函数描述：核心轮询程序
  * 输    入：void
  * 输    出：void
  * 返    回：void 
  * 备    注：1ms运行一次    
  *    
  *
******************************************************************************/
uint32_t test_time[5];
uint32_t test_runtime[3];
uint8_t data[5] = {0xAA,0xBB,0xCC,0xDD,0xEE};
//uint32_t test_systick[255];
//uint8_t test_cnt;
void KernelPolling()
{
    static uint32_t Cnt = 0;

    //初始化检查，如果失败，则所有LED亮起
    InitCheck;

    //核心运行检查
    KernelRunningCheck;

    KernelRunning = false;

    //查询是否有数据需要解析
    ANO_DT_Data_Receive_Anl_Task();
    
    //时间段轮询计数
    Cnt++;
    
    //333Hz,修改到滴答计时器里面
    if (Cnt % 3 == 0)
    {        
        //飞行PID控制器更新
        FlightPidControl(0.003f);
        
        //电机逻辑控制
        MotorControl();
    }
    
    //125Hz
    if (Cnt % 8 == 0)
    { 
        //更新欧拉角
        GetAngle(&g_Attitude);
    }

    //91Hz
    if (Cnt % 11 == 0)
    {
        //光流状态更新任务
        AnoOF_State_Task(11);
    }
    
    //100Hz
    if (Cnt % 10 == 0)
    {        
        //更新巡线任务
        //警告！！！
        //此函数包含自动解锁内容，当运行此函数以后
        //按下LaunchPad板按钮会自动倒计时解锁飞机，当您不清楚解锁流程的时候
        //不要轻易解除注释
        
        //此函数为程控入口函数，当您了解程控机制后，可取消此函数注释
        UpdateCentControl(0.01f);
    }
    
    //50Hz任务
    if (Cnt % 20 == 0)
    {
        //气压计更新
        UpdateSPL06Info();

        //观测传感器数据计算
        WZ_Obs_Calcu(0.02f);
        
        //Z轴数据互补修正
        WZ_Fix_Calcu(0.02f);
        
        //高度控制器
        ALT_Ctrl(0.02f);        
        
        //位置控制器
        POS_Ctrl(0.02f);
    }
    
    //20Hz任务
    if (Cnt % 50 == 0)
    {            
        //一键控制程序
        One_Key_Take_off_Land_Ctrl_Task(50);
        
        //程控任务
        Program_Ctrl_Task(50);
    }

    //LED轮训函数
    PollingLED();
    
    //串口轮训函数
    PollingUSART();
}

/******************* (C) 版权所有 2018 北京中科浩电科技有限公司 *******************/
