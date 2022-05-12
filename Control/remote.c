/*==============================================================================
                         ##### How to use this driver #####
==============================================================================
遥控器驱动程序调用方式如下：
1.AnalyRC函数由NRF接收驱动函数调用，用以分析遥控器发送过来的数据，不需要单独调用；
2.RCReceiveHandle函数用以处理遥控器接收处理函数；

*/
//外部文件引用
#include "nrf24l01.h"
#include "control.h"
#include <math.h>
#include "myMath.h"
#include "LED.h"
#include "Remote.h"
#include "gcs.h"
#include "communication.h"
#include "pid.h"
#include "spl06.h"
#include "speed_estimator.h"
#include "arm_math.h"
//宏定义区



//Extern引用
extern uint8_t SBusRxBuff[30];
extern bool InitComplete;

//私有函数区
void RemoteUnlock(void);    



//私有变量区
SBusRemote_t SBusRemote;
uint8_t RC_rxData[32];
Remote_t Remote;
PreRemote_t PreRemote;
int OnlineCnt = 0; 

void Remote_init()
{
    for(int  i = 0; i < 8; i++)
    {
        *(&Remote.roll + i) = 1500;
    }
    
    Remote.AUX2 = 1000;
    
    g_UAVinfo.UAV_Mode = AutoMode;
}

/******************************************************************************
  * 函数名称：AnalyRC
  * 函数描述：分析遥控器数据
  * 输    入：void
  * 输    出：void
  * 返    回：void
  * 备    注：null    
  *    
  *
******************************************************************************/
void AnalyRC(void)
{
   if(!InitComplete)
   {
       return ;
   }
    
#ifdef NRF24L01    
    uint8_t CheckSum = 0;
    deQueue(&NRF_Mannager.qRx, RC_rxData,&CheckSum);
#endif
        //
//    test = sizeof(PreRemote_t);
    memcpy((uint8_t*)&PreRemote, RC_rxData, sizeof(PreRemote_t));   
    if(RC_rxData[0]==0xAA && RC_rxData[1]==0xAF)
    {
        g_LedManager.emLEDStatus = StatusFlash;
        
        Remote.roll =  ((uint16_t)RC_rxData[4]<<8)  | RC_rxData[5];  //通道1，横滚
        Remote.pitch = ((uint16_t)RC_rxData[6]<<8)  | RC_rxData[7];  //通道2，俯仰
        Remote.thr =   ((uint16_t)RC_rxData[8]<<8)  | RC_rxData[9];   //通道3，油门
        Remote.yaw =   ((uint16_t)RC_rxData[10]<<8) | RC_rxData[11];   //通道4，偏航
        Remote.AUX1 =  ((uint16_t)RC_rxData[12]<<8) | RC_rxData[13];   //通道5  左上角按键属于通道5,也就是AUX1，定高功能  
        Remote.AUX2 =  ((uint16_t)RC_rxData[14]<<8) | RC_rxData[15];   //通道6  右上角按键属于通道6,也就是AUX2，一键起飞功能  
        Remote.AUX3 =  ((uint16_t)RC_rxData[16]<<8) | RC_rxData[17];   //通道7  没有提到功能的按键都属于备用 
        Remote.AUX4 =  ((uint16_t)RC_rxData[18]<<8) | RC_rxData[19];   //通道8  
        Remote.AUX5 =  ((uint16_t)RC_rxData[20]<<8) | RC_rxData[21];   //通道9  
        Remote.AUX6 =  ((uint16_t)RC_rxData[22]<<8) | RC_rxData[23];   //通道10 
        Remote.AUX7 =  ((uint16_t)RC_rxData[24]<<8) | RC_rxData[25];   //通道11 
        
        RCReceiveHandle();
    }
    
    if((PreRemote.Start == 0xAAAA) && (PreRemote.Stop == 0x5555))
    {
        g_LedManager.emLEDStatus = StatusFlash;
        Remote.roll = PreRemote.ROLL;
        Remote.pitch = PreRemote.PITCH;
        Remote.thr = PreRemote.THROTTLE;
        Remote.yaw =  PreRemote.YAW;
        Remote.AUX1 =  PreRemote.SW_THREE;
        Remote.AUX2 =  PreRemote.SW_TWO;
        Remote.AUX6 = PreRemote.ContrlBit;
        RCReceiveHandle();
    }
    
    if(SBusRxBuff[0] == 0x0F)
    {
        //From:https://blog.csdn.net/qq_31232793/article/details/80244211
        SBusRemote.Channel[0]  = ((SBusRxBuff[1]    |SBusRxBuff[2]<<8)                  & 0x07FF);
        SBusRemote.Channel[1]  = ((SBusRxBuff[2]>>3 |SBusRxBuff[3]<<5)                  & 0x07FF);
        SBusRemote.Channel[2]  = ((SBusRxBuff[3]>>6 |SBusRxBuff[4]<<2 |SBusRxBuff[5]<<10)  & 0x07FF);
        SBusRemote.Channel[3]  = ((SBusRxBuff[5]>>1 |SBusRxBuff[6]<<7)                  & 0x07FF);
        SBusRemote.Channel[4]  = ((SBusRxBuff[6]>>4 |SBusRxBuff[7]<<4)                  & 0x07FF);
        SBusRemote.Channel[5]  = ((SBusRxBuff[7]>>7 |SBusRxBuff[8]<<1 |SBusRxBuff[9]<<9)   & 0x07FF);
        SBusRemote.Channel[6]  = ((SBusRxBuff[9]>>2 |SBusRxBuff[10]<<6)                 & 0x07FF);
        SBusRemote.Channel[7]  = ((SBusRxBuff[10]>>5|SBusRxBuff[11]<<3)                 & 0x07FF);
        SBusRemote.Channel[8]  = ((SBusRxBuff[12]   |SBusRxBuff[13]<<8)                 & 0x07FF);
        SBusRemote.Channel[9]  = ((SBusRxBuff[13]>>3|SBusRxBuff[14]<<5)                 & 0x07FF);
        SBusRemote.Channel[10] = ((SBusRxBuff[14]>>6|SBusRxBuff[15]<<2|SBusRxBuff[16]<<10) & 0x07FF);
        SBusRemote.Channel[11] = ((SBusRxBuff[16]>>1|SBusRxBuff[17]<<7)                 & 0x07FF);
        SBusRemote.Channel[12] = ((SBusRxBuff[17]>>4|SBusRxBuff[18]<<4)                 & 0x07FF);
        SBusRemote.Channel[13] = ((SBusRxBuff[18]>>7|SBusRxBuff[19]<<1|SBusRxBuff[20]<<9)  & 0x07FF);
        SBusRemote.Channel[14] = ((SBusRxBuff[20]>>2|SBusRxBuff[21]<<6)                 & 0x07FF);
        SBusRemote.Channel[15] = ((SBusRxBuff[21]>>5|SBusRxBuff[22]<<3)                 & 0x07FF);
        
        for(int i = 0;i<16;i++)
        {
            SBusRemote.Channel[i] = SBusRemote.Channel[i] * 0.753 + 757;
        }
        
//        if(SBusRemote.Channel[SBusPitch] > 2000 || SBusRemote.Channel[SBusPitch] < 1000 ||
//           SBusRemote.Channel[SBusRoll] > 2000 || SBusRemote.Channel[SBusRoll] < 1000 ||
//           SBusRemote.Channel[SBusYaw] > 2000 || SBusRemote.Channel[SBusYaw] < 1000 ||
//           SBusRemote.Channel[SBusThr] > 2000 || SBusRemote.Channel[SBusThr] < 1000)
//        {
//                    SBusRemote.Channel[SBusYaw] = 1500;
//            return ;
//        }
        Remote.pitch = SBusRemote.Channel[SBusPitch];
        Remote.roll = SBusRemote.Channel[SBusRoll];
        Remote.yaw = SBusRemote.Channel[SBusYaw];
        Remote.thr = SBusRemote.Channel[SBusThr];
        Remote.AUX1 = SBusRemote.Channel[SBusSW3];
        Remote.AUX2 = SBusRemote.Channel[SBusSW2];
        RCReceiveHandle();
    }
    
    OnlineCnt = 50;
}

void RemotePolling()
{
    OnlineCnt--;
    
    if(OnlineCnt <= 0 )
    {
        g_FMUflg.unlock = 0;
    }
}
 
/******************************************************************************
  * 函数名称：RemoteUnlock
  * 函数描述：遥控器解锁操作
  * 输    入：void
  * 输    出：void
  * 返    回：void 
  * 备    注：null    
  *    
  *
******************************************************************************/
void RemoteUnlock(void)
{
    volatile static uint8_t status = WAITING_1;
    static uint8_t cnt = 0;
#ifndef MSP432
    switch(status)
    {
        case WAITING_1://等待解锁
            //解锁三步奏，油门最低->油门最高->油门最低 看到LED灯不闪了 即完成解锁            
            if(Remote.thr < 1100)  //第一步        
            {             
                 status = WAITING_2;                 
            }
            break;
        case WAITING_2://第二步 
            if(Remote.thr > 1800)          
            {        
                static uint8_t cnt = 0;
                 cnt++;        
                if(cnt > 5) //最高油门需保持200ms以上，防止遥控开机初始化未完成的错误数据
                {    
                    cnt = 0;
                    status = WAITING_3;
                }
            }
            break;
        case WAITING_3:
            if(Remote.thr < 1100)  //第三步        
            {             
                status = WAITING_4;                 
            }
            break;            
        case WAITING_4:    //解锁前准备                   
            g_FMUflg.unlock = 1;   //解锁标志位

            status = PROCESS_31;   //进入控制
            break;
        case PROCESS_31:    //进入解锁状态
            if(!g_FMUflg.unlock)                           //其它紧急情况可直接锁定飞控
            {
                status = EXIT_255;                
            }

            break;
        case EXIT_255: //进入锁定                           
            g_FMUflg.unlock = 0;           //锁定
            status = WAITING_1;     //返回等待解锁
            break;
        default:
            status = EXIT_255;
            break;
    }
#else

    
    switch(status)
    {
        case WAITING_1:
            
            if(Remote.thr < 1100 && Remote.yaw > 1900)  //第一步        
            {             
                cnt++;
            }else
            {
                cnt = 0;
            }
            
            if(cnt > MAXUNLOCKCNT)
            {
                cnt = 0;
                status = WAITING_2;
                g_FMUflg.unlock = 1;//解锁标志位
                ResetPID();
                IMU_Reset();
                WZ_Fus_Reset();
            }
            break;
        case WAITING_2:

        
            if(Remote.thr < 1100 && Remote.yaw < 1100)  //第一步        
            {             
                cnt++;
            }else
            {
                cnt = 0;
            }
            
            if(cnt > MAXUNLOCKCNT)
            {
                cnt = 0;
                status = WAITING_1;
                g_FMUflg.unlock = 0;   
            }
            break;
    }
    
    if(Remote.AUX2 <1100)
    {
        //g_FMUflg.unlock = 1;
    }else if(Remote.AUX2 > 1500)
    {
        cnt = 0;
        status = WAITING_1;
        ResetPID();
        IMU_Reset();
        g_FMUflg.unlock = 0;
    }
#endif
}

/******************************************************************************
  * 函数名称：RCReceiveHandle
  * 函数描述：遥控器数据接收处理函数
  * 输    入：void
  * 输    出：void
  * 返    回：void
  * 备    注：null
  *    
  *
******************************************************************************/
#include "pos_ctrl.h"
#include "Ano_OF.h"
extern float PIDGroup_desired_yaw_pos_tmp;
void RCReceiveHandle()
{
    const float roll_pitch_ratio = 0.04f;  //遥控控制姿态的量

    if(ANO_OF.STATE.of_fus != 0 && g_UAVinfo.UAV_Mode >= Altitude_Hold)
    {
        PIDGroup[emPID_Pitch_Pos].desired = pos_out_trans[0];     //将位置控制输出为飞行角度的期望值  
        PIDGroup[emPID_Roll_Pos].desired  = pos_out_trans[1];
    }
    else
    {
        PIDGroup[emPID_Pitch_Pos].desired =-(Remote.pitch - 1500)*roll_pitch_ratio;     //将遥杆值作为飞行角度的期望值  
        PIDGroup[emPID_Roll_Pos].desired  = (Remote.roll - 1500)*roll_pitch_ratio;            
    }
    
    //此处可以修改为速度控制量
    if(g_FMUflg.unlock == 1 && Remote.thr > 1300)
    {
//        if((Remote.yaw - 1500)*roll_pitch_ratio > 15)
//        {
//            PIDGroup[emPID_Yaw_Spd].desired = -10;
//        }else if((Remote.yaw - 1500)*roll_pitch_ratio < -15)
//        {
//            PIDGroup[emPID_Yaw_Spd].desired = 10;
//        }else
//        {
//            PIDGroup[emPID_Yaw_Spd].desired = 0;
//        }
        
        if(Remote.yaw > 1700 )
        {    //以下为遥控控制偏航角 +-号代表方向 0.75代表控制偏航角的旋转量                            
            PIDGroup_desired_yaw_pos_tmp -= 0.1f;
        }
        else if(Remote.yaw <1300)
        {
            PIDGroup_desired_yaw_pos_tmp += 0.1f;
        }
        //
        if(PIDGroup[emPID_Yaw_Pos].desired>=180)
        {
            PIDGroup_desired_yaw_pos_tmp -= 360;
        }
        else if(PIDGroup[emPID_Yaw_Pos].desired<-180)
        {
            PIDGroup_desired_yaw_pos_tmp += 360;
        }
    }
    
    if(Remote.AUX1<1200)
    {
        g_UAVinfo.UAV_Mode = Stabilize_Mode;
    }
    else if(Remote.AUX1<1400)
    {
        g_UAVinfo.UAV_Mode = Altitude_Hold;
    }
    else if(Remote.AUX1<1600)
    {
//        g_UAVinfo.UAV_Mode = AutoLand;
    }
    else
    {
    
    }
    
    if(g_UAVinfo.UAV_Mode == Altitude_Hold)
    {
//        if(Remote.thr > 1800)
//        {
//            PIDGroup[emPID_Height_Pos].desired += 1  ;
//        }else if(Remote.thr < 1200)
//        {
//            PIDGroup[emPID_Height_Pos].desired -= 1;
//        }
    }
    
//    switch(Remote.AUX2)
//    {
//        case 2000:
//            
//            break;
//        case 1000:
//            g_FMUflg.unlock = 0;
//            break;
//    }

//一键起飞
//    if(Remote.AUX6)
//    {
//        if(g_UAVinfo.UAV_Mode == Altitude_Hold)
//        {
//            g_FMUflg.unlock = 1;
//        }
//    }

//内八上锁
//    if(Remote.thr < 1300 &&Remote.yaw > 1700 && Remote.pitch < 1300 && Remote.roll < 1300)                         //油门遥杆左下角锁定
//    {
//        g_FMUflg.unlock = 0;
//        ResetAlt();
//    }
    
    RemoteUnlock();
}

/******************************************************************************
  * 函数名称：UpdateFMUToRemote
  * 函数描述：更新飞控数据到遥控器
  * 输    入：void
  * 输    出：void
  * 返    回：void 
  * 备    注：此函数周期运行，通过遥控器把数据穿给上位机
  *    
  *
******************************************************************************/
void UpdateFMUToRemote()
{
    uint8_t Buff[QUEUE_DATA_MAXLENGTH];
    uint8_t length;

    if(deQueue(&USB_Send_Queue, Buff,&length))
    {
#ifdef NRF24L01    
        NRF_Mannager.Hardware_Mannager->send_buff(Buff);
#endif
    }
}

/******************* (C) 版权所有 2018 北京中科浩电科技有限公司 *******************/
