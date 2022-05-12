/*==============================================================================
                         ##### How to use this driver #####
==============================================================================
ң��������������÷�ʽ���£�
1.AnalyRC������NRF���������������ã����Է���ң�������͹��������ݣ�����Ҫ�������ã�
2.RCReceiveHandle�������Դ���ң�������մ�������

*/
//�ⲿ�ļ�����
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
//�궨����



//Extern����
extern uint8_t SBusRxBuff[30];
extern bool InitComplete;

//˽�к�����
void RemoteUnlock(void);    



//˽�б�����
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
  * �������ƣ�AnalyRC
  * ��������������ң��������
  * ��    �룺void
  * ��    ����void
  * ��    �أ�void
  * ��    ע��null    
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
        
        Remote.roll =  ((uint16_t)RC_rxData[4]<<8)  | RC_rxData[5];  //ͨ��1�����
        Remote.pitch = ((uint16_t)RC_rxData[6]<<8)  | RC_rxData[7];  //ͨ��2������
        Remote.thr =   ((uint16_t)RC_rxData[8]<<8)  | RC_rxData[9];   //ͨ��3������
        Remote.yaw =   ((uint16_t)RC_rxData[10]<<8) | RC_rxData[11];   //ͨ��4��ƫ��
        Remote.AUX1 =  ((uint16_t)RC_rxData[12]<<8) | RC_rxData[13];   //ͨ��5  ���Ͻǰ�������ͨ��5,Ҳ����AUX1�����߹���  
        Remote.AUX2 =  ((uint16_t)RC_rxData[14]<<8) | RC_rxData[15];   //ͨ��6  ���Ͻǰ�������ͨ��6,Ҳ����AUX2��һ����ɹ���  
        Remote.AUX3 =  ((uint16_t)RC_rxData[16]<<8) | RC_rxData[17];   //ͨ��7  û���ᵽ���ܵİ��������ڱ��� 
        Remote.AUX4 =  ((uint16_t)RC_rxData[18]<<8) | RC_rxData[19];   //ͨ��8  
        Remote.AUX5 =  ((uint16_t)RC_rxData[20]<<8) | RC_rxData[21];   //ͨ��9  
        Remote.AUX6 =  ((uint16_t)RC_rxData[22]<<8) | RC_rxData[23];   //ͨ��10 
        Remote.AUX7 =  ((uint16_t)RC_rxData[24]<<8) | RC_rxData[25];   //ͨ��11 
        
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
  * �������ƣ�RemoteUnlock
  * ����������ң������������
  * ��    �룺void
  * ��    ����void
  * ��    �أ�void 
  * ��    ע��null    
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
        case WAITING_1://�ȴ�����
            //���������࣬�������->�������->������� ����LED�Ʋ����� ����ɽ���            
            if(Remote.thr < 1100)  //��һ��        
            {             
                 status = WAITING_2;                 
            }
            break;
        case WAITING_2://�ڶ��� 
            if(Remote.thr > 1800)          
            {        
                static uint8_t cnt = 0;
                 cnt++;        
                if(cnt > 5) //��������豣��200ms���ϣ���ֹң�ؿ�����ʼ��δ��ɵĴ�������
                {    
                    cnt = 0;
                    status = WAITING_3;
                }
            }
            break;
        case WAITING_3:
            if(Remote.thr < 1100)  //������        
            {             
                status = WAITING_4;                 
            }
            break;            
        case WAITING_4:    //����ǰ׼��                   
            g_FMUflg.unlock = 1;   //������־λ

            status = PROCESS_31;   //�������
            break;
        case PROCESS_31:    //�������״̬
            if(!g_FMUflg.unlock)                           //�������������ֱ�������ɿ�
            {
                status = EXIT_255;                
            }

            break;
        case EXIT_255: //��������                           
            g_FMUflg.unlock = 0;           //����
            status = WAITING_1;     //���صȴ�����
            break;
        default:
            status = EXIT_255;
            break;
    }
#else

    
    switch(status)
    {
        case WAITING_1:
            
            if(Remote.thr < 1100 && Remote.yaw > 1900)  //��һ��        
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
                g_FMUflg.unlock = 1;//������־λ
                ResetPID();
                IMU_Reset();
                WZ_Fus_Reset();
            }
            break;
        case WAITING_2:

        
            if(Remote.thr < 1100 && Remote.yaw < 1100)  //��һ��        
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
  * �������ƣ�RCReceiveHandle
  * ����������ң�������ݽ��մ�����
  * ��    �룺void
  * ��    ����void
  * ��    �أ�void
  * ��    ע��null
  *    
  *
******************************************************************************/
#include "pos_ctrl.h"
#include "Ano_OF.h"
extern float PIDGroup_desired_yaw_pos_tmp;
void RCReceiveHandle()
{
    const float roll_pitch_ratio = 0.04f;  //ң�ؿ�����̬����

    if(ANO_OF.STATE.of_fus != 0 && g_UAVinfo.UAV_Mode >= Altitude_Hold)
    {
        PIDGroup[emPID_Pitch_Pos].desired = pos_out_trans[0];     //��λ�ÿ������Ϊ���нǶȵ�����ֵ  
        PIDGroup[emPID_Roll_Pos].desired  = pos_out_trans[1];
    }
    else
    {
        PIDGroup[emPID_Pitch_Pos].desired =-(Remote.pitch - 1500)*roll_pitch_ratio;     //��ң��ֵ��Ϊ���нǶȵ�����ֵ  
        PIDGroup[emPID_Roll_Pos].desired  = (Remote.roll - 1500)*roll_pitch_ratio;            
    }
    
    //�˴������޸�Ϊ�ٶȿ�����
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
        {    //����Ϊң�ؿ���ƫ���� +-�Ŵ����� 0.75�������ƫ���ǵ���ת��                            
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

//һ�����
//    if(Remote.AUX6)
//    {
//        if(g_UAVinfo.UAV_Mode == Altitude_Hold)
//        {
//            g_FMUflg.unlock = 1;
//        }
//    }

//�ڰ�����
//    if(Remote.thr < 1300 &&Remote.yaw > 1700 && Remote.pitch < 1300 && Remote.roll < 1300)                         //����ң�����½�����
//    {
//        g_FMUflg.unlock = 0;
//        ResetAlt();
//    }
    
    RemoteUnlock();
}

/******************************************************************************
  * �������ƣ�UpdateFMUToRemote
  * �������������·ɿ����ݵ�ң����
  * ��    �룺void
  * ��    ����void
  * ��    �أ�void 
  * ��    ע���˺����������У�ͨ��ң���������ݴ�����λ��
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

/******************* (C) ��Ȩ���� 2018 �����пƺƵ�Ƽ����޹�˾ *******************/
