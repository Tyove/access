/**
  ******************************************************************************
  * Copyright (c) 2018,�����пƺƵ�Ƽ����޹�˾
  * All rights reserved.
  * �ļ����ƣ�remote.h
  * ժ    Ҫ��
  *
  * ��ǰ�汾��V1.0
  * ��    �ߣ������пƺƵ�Ƽ����޹�˾�з��� 
  * ������ڣ�    
  * �޸�˵����
  * 
  *
  * ��ʷ�汾��
  *
  *
  *******************************************************************************/
#ifndef _REMOTE_H
#define _REMOTE_H
//�ⲿ�ļ�����
#include "stdint.h"


//�궨����
#define MAXUNLOCKCNT  100

//���ݽṹ����
typedef struct
{
    uint16_t Channel[16];
}SBusRemote_t;

typedef enum
{
    SBusRoll = 0,
    SBusPitch,
    SBusThr,
    SBusYaw,
    SBusSW3,
    SBusLeft,
    SBusRight,
    SBusSW2,
}SBusChannel_t;

typedef struct
{
    uint16_t Start;        //��ʼλ 0XAAAA
    int16_t ContrlBit;    //У��λ ox01
    int16_t THROTTLE;      
    int16_t PITCH;
    int16_t ROLL;
    int16_t YAW;
    int16_t SW_TWO;       //�����������
    int16_t SW_THREE;      //�Ҳ���������
    int16_t LEFT;          //��ದ��
    int16_t RIGHT;          //�Ҳದ��
    int16_t CaliFlag;      //У׼����
    int16_t Arm_State;      //��������
    uint16_t Stop;         //ֹͣλ 0x5555
}PreRemote_t;

typedef struct
{
    uint16_t roll;
    uint16_t pitch;
    uint16_t thr;
    uint16_t yaw;
    uint16_t AUX1;
    uint16_t AUX2;
    uint16_t AUX3;
    uint16_t AUX4;    
    uint16_t AUX5;
    uint16_t AUX6;
    uint16_t AUX7;        
}Remote_t;

//Extern����
extern Remote_t Remote;             //ң��ͨ��ֵ
extern PreRemote_t PreRemote;

//��������
void AnalyRC(void);
void UpdateFMUToRemote(void);
void RemotePolling(void);
void Remote_init(void);
void RCReceiveHandle(void);
#endif


/******************* (C) ��Ȩ���� 2018 �����пƺƵ�Ƽ����޹�˾ *******************/
