/**
  ******************************************************************************
  * Copyright (c) 2018,�����пƺƵ�Ƽ����޹�˾
  * All rights reserved.
  * �ļ����ƣ�height_control.h
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
#ifndef __HEIGHT_CONTROL_H
#define __HEIGHT_CONTROL_H
//�ⲿ�ļ�����
#include "include.h"


//�궨����



//���ݽṹ����
typedef struct
{
    float Z_Speed;
    float Z_Acc;
    float Z_Postion;

    float Alt;
    uint16_t Thr;
}HeightInfo_t;


//Extern����
extern HeightInfo_t HeightInfo;
extern uint8_t fc_state_take_off;

//��������
void UpdateAltInfo(float dt);
float get_z_speed(void);
float get_alt(void);
float UpdateAlt_Spd_Pid(float Setpoint, float Current,float dt);
float UpdateAlt_Acc_Pid(float Setpoint, float Current,float dt);

void ControlAlt(float dt);
//=====================ANOTC=======================
void ALT_Ctrl(float dT_s);
#endif

/******************* (C) ��Ȩ���� 2018 �����пƺƵ�Ƽ����޹�˾ *******************/
