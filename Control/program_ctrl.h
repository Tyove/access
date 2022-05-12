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
#ifndef __program_ctrl_H
#define __program_ctrl_H
//�ⲿ�ļ�����
#include "include.h"


//�궨����
#define u16 uint16_t
#define s16 int16_t 
#define u8 uint8_t
#define s8 int8_t
#define u32 uint32_t
#define s32 int32_t

//���ݽṹ����


typedef struct
{
    float Distance;
    float Speed;
    uint16_t Cnt;
    uint16_t Exp_Time;
}StandardControlDirction_t;

typedef enum
{
	X = 0,
	Y = 1,
	Z = 2,
    
    NumofVectorDir,
}Vector_t;

typedef struct
{
    StandardControlDirction_t StandardControlDirction[NumofVectorDir];
}StandardControl_t;

typedef enum 
{
	AUTO_TAKE_OFF_NULL = 0,
	AUTO_TAKE_OFF = 1,
	AUTO_TAKE_OFF_FINISH,
	AUTO_LAND,
}_atol_sta_enum;

typedef struct
{
	//sta
	u8 state_ok;
	u8 cmd_state[2];
	//time
	u32 fb_process_t_ms[4];
	u32 exp_process_t_ms[4];
	
	//ctrl
	float ref_dir[2];
	float vel_cmps_ref[NumofVectorDir];
	float vel_cmps_w[NumofVectorDir];
	float vel_cmps_h[NumofVectorDir];
	s16 yaw_pal_dps;
	//
	float auto_takeoff_land_velocity;
}_program_ctrl_st;

//Extern����
extern _program_ctrl_st program_ctrl;
extern StandardControl_t StandardControl;


//��������
//static
void FlyCtrlReset(void);

//public
void Program_Ctrl_DataAnl(u8 *data);

void Program_Ctrl_Task(u8 dT_ms);

void One_Key_Takeoff(void);
void One_Key_Land(void);
void One_Key_Take_off_Land_Ctrl_Task(u8 dT_ms);

void UpdateCMD(uint16_t Distance, uint16_t Speed, uint8_t Cmd);

void StdControl(StandardControl_t *ptrS);
#endif

/******************* (C) ��Ȩ���� 2018 �����пƺƵ�Ƽ����޹�˾ *******************/

