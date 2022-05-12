/**
  ******************************************************************************
  * Copyright (c) 2018,北京中科浩电科技有限公司
  * All rights reserved.
  * 文件名称：height_control.h
  * 摘    要：
  *
  * 当前版本：V1.0
  * 作    者：北京中科浩电科技有限公司研发部 
  * 完成日期：    
  * 修改说明：
  * 
  *
  * 历史版本：
  *
  *
  *******************************************************************************/
#ifndef __program_ctrl_H
#define __program_ctrl_H
//外部文件引用
#include "include.h"


//宏定义区
#define u16 uint16_t
#define s16 int16_t 
#define u8 uint8_t
#define s8 int8_t
#define u32 uint32_t
#define s32 int32_t

//数据结构声明


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

//Extern引用
extern _program_ctrl_st program_ctrl;
extern StandardControl_t StandardControl;


//函数声明
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

/******************* (C) 版权所有 2018 北京中科浩电科技有限公司 *******************/

