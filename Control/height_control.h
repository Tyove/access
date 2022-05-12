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
#ifndef __HEIGHT_CONTROL_H
#define __HEIGHT_CONTROL_H
//外部文件引用
#include "include.h"


//宏定义区



//数据结构声明
typedef struct
{
    float Z_Speed;
    float Z_Acc;
    float Z_Postion;

    float Alt;
    uint16_t Thr;
}HeightInfo_t;


//Extern引用
extern HeightInfo_t HeightInfo;
extern uint8_t fc_state_take_off;

//函数声明
void UpdateAltInfo(float dt);
float get_z_speed(void);
float get_alt(void);
float UpdateAlt_Spd_Pid(float Setpoint, float Current,float dt);
float UpdateAlt_Acc_Pid(float Setpoint, float Current,float dt);

void ControlAlt(float dt);
//=====================ANOTC=======================
void ALT_Ctrl(float dT_s);
#endif

/******************* (C) 版权所有 2018 北京中科浩电科技有限公司 *******************/
