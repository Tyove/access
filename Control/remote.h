/**
  ******************************************************************************
  * Copyright (c) 2018,北京中科浩电科技有限公司
  * All rights reserved.
  * 文件名称：remote.h
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
#ifndef _REMOTE_H
#define _REMOTE_H
//外部文件引用
#include "stdint.h"


//宏定义区
#define MAXUNLOCKCNT  100

//数据结构声明
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
    uint16_t Start;        //起始位 0XAAAA
    int16_t ContrlBit;    //校验位 ox01
    int16_t THROTTLE;      
    int16_t PITCH;
    int16_t ROLL;
    int16_t YAW;
    int16_t SW_TWO;       //左侧两档开关
    int16_t SW_THREE;      //右侧三档开关
    int16_t LEFT;          //左侧拨盘
    int16_t RIGHT;          //右侧拨盘
    int16_t CaliFlag;      //校准按键
    int16_t Arm_State;      //启动按键
    uint16_t Stop;         //停止位 0x5555
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

//Extern引用
extern Remote_t Remote;             //遥控通道值
extern PreRemote_t PreRemote;

//函数声明
void AnalyRC(void);
void UpdateFMUToRemote(void);
void RemotePolling(void);
void Remote_init(void);
void RCReceiveHandle(void);
#endif


/******************* (C) 版权所有 2018 北京中科浩电科技有限公司 *******************/
