/**
  ******************************************************************************
  * Copyright (c) 2018,北京中科浩电科技有限公司
  * All rights reserved.
  * 文件名称：LED.h
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
#ifndef __LED_H
#define __LED_H
//外部文件引用
#include "include.h"


//宏定义区
#define M1_ON                       Hardware_LED_MOTOR1_ON()
#define M1_OFF                      Hardware_LED_MOTOR1_OFF()
#define M1_Toggle                   Hardware_LED_MOTOR1_TOGGLE()

#define M2_ON                       Hardware_LED_MOTOR2_ON()
#define M2_OFF                      Hardware_LED_MOTOR2_OFF()
#define M2_Toggle                   Hardware_LED_MOTOR2_TOGGLE()

#define M3_ON                       Hardware_LED_MOTOR3_ON()
#define M3_OFF                      Hardware_LED_MOTOR3_OFF()
#define M3_Toggle                   Hardware_LED_MOTOR3_TOGGLE()

#define M4_ON                       Hardware_LED_MOTOR4_ON()
#define M4_OFF                      Hardware_LED_MOTOR4_OFF()
#define M4_Toggle                   Hardware_LED_MOTOR4_TOGGLE()

#ifdef ZKHD_HAWK_RGB_LED_ENABLE
#define LED_STATUS_OFF              Hardware_LED_Red_OFF()
#define LED_STATUS_ON               Hardware_LED_Red_ON(100)
#define LED_STATUS_TOGGLE           Hardware_LED_Red_TOGGLE()

#define LED_POWER_OFF               Hardware_LED_Green_OFF()
#define LED_POWER_ON                Hardware_LED_Green_ON(100)
#define LED_POWER_TOGGLE            Hardware_LED_Green_TOGGLE()

#define BEEP_ON     P2OUT |= GPIO_PIN5;
#define BEEP_OFF    P2OUT &= ~GPIO_PIN5;

#endif



typedef enum
{
    StatusToggle = 0,                   //状态灯取反
    StatusOn,                       //状态灯亮起
    StatusOff,                      //状态灯熄灭    
    StatusFlash,                    //状态灯闪烁
}emLEDStatus_t;

typedef enum
{
    PowerOn = 0,                        //电源灯亮起
    PowerOff,                       //电源灯熄灭  
    PowerToggle,                    //电源灯取反
    PowerFlash,                     //电源灯闪烁 
}emLEDPower_t;

typedef enum
{
    MotorOn = 0,                        //电机灯亮起
    MotorOff,                       //电机灯熄灭 
    MotorFlash,                     //电机灯闪烁
    MotorClockwiseFlash,            //null
}emLEDMotor_t;

//数据结构声明
typedef struct
{
    uint16_t u16FlashTime;          //闪烁计时,1ms累加一次
    
    emLEDMotor_t   emLEDMotor;
    emLEDPower_t   emLEDPower;
    emLEDStatus_t  emLEDStatus;
}LedManager_t;


//Extern引用
extern LedManager_t g_LedManager;


//函数声明
void LEDInit(void);
void PollingLED(void);

#endif

/******************* (C) 版权所有 2018 北京中科浩电科技有限公司 *******************/
