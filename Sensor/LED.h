/**
  ******************************************************************************
  * Copyright (c) 2018,�����пƺƵ�Ƽ����޹�˾
  * All rights reserved.
  * �ļ����ƣ�LED.h
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
#ifndef __LED_H
#define __LED_H
//�ⲿ�ļ�����
#include "include.h"


//�궨����
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
    StatusToggle = 0,                   //״̬��ȡ��
    StatusOn,                       //״̬������
    StatusOff,                      //״̬��Ϩ��    
    StatusFlash,                    //״̬����˸
}emLEDStatus_t;

typedef enum
{
    PowerOn = 0,                        //��Դ������
    PowerOff,                       //��Դ��Ϩ��  
    PowerToggle,                    //��Դ��ȡ��
    PowerFlash,                     //��Դ����˸ 
}emLEDPower_t;

typedef enum
{
    MotorOn = 0,                        //���������
    MotorOff,                       //�����Ϩ�� 
    MotorFlash,                     //�������˸
    MotorClockwiseFlash,            //null
}emLEDMotor_t;

//���ݽṹ����
typedef struct
{
    uint16_t u16FlashTime;          //��˸��ʱ,1ms�ۼ�һ��
    
    emLEDMotor_t   emLEDMotor;
    emLEDPower_t   emLEDPower;
    emLEDStatus_t  emLEDStatus;
}LedManager_t;


//Extern����
extern LedManager_t g_LedManager;


//��������
void LEDInit(void);
void PollingLED(void);

#endif

/******************* (C) ��Ȩ���� 2018 �����пƺƵ�Ƽ����޹�˾ *******************/
