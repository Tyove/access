/**
  ******************************************************************************
  * Copyright (c) 2018,�����пƺƵ�Ƽ����޹�˾
  * All rights reserved.
  * �ļ����ƣ�SPL06.h
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
#ifndef SPL06_H
#define SPL06_H
//�ⲿ�ļ�����
#include "include.h"

//�궨����
#define HW_ADR_L                    0x76//SDO LOW
#define HW_ADR                      HW_ADR_L<<1
#define CONTINUOUS_PRESSURE         1
#define CONTINUOUS_TEMPERATURE      2
#define CONTINUOUS_P_AND_T          3
#define PRESSURE_SENSOR             0
#define TEMPERATURE_SENSOR          1
#define OFFSET_COUNT                30

//���ݽṹ����
typedef struct
{    
    int16_t i16C0;
    int16_t i16C1;
    int32_t i32C00;
    int32_t i32C10;
    int16_t i16C01;
    int16_t i16C11;
    int16_t i16C20;
    int16_t i16C21;
    int16_t i16C30;       
}SPL06Param_t0;

typedef struct
{    
    SPL06Param_t0 Param;
    uint8_t u8Chip_id;
    int32_t i32RawPressure;
    int32_t i32RawTemperature;
    int32_t i32KP;
    int32_t i32KT;
    
    float fGround_Alt;
    float fALT;                  //height above sea level        
    float fRelative_Alt;
    
    float fTemperature;
    float fPressure;
    float fLast_Pressure;
    
    float fOffset;
    bool Check;
}SPL06Manager_t;

//Extern����
extern SPL06Manager_t g_SPL06Manager;

//��������
void ResetAlt(void);
void  SPL06_Init(void);
float GetSPL06Temp(void);
float GetSPL06Press(void);
void  UpdateSPL06Info(void);

#define MAX_DEVICE_NAME_LENGTH  15
typedef struct
{
    //����������
    bool (*FInit)(void);
    void (*FUpdate)(void);
    
    bool bCheck;
    bool bEnable;
    char DeviceName[MAX_DEVICE_NAME_LENGTH];
}Device_t;

typedef enum
{
	SPL06_SENSOR_PRESSURE    = 0,
	SPL06_SENSOR_TEMPERATURE = 1,	
}SPL06_SENSOR_TARG;

typedef enum
{
	SPL06_STANDBY_MODE    = 1,	/*����ģʽ*/
	SPL06_COMMAND_MODE    = 2,	/*����ģʽ*/	
	SPL06_BACKGROUND_MODE = 3,	/*��̨ģʽ*/
}SPL06_MEAS_MODE;

typedef enum
{
	/*Standby Mode*/
	SPL06_IDLE_OR_STOP_BACKGROUND_MEAS_TYPE = 0,

	/*Command Mode(�����һ�β���)*/	
	SPL06_PRESSURE_MEAS_TYPE			    = 1,
	SPL06_TEMPERATURE_MEAS_TYPE			    = 2,
	SPL06_NA_1_TYPE 						= 3,
	SPL06_NA_2_TYPE 						= 4,
	
	/*Background Mode(��̨�Զ���������)*/
	SPL06_CONTINUOUS_PRESSURE_TYPE    	    = 5,
	SPL06_CONTINUOUS_TEMPERATURE_TYPE       = 6,
	SPL06_CONTINUOUS_P_AND_T_TYPE           = 7,
}SPL06_MEAS_TYPE;

typedef struct
{    
    int16_t c0;
    int16_t c1;
    int32_t c00;
    int32_t c10;
    int16_t c01;
    int16_t c11;
    int16_t c20;
    int16_t c21;
    int16_t c30;       
}SPL06Param_t;

typedef struct
{
    Device_t Device;
    uint8_t ChipId;                  /*��ƷID | �汾ID*/
    SPL06Param_t    SPL06Param;
    float SPL06Temperature;
    float SPL06Press;
    int32_t RawPressure;
    int32_t RawTemperature;
    int32_t RawAltitude;            /*ԭʼ���θ߶� cm*/
    int32_t i32RawAltitude;
    int32_t Kp;
    int32_t Kt;
//    xQueueHandle Spl06DeviceQueue;
}SPL06_t;

extern SPL06_t device_SPL06;

bool SPL06Init(void);
void SPL06Update(void);


#endif

/******************* (C) ��Ȩ���� 2018 �����пƺƵ�Ƽ����޹�˾ *******************/
