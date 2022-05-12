/**
  ******************************************************************************
  * Copyright (c) 2018,北京中科浩电科技有限公司
  * All rights reserved.
  * 文件名称：SPL06.h
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
#ifndef SPL06_H
#define SPL06_H
//外部文件引用
#include "include.h"

//宏定义区
#define HW_ADR_L                    0x76//SDO LOW
#define HW_ADR                      HW_ADR_L<<1
#define CONTINUOUS_PRESSURE         1
#define CONTINUOUS_TEMPERATURE      2
#define CONTINUOUS_P_AND_T          3
#define PRESSURE_SENSOR             0
#define TEMPERATURE_SENSOR          1
#define OFFSET_COUNT                30

//数据结构声明
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

//Extern引用
extern SPL06Manager_t g_SPL06Manager;

//函数声明
void ResetAlt(void);
void  SPL06_Init(void);
float GetSPL06Temp(void);
float GetSPL06Press(void);
void  UpdateSPL06Info(void);

#define MAX_DEVICE_NAME_LENGTH  15
typedef struct
{
    //函数功能区
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
	SPL06_STANDBY_MODE    = 1,	/*备用模式*/
	SPL06_COMMAND_MODE    = 2,	/*命令模式*/	
	SPL06_BACKGROUND_MODE = 3,	/*后台模式*/
}SPL06_MEAS_MODE;

typedef enum
{
	/*Standby Mode*/
	SPL06_IDLE_OR_STOP_BACKGROUND_MEAS_TYPE = 0,

	/*Command Mode(命令发起一次测量)*/	
	SPL06_PRESSURE_MEAS_TYPE			    = 1,
	SPL06_TEMPERATURE_MEAS_TYPE			    = 2,
	SPL06_NA_1_TYPE 						= 3,
	SPL06_NA_2_TYPE 						= 4,
	
	/*Background Mode(后台自动连续测量)*/
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
    uint8_t ChipId;                  /*产品ID | 版本ID*/
    SPL06Param_t    SPL06Param;
    float SPL06Temperature;
    float SPL06Press;
    int32_t RawPressure;
    int32_t RawTemperature;
    int32_t RawAltitude;            /*原始海拔高度 cm*/
    int32_t i32RawAltitude;
    int32_t Kp;
    int32_t Kt;
//    xQueueHandle Spl06DeviceQueue;
}SPL06_t;

extern SPL06_t device_SPL06;

bool SPL06Init(void);
void SPL06Update(void);


#endif

/******************* (C) 版权所有 2018 北京中科浩电科技有限公司 *******************/
