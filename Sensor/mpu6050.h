/**
  ******************************************************************************
  * Copyright (c) 2018,北京中科浩电科技有限公司
  * All rights reserved.
  * 文件名称：mpu6050.h
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
#ifndef __MPU6050_H
#define __MPU6050_H
//外部文件引用
#include "include.h"
#include "i2c.h"
#include "stdbool.h"

//数据结构声明
typedef struct{
    int16_t accX;
    int16_t accY;
    int16_t accZ;
    int16_t gyroX;
    int16_t gyroY;
    int16_t gyroZ;
    
    int16_t Offset[6];
    bool Check;
}MPU6050Manager_t;


//Extern引用
extern MPU6050Manager_t g_MPUManager;

//函数声明
bool MPU6050Init(void);
void GetMPU6050Data(void);
void GetMPU6050Offset(void);

#endif 

/******************* (C) 版权所有 2018 北京中科浩电科技有限公司 *******************/
