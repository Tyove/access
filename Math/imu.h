/**
  ******************************************************************************
  * Copyright (c) 2018,北京中科浩电科技有限公司
  * All rights reserved.
  * 文件名称：imu.h
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
#ifndef __IMU_H
#define __IMU_H
//外部文件引用
#include "include.h"
#include "mpu6050.h"
//宏定义区



//数据结构声明
typedef struct{
    float roll;
    float pitch;
    float yaw;
}Attitude_t;

typedef struct {  //四元数
  float q0;
  float q1;
  float q2;
  float q3;
} Quaternion;

//Extern引用
extern Attitude_t g_Attitude;    //当前角度姿态值


//函数声明
float GetNormAccz(void);
void ATT_Update(const MPU6050Manager_t *pMpu,Attitude_t *pAngE, float dt) ;
void GetAngle(Attitude_t *pAngE);
void IMU_Reset(void);
#endif

/******************* (C) 版权所有 2018 北京中科浩电科技有限公司 *******************/

