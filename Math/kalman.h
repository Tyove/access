/**
  ******************************************************************************
  * Copyright (c) 2018,北京中科浩电科技有限公司
  * All rights reserved.
  * 文件名称：kalman.h
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

#ifndef _KALMAN_H
#define _KALMAN_H
//外部文件引用



//宏定义区



//数据结构声明
typedef struct 
{
    float LastP;
    float Now_P;
    float out;
    float Kg;
    float Q;
    float R;    
}EKF_Filter_t;


//Extern引用



//函数声明
void KalmanFilter(EKF_Filter_t *ekf, float input);  //一维卡尔曼

#endif

/******************* (C) 版权所有 2018 北京中科浩电科技有限公司 *******************/
