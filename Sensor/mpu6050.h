/**
  ******************************************************************************
  * Copyright (c) 2018,�����пƺƵ�Ƽ����޹�˾
  * All rights reserved.
  * �ļ����ƣ�mpu6050.h
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
#ifndef __MPU6050_H
#define __MPU6050_H
//�ⲿ�ļ�����
#include "include.h"
#include "i2c.h"
#include "stdbool.h"

//���ݽṹ����
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


//Extern����
extern MPU6050Manager_t g_MPUManager;

//��������
bool MPU6050Init(void);
void GetMPU6050Data(void);
void GetMPU6050Offset(void);

#endif 

/******************* (C) ��Ȩ���� 2018 �����пƺƵ�Ƽ����޹�˾ *******************/
