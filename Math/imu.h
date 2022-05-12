/**
  ******************************************************************************
  * Copyright (c) 2018,�����пƺƵ�Ƽ����޹�˾
  * All rights reserved.
  * �ļ����ƣ�imu.h
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
#ifndef __IMU_H
#define __IMU_H
//�ⲿ�ļ�����
#include "include.h"
#include "mpu6050.h"
//�궨����



//���ݽṹ����
typedef struct{
    float roll;
    float pitch;
    float yaw;
}Attitude_t;

typedef struct {  //��Ԫ��
  float q0;
  float q1;
  float q2;
  float q3;
} Quaternion;

//Extern����
extern Attitude_t g_Attitude;    //��ǰ�Ƕ���ֵ̬


//��������
float GetNormAccz(void);
void ATT_Update(const MPU6050Manager_t *pMpu,Attitude_t *pAngE, float dt) ;
void GetAngle(Attitude_t *pAngE);
void IMU_Reset(void);
#endif

/******************* (C) ��Ȩ���� 2018 �����пƺƵ�Ƽ����޹�˾ *******************/

