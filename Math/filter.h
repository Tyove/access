/**
  ******************************************************************************
  * Copyright (c) 2018,�����пƺƵ�Ƽ����޹�˾
  * All rights reserved.
  * �ļ����ƣ�filter.h
  * ժ    Ҫ��
  *
  * �޸�˵����
  * 
  *
  * ��ʷ�汾��
  *
  *
  *******************************************************************************/
#ifndef __Algorithm_filter_H
#define __Algorithm_filter_H
//�ⲿ�ļ�����
#include "include.h"
#include "myMath.h"


//�궨����



//���ݽṹ����
typedef struct{
    float old_data;
    float new_data;
    float factor;
}Filter_LPF_1;

typedef struct {
    uint16_t cnt;
    uint16_t input;
    uint16_t *average;
    uint8_t  max_cnt;
}MovAverage_t;


//Extern����



//��������
float LPF_1_Filter_1(Filter_LPF_1 *LPF_1);
float LPF_1_Filter_2(Filter_LPF_1 *LPF_1 , float dt);
uint16_t AntiPulse_MovingAverage_Filter(MovAverage_t *MovAverage);
uint16_t MovingAverage_Filter(MovAverage_t *MovAverage);
float IIR_I_Filter(float InData, float *x, float *y,  const float *b, uint8_t nb, const float *a, uint8_t na);
void  Moving_Average(float in , float moavarray[] , uint16_t len , uint16_t fil_cnt[2] , float *out);
float Moving_Median(uint8_t item , uint8_t width_num , float in);
#endif /* __Algorithm_filter_H */

/******************* (C) ��Ȩ���� 2018 �����пƺƵ�Ƽ����޹�˾ *******************/
