/*==============================================================================
                         ##### How to use this driver #####
==============================================================================
�˲���ʹ����ֱ�Ӳ��պ������У�



*/
//�ⲿ�ļ�����
#include <string.h>
#include "filter.h"
#include <math.h>
#include "myMath.h"


//�궨����



//Extern����



//˽�к�����



//˽�б�����




/******************************************************************************
  * �������ƣ�MovMiddle
  * ������������ֵ�˲�����
  * ��    �룺Ҫ�˲�������
  * ��    ����void
  * ��    �أ��˲�������� 
  * ��    ע��null    
  *    
  *
******************************************************************************/
int16_t MovMiddle(int16_t input)
{    
    uint8_t i = 0;
    volatile uint8_t j = 0;
    const uint8_t MOV_MIDDLE_NUM = 5;
    static int16_t middle[5] = {0};
    int16_t middle_t[5] = {0};

    for(i = 1; i < MOV_MIDDLE_NUM; i++)
    {
        middle[i - 1] =  middle[i];
    }
    
    middle[MOV_MIDDLE_NUM - 1] = input;
    memcpy(middle_t,middle,MOV_MIDDLE_NUM * sizeof(uint32_t));
    
    for(i = 0; i < MOV_MIDDLE_NUM - 1; i++)
    {
        for(j = i + 1; j < MOV_MIDDLE_NUM; j++)
        {
            if(middle_t[i] > middle_t[j])
            {
                middle_t[i] ^= middle_t[j];
                middle_t[j] ^= middle_t[i];
                middle_t[i] ^= middle_t[j];
            }
        }
    }
    int16_t Result = middle_t[(MOV_MIDDLE_NUM + 1) >> 1];
    
    return Result;
}

/******************************************************************************
  * �������ƣ�AntiPulse_MovingAverage_Filter
  * �����������������ͻ�����ֵ�˲�
  * ��    �룺MovAverage_t *MovAverage ������ֵ�ṹ��ָ��
  * ��    ����
  * ��    �أ� 
  * ��    ע��ÿ�β�����һ�������ݷ�����У���N�����ݽ�������ƽ������
  *    
  *
******************************************************************************/
uint16_t AntiPulse_MovingAverage_Filter(MovAverage_t *MovAverage)
{
    uint8_t i;    
    uint32_t sum = 0;
    uint16_t max = 0;
    uint16_t min = 0xffff;

    MovAverage->average[MovAverage->cnt] = MovAverage->input;    
    MovAverage->cnt++;    
    
    if(MovAverage->cnt == MovAverage->max_cnt)
    {
        MovAverage->cnt = 0;
    }    
    for(i = 0; i < MovAverage->max_cnt; i++)
    {
        if(MovAverage->average[i] > max)
        {
            max = MovAverage->average[i];
        }
        else if(MovAverage->average[i] < min)
        {
            min = MovAverage->average[i];
        }
        
        sum += MovAverage->average[i];
    }
    
    return ((sum - max - min) / (MovAverage->max_cnt - 2));                                    
}

/******************************************************************************
  * �������ƣ�MovingAverage_Filter
  * ����������������ֵ�˲�
  * ��    �룺MovAverage_t *MovAverage ������ֵ�ṹ��ָ��
  * ��    ����void
  * ��    �أ��˲����� 
  * ��    ע��null
  *   
  *
******************************************************************************/
uint16_t MovingAverage_Filter(MovAverage_t *MovAverage)
{
    uint8_t i;    
    uint32_t sum = 0;

    MovAverage->average[MovAverage->cnt] = MovAverage->input;    
    MovAverage->cnt++;  
    
    if(MovAverage->cnt == MovAverage->max_cnt)
    {
        MovAverage->cnt = 0;
    }    
    
    for(i = 0; i < MovAverage->max_cnt; i++)
    {
        sum += MovAverage->average[i];
    }
    
    return (sum / MovAverage->max_cnt);                                    
}

/******************************************************************************
  * �������ƣ�IIR_I_Filter
  * ����������IIRֱ��I���˲���
  * ��    �룺InData Ϊ��ǰ����
               *x     ����δ�˲�������
               *y     �����˲��������
               *b     ����ϵ��b
               *a     ����ϵ��a
               nb     ����*b�ĳ���
               na     ����*a�ĳ���
  * ��    ����void
  * ��    �أ� �˲�����
  * ��    ע�� ����ԭ��
               y(n) = b0 * x(n) + b1 * x(n - 1) + b2 * x(n - 2) - a1 * y(n - 1) - a2 * y(n - 2)
  *    
  *
******************************************************************************/
float IIR_I_Filter(float InputData, float *x, float *y,  const float *b, uint8_t nb, const float *a, uint8_t na)
{
    float z1,z2 = 0;
    int16_t i;

    for(i = nb - 1; i > 0; i--)
    {
        x[i] = x[i - 1];
        y[i] = y[i - 1];
    }

    x[0] = InputData;
    z1 = x[0] * b[0];

    for(i = 1; i < nb; i++)
    {
        z1 += x[i] * b[i];
        z2 += y[i] * a[i];
    }

    y[0] = z1 - z2;
    
    return y[0];
}

/******************************************************************************
  * �������ƣ�LPF_1st
  * ����������һ���ͺ��˲�
  * ��    �룺һ���ͺ��˲��ṹ��ָ��
  * ��    ����void
  * ��    �أ��˲����
  * ��    ע��    
  *    
  *
******************************************************************************/
float LPF_1_Filter_1(Filter_LPF_1 *LPF_1)
{
    return LPF_1->old_data * (1 - LPF_1->factor) + LPF_1->new_data *  LPF_1->factor;
}

/******************************************************************************
  * �������ƣ�LPF_1_Filter_2
  * ������������ͨ�˲���
  * ��    �룺Filter_LPF_1 *LPF_1:һ���ͺ��˲��ṹ��ָ��
              float dt:��λ����ʱ��
  * ��    ����void
  * ��    �أ��˲����
  * ��    ע��
     һ����ɢ��ͨ�˲���  type frequent.
     Examples for _filter:
  #define  _filter   7.9577e - 3  // �� "1 / ( 2 * PI * f_cut )"�����ʽ�������; 
  * f_cut = 10 Hz -> _filter = 15.9155e - 3
  * f_cut = 15 Hz -> _filter = 10.6103e - 3
  * f_cut = 20 Hz -> _filter =  7.9577e - 3
  * f_cut = 25 Hz -> _filter =  6.3662e - 3
  * f_cut = 30 Hz -> _filter =  5.3052e - 3
  *
******************************************************************************/
float LPF_1_Filter_2(Filter_LPF_1 *LPF_1, float dt)
{
     return LPF_1->old_data + (dt /( 1 / ( 2 * PI * LPF_1->factor ) + dt)) * (LPF_1->new_data - LPF_1->old_data);    
}

/******************* (C) ��Ȩ���� 2018 �����пƺƵ�Ƽ����޹�˾ *******************/

