/*==============================================================================
                         ##### How to use this driver #####
==============================================================================
滤波器使用请直接参照函数进行；



*/
//外部文件引用
#include <string.h>
#include "filter.h"
#include <math.h>
#include "myMath.h"


//宏定义区



//Extern引用



//私有函数区



//私有变量区




/******************************************************************************
  * 函数名称：MovMiddle
  * 函数描述：中值滤波函数
  * 输    入：要滤波的数据
  * 输    出：void
  * 返    回：滤波后的数据 
  * 备    注：null    
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
  * 函数名称：AntiPulse_MovingAverage_Filter
  * 函数描述：抗干扰型滑动均值滤波
  * 输    入：MovAverage_t *MovAverage 滑动均值结构体指针
  * 输    出：
  * 返    回： 
  * 备    注：每次采样到一个新数据放入队列，对N个数据进行算术平均运算
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
  * 函数名称：MovingAverage_Filter
  * 函数描述：滑动均值滤波
  * 输    入：MovAverage_t *MovAverage 滑动均值结构体指针
  * 输    出：void
  * 返    回：滤波后结果 
  * 备    注：null
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
  * 函数名称：IIR_I_Filter
  * 函数描述：IIR直接I型滤波器
  * 输    入：InData 为当前数据
               *x     储存未滤波的数据
               *y     储存滤波后的数据
               *b     储存系数b
               *a     储存系数a
               nb     数组*b的长度
               na     数组*a的长度
  * 输    出：void
  * 返    回： 滤波数据
  * 备    注： 函数原型
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
  * 函数名称：LPF_1st
  * 函数描述：一阶滞后滤波
  * 输    入：一节滞后滤波结构体指针
  * 输    出：void
  * 返    回：滤波结果
  * 备    注：    
  *    
  *
******************************************************************************/
float LPF_1_Filter_1(Filter_LPF_1 *LPF_1)
{
    return LPF_1->old_data * (1 - LPF_1->factor) + LPF_1->new_data *  LPF_1->factor;
}

/******************************************************************************
  * 函数名称：LPF_1_Filter_2
  * 函数描述：低通滤波器
  * 输    入：Filter_LPF_1 *LPF_1:一节滞后滤波结构体指针
              float dt:单位运行时间
  * 输    出：void
  * 返    回：滤波结果
  * 备    注：
     一阶离散低通滤波器  type frequent.
     Examples for _filter:
  #define  _filter   7.9577e - 3  // 由 "1 / ( 2 * PI * f_cut )"这个公式计算得来; 
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

/******************* (C) 版权所有 2018 北京中科浩电科技有限公司 *******************/

