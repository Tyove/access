#ifndef __ANO_OF_H_
#define __ANO_OF_H_

#include "driverlib.h"
//以下为全局变量，在其他文件中，引用本h文件，即可在其他文件中访问到以下变量
typedef enum 
{
	unavailable = 0,
	available,
}_ano_of_sta_enum;

typedef struct
{
	_ano_of_sta_enum alt;
	_ano_of_sta_enum of;
	_ano_of_sta_enum alt_fus;
	_ano_of_sta_enum of_fus;
	
} _ano_of_sta_st;


	
typedef struct
{
	//飞控光流数据检测
	uint8_t fc_data_online;
	/*
	STATE :
	0bit: 1-高度有效；0-高度无效
	1bit: 1-光流有效；0-光流无效
	2bit: 1-高度融合有效；0-高度融合无效
	3bit: 1-光流融合有效；0-光流融合无效
	4:0
	7bit: 1
	*/

	_ano_of_sta_st STATE;
	//光流信息质量：QUA
	uint8_t		QUALITY;
	//原始光流信息，具体意义见光流模块手册
	int8_t		DX,DY;
	//融合后的光流信息，具体意义见光流模块手册
	int16_t		DX2,DY2,DX2FIX,DY2FIX;
	//原始高度信息和融合后高度信息
	uint16_t	 ALT, ALT2;
	//原始陀螺仪数据
	int16_t		 GYR_X, GYR_Y, GYR_Z;
	//滤波后陀螺仪数据
	int16_t		 GYR_X2, GYR_Y2, GYR_Z2;
	//原始加速度数据
	int16_t		 ACC_X, ACC_Y, ACC_Z;
	//滤波后加速度数据
	int16_t		 ACC_X2, ACC_Y2, ACC_Z2;
	//欧拉角格式的姿态数据
	float		 ATT_ROL, ATT_PIT, ATT_YAW;
	//四元数格式的姿态数据
	float		 ATT_S1, ATT_S2, ATT_S3, ATT_S4;	

}_ano_of_st;
extern _ano_of_st ANO_OF;

static void AnoOF_Check(uint8_t *dT_ms);
//光流state任务
void AnoOF_State_Task(uint8_t dT_ms);
//本函数需要在串口接收中断中调用，每接收一字节数据，调用本函数一次
void AnoOF_GetOneByte(uint8_t data);

#endif
