#ifndef __ANO_OF_H_
#define __ANO_OF_H_

#include "driverlib.h"
//����Ϊȫ�ֱ������������ļ��У����ñ�h�ļ��������������ļ��з��ʵ����±���
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
	//�ɿع������ݼ��
	uint8_t fc_data_online;
	/*
	STATE :
	0bit: 1-�߶���Ч��0-�߶���Ч
	1bit: 1-������Ч��0-������Ч
	2bit: 1-�߶��ں���Ч��0-�߶��ں���Ч
	3bit: 1-�����ں���Ч��0-�����ں���Ч
	4:0
	7bit: 1
	*/

	_ano_of_sta_st STATE;
	//������Ϣ������QUA
	uint8_t		QUALITY;
	//ԭʼ������Ϣ���������������ģ���ֲ�
	int8_t		DX,DY;
	//�ںϺ�Ĺ�����Ϣ���������������ģ���ֲ�
	int16_t		DX2,DY2,DX2FIX,DY2FIX;
	//ԭʼ�߶���Ϣ���ںϺ�߶���Ϣ
	uint16_t	 ALT, ALT2;
	//ԭʼ����������
	int16_t		 GYR_X, GYR_Y, GYR_Z;
	//�˲�������������
	int16_t		 GYR_X2, GYR_Y2, GYR_Z2;
	//ԭʼ���ٶ�����
	int16_t		 ACC_X, ACC_Y, ACC_Z;
	//�˲�����ٶ�����
	int16_t		 ACC_X2, ACC_Y2, ACC_Z2;
	//ŷ���Ǹ�ʽ����̬����
	float		 ATT_ROL, ATT_PIT, ATT_YAW;
	//��Ԫ����ʽ����̬����
	float		 ATT_S1, ATT_S2, ATT_S3, ATT_S4;	

}_ano_of_st;
extern _ano_of_st ANO_OF;

static void AnoOF_Check(uint8_t *dT_ms);
//����state����
void AnoOF_State_Task(uint8_t dT_ms);
//��������Ҫ�ڴ��ڽ����ж��е��ã�ÿ����һ�ֽ����ݣ����ñ�����һ��
void AnoOF_GetOneByte(uint8_t data);

#endif
