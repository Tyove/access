#include "Ano_OF.h"
//#include "Ano_FcData.h"

/*
OF_STATE :
0bit: 1-�߶���Ч��0-�߶���Ч
1bit: 1-������Ч��0-������Ч
2bit: 1-�߶��ں���Ч��0-�߶��ں���Ч
3bit: 1-�����ں���Ч��0-�����ں���Ч
4:0
7bit: 1
*/
_ano_of_st ANO_OF;

void AnoOF_DataAnl(uint8_t *data_buf,uint8_t num);

#define OF_BUF_NUM 50
static uint8_t _datatemp[OF_BUF_NUM];
static uint8_t _data_cnt = 0;
//static uint8_t anoof_data_ok;

void AnoOF_State_Task(uint8_t dT_ms)
{
//	if(anoof_data_ok)
//	{
//		anoof_data_ok = 0;
//		AnoOF_DataAnl(_datatemp,_data_cnt+5);
//	}
	
	AnoOF_Check(&dT_ms);
}


//AnoANO_OF.GetOneByte�ǳ������ݽ�������������ÿ���յ�һ�ֽڹ������ݣ����ñ�����һ�Σ������������Ǵ����յ�������
//����������α����ã����ս��յ�������һ֡���ݺ󣬻��Զ��������ݽ�������AnoANO_OF.DataAnl
void AnoOF_GetOneByte(uint8_t data)
{
	static uint8_t _data_len = 0;
	static uint8_t state = 0;
	
	if(state==0&&data==0xAA)
	{
		state=1;
		_datatemp[0]=data;
	}
	else if(state==1&&data==0x22)	//Դ��ַ
	{
		state=2;
		_datatemp[1]=data;
	}
	else if(state==2)			//Ŀ�ĵ�ַ
	{
		state=3;
		_datatemp[2]=data;
	}
	else if(state==3)			//������
	{
		state = 4;
		_datatemp[3]=data;
	}
	else if(state==4)			//����
	{
		state = 5;
		_datatemp[4]=data;
		_data_len = data;
		if(_data_len>OF_BUF_NUM-10)
		{
			state = 0;
		}
//		if(_data_len>40)
//		{
//			while(1)
//			{
//				_data_len = 44;
//			}
//		}
		_data_cnt = 0;
	}
	else if(state==5&&_data_len>0)
	{
		_data_len--;
		_datatemp[5+_data_cnt++]=data;
		if(_data_len==0)
			state = 6;
	}
	else if(state==6)
	{
		state = 0;
		_datatemp[5+_data_cnt]=data;
		AnoOF_DataAnl(_datatemp,_data_cnt+6);//
//		anoof_data_ok = 1 ;//
	}
	else
		state = 0;
}

static void AnoOF_State_Get(uint8_t state)
{
	ANO_OF.STATE.alt = (_ano_of_sta_enum)(state & (0x01));
	ANO_OF.STATE.of = (_ano_of_sta_enum)((state & (0x02))>>1);
	ANO_OF.STATE.alt_fus = (_ano_of_sta_enum)((state & (0x04))>>2);
	ANO_OF.STATE.of_fus = (_ano_of_sta_enum)((state & (0x08))>>3);
}

//AnoANO_OF.DataAnlΪ�������ݽ�������������ͨ���������õ�����ģ������ĸ�������
//�������ݵ����壬�������������ģ��ʹ���ֲᣬ����ϸ�Ľ���
static uint8_t of_check_f[2];
static uint16_t of_check_cnt[2] = { 10000,10000 };
static void AnoOF_Check(uint8_t *dT_ms)
{
	for(uint8_t i=0;i<2;i++)
	{
		if(of_check_f[i] == 0 )
		{
			if(of_check_cnt[i]<10000)
			{
				of_check_cnt[i] += *dT_ms;	
			}
		}
		else
		{
			of_check_cnt[i] = 0;
		}
		

		of_check_f[i] = 0;
	}
	
	
	if(of_check_cnt[0] > 1000 || of_check_cnt[1] > 1000)
	{
		ANO_OF.fc_data_online = 0;
		//
		AnoOF_State_Get(0);
	}
	else
	{
		ANO_OF.fc_data_online = 1;
	}
		

	
}


void AnoOF_DataAnl(uint8_t *data_buf,uint8_t num)
{
	uint8_t sum = 0;
	for(uint8_t i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		
	
	if(*(data_buf+3)==0X51)//������Ϣ
	{
		if(*(data_buf+5)==0)//ԭʼ������Ϣ
		{
//			ANO_OF.STATE 	= *(data_buf+6);
			AnoOF_State_Get(*(data_buf+6));
			ANO_OF.DX  		= *(data_buf+7);
			ANO_OF.DY  		= *(data_buf+8);
			ANO_OF.QUALITY  	= *(data_buf+9);
		}
		else if(*(data_buf+5)==1)//�ںϺ������Ϣ
		{
//			ANO_OF.STATE 	= *(data_buf+6);
			AnoOF_State_Get(*(data_buf+6));
			ANO_OF.DX2		= (int16_t)(*(data_buf+7)<<8)|*(data_buf+8) ;
			ANO_OF.DY2		= (int16_t)(*(data_buf+9)<<8)|*(data_buf+10) ;
			ANO_OF.DX2FIX	= (int16_t)(*(data_buf+11)<<8)|*(data_buf+12) ;
			ANO_OF.DY2FIX	= (int16_t)(*(data_buf+13)<<8)|*(data_buf+14) ;
			ANO_OF.QUALITY  	= *(data_buf+19);
			
			of_check_f[0] = 1;
		}
	}
	if(*(data_buf+3)==0X52)//�߶���Ϣ
	{
		if(*(data_buf+5)==0)//ԭʼ�߶���Ϣ
		{
			ANO_OF.ALT = (uint16_t)(*(data_buf+6)<<8)|*(data_buf+7) ;
			of_check_f[1] = 1;
		}
		else if(*(data_buf+5)==1)//�ںϺ�߶���Ϣ
		{
			ANO_OF.ALT2 = (uint16_t)(*(data_buf+6)<<8)|*(data_buf+7) ;
		}
	}
//	if(*(data_buf+2)==0X53)//��������
//	{
//		if(*(data_buf+4)==0)//ԭʼ����
//		{
//			ANO_OF.GYR_X = (int16_t)(*(data_buf+5)<<8)|*(data_buf+6) ;
//			ANO_OF.GYR_Y = (int16_t)(*(data_buf+7)<<8)|*(data_buf+8) ;
//			ANO_OF.GYR_Z = (int16_t)(*(data_buf+9)<<8)|*(data_buf+10) ;
//			ANO_OF.ACC_X = (int16_t)(*(data_buf+11)<<8)|*(data_buf+12) ;
//			ANO_OF.ACC_Y = (int16_t)(*(data_buf+13)<<8)|*(data_buf+14) ;
//			ANO_OF.ACC_Z = (int16_t)(*(data_buf+15)<<8)|*(data_buf+16) ;
//		}
//		else if(*(data_buf+4)==1)//�˲�������
//		{
//			ANO_OF.GYR_X2 = (int16_t)(*(data_buf+5)<<8)|*(data_buf+6) ;
//			ANO_OF.GYR_Y2 = (int16_t)(*(data_buf+7)<<8)|*(data_buf+8) ;
//			ANO_OF.GYR_Z2 = (int16_t)(*(data_buf+9)<<8)|*(data_buf+10) ;
//			ANO_OF.ACC_X2 = (int16_t)(*(data_buf+11)<<8)|*(data_buf+12) ;
//			ANO_OF.ACC_Y2 = (int16_t)(*(data_buf+13)<<8)|*(data_buf+14) ;
//			ANO_OF.ACC_Z2 = (int16_t)(*(data_buf+15)<<8)|*(data_buf+16) ;
//		}
//	}
//	if(*(data_buf+2)==0X54)//��̬��Ϣ
//	{
//		if(*(data_buf+4)==0)//ŷ���Ǹ�ʽ
//		{
//			ANO_OF.ATT_ROL = ((int16_t)(*(data_buf+5)<<8)|*(data_buf+6)) * 0.01 ;
//			ANO_OF.ATT_PIT = ((int16_t)(*(data_buf+7)<<8)|*(data_buf+8)) * 0.01 ;
//			ANO_OF.ATT_YAW = ((int16_t)(*(data_buf+9)<<8)|*(data_buf+10)) * 0.01 ;
//		}
//		else if(*(data_buf+4)==1)//��Ԫ����ʽ
//		{
//			ANO_OF.ATT_S1 = ((int16_t)(*(data_buf+5)<<8)|*(data_buf+6)) * 0.0001 ;
//			ANO_OF.ATT_S2 = ((int16_t)(*(data_buf+7)<<8)|*(data_buf+8)) * 0.0001 ;
//			ANO_OF.ATT_S3 = ((int16_t)(*(data_buf+9)<<8)|*(data_buf+10)) * 0.0001 ;
//			ANO_OF.ATT_S4 = ((int16_t)(*(data_buf+11)<<8)|*(data_buf+12)) * 0.0001 ;
//		}
//	}
}

