/*==============================================================================
                         ##### How to use this driver #####
==============================================================================


*/
//�ⲿ�ļ�����
#include "speed_estimator.h"
#include "height_control.h"
#include "SPL06.h"
#include "control.h"
#include "gcs.h"
#include "myMath.h"


//�궨����
#define VELOCITY_LIMIT        (130.f)    /*�ٶ��޷� ��λcm/s*/

//Extern����


//˽�к�����
float applyDeadbandf(float value, float deadband);

//˽�б�����
	//float Err = 0.f;            /*λ�����*/
	//float wBaro = 0.35f;            /*��ѹУ��Ȩ��*/
	//float HeightLPF = 0.f;    /*�ںϸ߶ȣ���ͨ*/
	//float rangeLpf = 0.f;
	//float accZLpf = 0.f;            /*Z����ٶȵ�ͨ*/

	///******************************************************************************
	//  * �������ƣ�UpdateAltSpeed
	//  * ����������Z���ٶȹ���
	//  * ��    �룺float dt����λ����ʱ��
	//  * ��    ����void
	//  * ��    �أ�void 
	//  * ��    ע��null  
	//  *
	//  *
	//******************************************************************************/
	//void UpdateAltSpeed(float dt)
	//{
	//    float ewdt = 0;
	//    float weight = wBaro;
	//    
	//    HeightInfo.Alt = HeightInfo.Alt;    
	//    HeightLPF += (HeightInfo.Alt - HeightLPF) * 0.1f;    
	//    
	//    bool isKeyFlightLand = (g_UAVinfo.UAV_Mode == Altitude_Hold);    
	//    
	//    float accZRemovalDead = applyDeadbandf(HeightInfo.Z_Acc, 4);/*ȥ��������Z����ٶ�*/
	//    accZLpf += (accZRemovalDead - accZLpf) * 0.1f;        /*��ͨ*/
	//    
	//    if(isKeyFlightLand == true)
	//    {
	//        LIMIT(accZLpf, -1000.f, 1000.f);
	//    }
	//    else
	//    {
	//        HeightInfo.Z_Acc = accZRemovalDead;
	//    }

	//    HeightInfo.Z_Acc = accZRemovalDead;
	//    HeightInfo.Z_Acc -= 0.02f * Err * weight * weight * dt;    /*�������ٶ�*/
	//    
	//    HeightInfo.Z_Postion += HeightInfo.Z_Speed * dt + HeightInfo.Z_Acc * dt * dt / 2.0f;
	//    HeightInfo.Z_Speed += HeightInfo.Z_Acc * dt;
	//    
	//    Err = HeightInfo.Alt - HeightInfo.Z_Postion;        
	//    
	//    ewdt = Err * weight * dt;
	//    HeightInfo.Z_Postion += ewdt;
	//    HeightInfo.Z_Speed += weight * ewdt;
	//    
	//    if(isKeyFlightLand == true)        
	//    {
	//        HeightInfo.Z_Speed = LIMIT(HeightInfo.Z_Speed, -VELOCITY_LIMIT, VELOCITY_LIMIT);    
	//    }
	//}

	///******************************************************************************
	//  * �������ƣ�ResetAltSpeed
	//  * ���������������ٶ�����
	//  * ��    �룺void
	//  * ��    ����void
	//  * ��    �أ�void 
	//  * ��    ע��null  
	//  *
	//  *
	//******************************************************************************/
	//void ResetAltSpeed(void)
	//{    
	//    accZLpf = 0.f;
	//    HeightInfo.Alt  = 0.f;
	//    HeightLPF = 0.f;
	//    HeightInfo.Z_Speed = 0.f;
	//    HeightInfo.Z_Postion = HeightInfo.Alt;
	//}

	///******************************************************************************
	//  * �������ƣ�applyDeadbandf
	//  * ������������Ӧ������Χ
	//  * ��    �룺
	//  * value������
	//  * deadband��������Χ
	//  * ��    ����void
	//  * ��    �أ�void 
	//  * ��    ע��null  
	//  *
	//  *
	//******************************************************************************/
	//float applyDeadbandf(float value, float deadband)
	//{
	//    if (ABS(value) < deadband) {
	//        value = 0;
	//    } else if (value > 0) {
	//        value -= deadband;
	//    } else if (value < 0) {
	//        value += deadband;
	//    }
	//    return value;
	//}

//===================ANOTC===================
#include "Ano_OF.h"

#define FIX_A1 0.8f
#define FIX_B1 0.5f
#define FIX_C1 0.2f

#define FIX_A2 1.6f
#define FIX_B2 0.5f
#define FIX_C2	0.2f

//wz_fus struct
float fix_rat[3];
_wz_fus_sta_enum wz_fus_sta;

int32_t obs_wz_velocity[2];
int32_t obs_wz_height[2];

int32_t obs_wz_hasl;
int32_t obs_wz_hasl_gnd;

float obs_wz_velocity_ref;
float obs_wz_height_ref;
	
float est_wz_velocity;
float est_wz_height;

float fix_wz_velocity;
float fix_wz_height;
float fix_wz_acceleration;

static uint8_t height_init_f = 10;

static int32_t height_old[2];
bool Select_Baro = false;
void WZ_Obs_Calcu(float dT_s)//����OBS���ݸ�������
{
	//height[1]
	
	obs_wz_hasl = (int32_t)(g_SPL06Manager.fALT *100);//cm
	
	//
	if(height_init_f!=0)
	{
		//
		height_init_f --;
		//
		obs_wz_hasl_gnd = obs_wz_hasl;
	}
	else
	{
		obs_wz_height[0] = obs_wz_hasl - obs_wz_hasl_gnd;
	}
	//height[2]
    {
        uint16_t tmp = ANO_OF.ALT;
        static uint16_t Alt_Old = 0;
        static float time = 0;
        
        if(tmp < 200)
        {
            obs_wz_height[1] = ANO_OF.ALT;
					
					
					
            Alt_Old = ANO_OF.ALT;
            
            time = 0;
        }else
        {
            obs_wz_height[1] = Alt_Old;
            time += dT_s;
            
            if(time > 3)
            {
                Select_Baro = true;
                ANO_OF.STATE.alt = 0;
            }
        }
    }
	
	//velocity
	obs_wz_velocity[0] = (int32_t)((obs_wz_height[0] - height_old[0])/dT_s);
	obs_wz_velocity[1] = (int32_t)((obs_wz_height[1] - height_old[1])/dT_s);
	
	height_old[0] = obs_wz_height[0];
	height_old[1] = obs_wz_height[1];
	
	//selector
	if(ANO_OF.STATE.alt != 0 && !Select_Baro)
	{
		obs_wz_velocity_ref = obs_wz_velocity[1];
		//
		fix_rat[0] = FIX_A2;
		fix_rat[1] = FIX_B2;
		fix_rat[2] = FIX_C2;
	}
	else
	{
		obs_wz_velocity_ref = obs_wz_velocity[0];
		//
		fix_rat[0] = FIX_A1;
		fix_rat[1] = FIX_B1;
		fix_rat[2] = FIX_C1;
	}
}


void WZ_Est_Calcu(float dT_s)//����������ݸ�������
{
    //���ٶȼƻ���
	est_wz_velocity += HeightInfo.Z_Acc *dT_s;
	
    //�ٶ�ֵ����
	est_wz_height += est_wz_velocity *dT_s;
}


void WZ_Fix_Calcu(float dT_s)//�����������
{
	//==calcu
	//�ٶ�ֵ�޷�
	obs_wz_velocity_ref = (obs_wz_velocity_ref > 500) ? 500 : ((obs_wz_velocity_ref < -500 ) ? -500 : obs_wz_velocity_ref);
	
    //����۲�߶Ȼ���ֵ
	obs_wz_height_ref += obs_wz_velocity_ref * dT_s;
	
    //�����ĸ߶�ֵ = �������� * ���۲�߶Ȼ���ֵ - ���ٶȵĻ��ָ߶ȣ�
	fix_wz_height = fix_rat[0] * ((float)obs_wz_height_ref - est_wz_height);
    
	//�������ٶ�ֵ = �������� * ���۲��ٶ�ֵ - ���ٶȵĻ����ٶȣ�
	fix_wz_velocity = fix_rat[1] * ((float)obs_wz_velocity_ref - est_wz_velocity);
    
	//�����ļ��ٶ�ֵ = �������� * ���������ٶ�ֵ��* dt;
	fix_wz_acceleration += fix_rat[2] * (fix_wz_velocity) *dT_s;
    
    //�����ļ��ٶ��޷�
	fix_wz_acceleration = (fix_wz_acceleration > 50) ? 50 : ((fix_wz_acceleration < -50 ) ? -50 : fix_wz_acceleration);
	
    //
	//==fix
	//
	est_wz_height += fix_wz_height *dT_s;	
	//
	est_wz_velocity += (fix_wz_acceleration + fix_wz_velocity ) *dT_s;
	//==
	HeightInfo.Z_Speed = est_wz_velocity;
	HeightInfo.Z_Postion = est_wz_height;
	
	//
	if(wz_fus_sta == FIX_RESET || g_FMUflg.unlock == 0)
	{
		WZ_Fus_Reset();
		//
		wz_fus_sta = FIX_WORKING;
	}
}

void WZ_Fus_Reset()
{
	obs_wz_height_ref = 0;
	obs_wz_velocity_ref = 0;
	est_wz_height = 0;
    est_wz_velocity = 0;
}

/******************* (C) ��Ȩ���� 2018 �����пƺƵ�Ƽ����޹�˾ *******************/
