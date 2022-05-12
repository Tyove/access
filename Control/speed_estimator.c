/*==============================================================================
                         ##### How to use this driver #####
==============================================================================


*/
//外部文件引用
#include "speed_estimator.h"
#include "height_control.h"
#include "SPL06.h"
#include "control.h"
#include "gcs.h"
#include "myMath.h"


//宏定义区
#define VELOCITY_LIMIT        (130.f)    /*速度限幅 单位cm/s*/

//Extern引用


//私有函数区
float applyDeadbandf(float value, float deadband);

//私有变量区
	//float Err = 0.f;            /*位移误差*/
	//float wBaro = 0.35f;            /*气压校正权重*/
	//float HeightLPF = 0.f;    /*融合高度，低通*/
	//float rangeLpf = 0.f;
	//float accZLpf = 0.f;            /*Z轴加速度低通*/

	///******************************************************************************
	//  * 函数名称：UpdateAltSpeed
	//  * 函数描述：Z轴速度估算
	//  * 输    入：float dt：单位运行时间
	//  * 输    出：void
	//  * 返    回：void 
	//  * 备    注：null  
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
	//    float accZRemovalDead = applyDeadbandf(HeightInfo.Z_Acc, 4);/*去除死区的Z轴加速度*/
	//    accZLpf += (accZRemovalDead - accZLpf) * 0.1f;        /*低通*/
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
	//    HeightInfo.Z_Acc -= 0.02f * Err * weight * weight * dt;    /*补偿加速度*/
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
	//  * 函数名称：ResetAltSpeed
	//  * 函数描述：重置速度数据
	//  * 输    入：void
	//  * 输    出：void
	//  * 返    回：void 
	//  * 备    注：null  
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
	//  * 函数名称：applyDeadbandf
	//  * 函数描述：适应死区范围
	//  * 输    入：
	//  * value：数据
	//  * deadband：死区范围
	//  * 输    出：void
	//  * 返    回：void 
	//  * 备    注：null  
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
void WZ_Obs_Calcu(float dT_s)//跟随OBS数据更新周期
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


void WZ_Est_Calcu(float dT_s)//跟随惯性数据更新周期
{
    //加速度计积分
	est_wz_velocity += HeightInfo.Z_Acc *dT_s;
	
    //速度值积分
	est_wz_height += est_wz_velocity *dT_s;
}


void WZ_Fix_Calcu(float dT_s)//跟随控制周期
{
	//==calcu
	//速度值限幅
	obs_wz_velocity_ref = (obs_wz_velocity_ref > 500) ? 500 : ((obs_wz_velocity_ref < -500 ) ? -500 : obs_wz_velocity_ref);
	
    //计算观测高度积分值
	obs_wz_height_ref += obs_wz_velocity_ref * dT_s;
	
    //修正的高度值 = 修正参数 * （观测高度积分值 - 加速度的积分高度）
	fix_wz_height = fix_rat[0] * ((float)obs_wz_height_ref - est_wz_height);
    
	//修正的速度值 = 修正参数 * （观测速度值 - 加速度的积分速度）
	fix_wz_velocity = fix_rat[1] * ((float)obs_wz_velocity_ref - est_wz_velocity);
    
	//修正的加速度值 = 修正参数 * （修正的速度值）* dt;
	fix_wz_acceleration += fix_rat[2] * (fix_wz_velocity) *dT_s;
    
    //修正的加速度限幅
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

/******************* (C) 版权所有 2018 北京中科浩电科技有限公司 *******************/
