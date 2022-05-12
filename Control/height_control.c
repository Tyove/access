/*==============================================================================
                         ##### How to use this driver #####
==============================================================================
高度控制驱动


*/
//外部文件引用
#include "height_control.h"
#include "control.h"
#include "SPL06.h"
#include "imu.h"
#include "mpu6050.h"
#include "fmuConfig.h"
#include "myMath.h"
#include "math.h"
#include "Remote.h"
#include "pid.h"

//私有变量区
HeightInfo_t HeightInfo;


//======================ANOTC==========================================
#include "gcs.h"
#include "program_ctrl.h"
//程序移植映射表
//=====mapping=====

#define PROGRAM_CTRL_VELOCITY_Z             (program_ctrl.vel_cmps_h[Z])
#define PROGRAM_CTRL_AUTO_TAKEOFF_LAND_VEL  (program_ctrl.auto_takeoff_land_velocity)

//=====mapping=====

#define MAX_EXP_WZ_VEL_UP 200
#define MAX_EXP_WZ_VEL_DW 150
#define MAX_EXP_WZ_ACC    500


//wz_ctrl struct
static float exp_vel_transition[4];
static float exp_vel_d;

static float exp_acc;
static float fb_acc;
static float exp_vel;
static float fb_vel;
static float exp_hei;
static float fb_hei;

//pid struct
static float hei_err,vel_err,acc_err,acc_err_i,acc_out,wz_out;

#define H_KP 1.5f
#define V_KP 5.0f
#define V_KD 0.05f
#define A_KP 0.4f
#define A_KI 0.6f

//extern uint8_t fc_state
extern int16_t ExpAlt;
extern int16_t FbAlt;
uint8_t fc_state_take_off = 0;
void ALT_Ctrl(float dT_s)
{
	//==input calculate
	//fb
	fb_vel = HeightInfo.Z_Speed;
	fb_hei = HeightInfo.Z_Postion;
	fb_acc = HeightInfo.Z_Acc;
    
	//exp
	exp_vel_transition[0] = (Remote.thr - 1500) * 0.001f;
    
    if(exp_hei >= 150)
    {
        if(exp_vel_transition[0] > 0)
        {
            exp_vel_transition[0] = 0;
        }
    }
    
	//deadzone
	if(exp_vel_transition[0]<0.1f && exp_vel_transition[0]>-0.1f)
	{
		exp_vel_transition[0] = 0;
	}
    
	//
	if(exp_vel_transition[0]>0)
	{
		//摇杆设置的Z速度 + 程控Z速度 + 起飞降落Z速度
		exp_vel_transition[1] = exp_vel_transition[0] *MAX_EXP_WZ_VEL_UP + PROGRAM_CTRL_VELOCITY_Z + PROGRAM_CTRL_AUTO_TAKEOFF_LAND_VEL;
	}
	else
	{
		exp_vel_transition[1] = exp_vel_transition[0] *MAX_EXP_WZ_VEL_DW + PROGRAM_CTRL_VELOCITY_Z + PROGRAM_CTRL_AUTO_TAKEOFF_LAND_VEL;
	}
	//
	float tmp = MAX_EXP_WZ_ACC*dT_s;
	exp_vel_d = (exp_vel_transition[1] - exp_vel_transition[2]);
	if(exp_vel_d > tmp)
	{
		exp_vel_d = tmp;
	}
	else if(exp_vel_d < -tmp)
	{
		exp_vel_d = -tmp;
	}
	//
	exp_vel_transition[2] += exp_vel_d;
	//
	exp_vel_transition[3] += 0.2f *(exp_vel_transition[2] - exp_vel_transition[3]);
	
	//==exp_val state
	//
	if(g_UAVinfo.UAV_Mode >= Altitude_Hold && fc_state_take_off != 0 )
  {
		exp_vel = exp_vel_transition[3];
		//
		exp_hei += exp_vel *dT_s;
		//
		if(exp_hei > fb_hei+150)
		{
			exp_hei = fb_hei+150;
		}
		else if(exp_hei < fb_hei-150)
		{
			exp_hei = fb_hei-150;
		}
	}
	else
	{
		exp_vel = 0;
		exp_hei = fb_hei;
	}
    
	//==ctrl
    ExpAlt = exp_hei;
    FbAlt = fb_hei;
    //高度限幅，升限为1.5M
//    exp_hei = LIMIT(exp_hei,-20,150);
    
	hei_err = (exp_hei - fb_hei);
	vel_err = ((H_KP *hei_err  + exp_vel) - (fb_vel + V_KD *fb_acc));
	exp_acc = (V_KP *vel_err);
	acc_err = exp_acc - fb_acc;
	acc_err_i += A_KI *acc_err *dT_s;
	acc_err_i = (acc_err_i > 600)?600:((acc_err_i<0)?0:acc_err_i);
	//output
	acc_out = A_KP *exp_acc;
	wz_out = acc_out + acc_err_i;
	wz_out = (wz_out > 1000)?1000:((wz_out < 0)?0:wz_out);
	HeightInfo.Thr = wz_out;
	
	//unlock state
	if(g_FMUflg.unlock == 0)
	{
		acc_err_i = 0;
		exp_hei = fb_hei;
		fc_state_take_off = 0;
	}
	else
	{
		if(g_UAVinfo.UAV_Mode >= Altitude_Hold)
		{
			//有向上的目标速度，状态切换为起飞
			if((int16_t)(exp_vel_transition[2])>0)
			{
                //增加高度位置判断
                if(HeightInfo.Z_Postion < 40 && HeightInfo.Z_Postion > -40)
                {
                    fc_state_take_off = 1;
                }else
                {
                    HeightInfo.Z_Postion = 0;
                    HeightInfo.Z_Acc = 0;
                    HeightInfo.Z_Speed = 0;
                }
			}
		}
		else//g_UAVinfo.UAV_Mode < Altitude_Hold
		{
			fc_state_take_off = 1;
		}
	}
}

/******************* (C) 版权所有 2018 北京中科浩电科技有限公司 *******************/
