#include "pos_ctrl.h"
#include "Remote.h"
#include "Ano_OF.h"
#include "height_control.h"
#include "myMath.h"
#include "program_ctrl.h"

//程序移植映射表
//=====mapping=====
#define PROGRAM_CTRL_VELOCITY_X (program_ctrl.vel_cmps_h[X])
#define PROGRAM_CTRL_VELOCITY_Y (program_ctrl.vel_cmps_h[Y])

//=====mapping=====

//==pos ctrl struct
//==ctrl_val
float exp_velocity[2];
float fb_velocity[2];
float fb_velocity_old[2];
float fb_velocity_d[2];
float fb_velocity_fixed[2];
//==pid
float pos_velocity_err[2];
float pos_velocity_err_i[2];
float pos_out[2];
float pos_out_trans[2];//extern
/*
		 机头
     +X
      |		 
      |
+Y----◎Z-----
      |
      |
     机尾
*/
#define POS_KP   0.15f
#define POS_KD   0.00f
#define POS_KI   0.12f

void POS_Ctrl(float dT_s)
{
	//====input calcu====
	//
	exp_velocity[0] = -(Remote.pitch - 1500) * 0.3f + PROGRAM_CTRL_VELOCITY_X;
	exp_velocity[1] = -(Remote.roll - 1500) * 0.3f  + PROGRAM_CTRL_VELOCITY_Y;
	if(ABS(exp_velocity[0])<10)
	{
		exp_velocity[0] = 0;
	}
	if(ABS(exp_velocity[1])<10)
	{
		exp_velocity[1] = 0;
	}
    
	//
	fb_velocity[0] = ANO_OF.DX2;
	fb_velocity[1] = ANO_OF.DY2;
	//
	fb_velocity_fixed[0] = ANO_OF.DX2FIX;
	fb_velocity_fixed[1] = ANO_OF.DY2FIX;
	//====ctrl====
	//计算位置速度误差 = 期望速度 - 反馈速度
	pos_velocity_err[0] = exp_velocity[0] - fb_velocity[0];//fb_velocity_fixed[0];
	pos_velocity_err[1] = exp_velocity[1] - fb_velocity[1];//fb_velocity_fixed[1];
	
    //反馈速度的增量值
	fb_velocity_d[0] = - (fb_velocity[0] - fb_velocity_old[0]);
	fb_velocity_d[1] = - (fb_velocity[1] - fb_velocity_old[1]);
	
    //位置速度误差 
	pos_velocity_err_i[0] += (exp_velocity[0] - fb_velocity_fixed[0]) * dT_s;
	pos_velocity_err_i[1] += (exp_velocity[1] - fb_velocity_fixed[1]) * dT_s;
    
	//limit
	pos_velocity_err_i[0] = LIMIT( pos_velocity_err_i[0], -100, 100 );
	pos_velocity_err_i[1] = LIMIT( pos_velocity_err_i[1], -100, 100 );
	//
	
	//==historical value
	fb_velocity_old[0] = fb_velocity[0];
	fb_velocity_old[1] = fb_velocity[1];
	
	//====output====
	pos_out[0] = pos_velocity_err[0] *POS_KP + fb_velocity_d[0] *POS_KD + pos_velocity_err_i[0] *POS_KI;
	pos_out[1] = pos_velocity_err[1] *POS_KP + fb_velocity_d[1] *POS_KD + pos_velocity_err_i[1] *POS_KI;
	//limit
	pos_out[0] = LIMIT(pos_out[0],-20,20); 
	pos_out[1] = LIMIT(pos_out[1],-20,20); 
	//
	pos_out_trans[0] =  pos_out[0];
	pos_out_trans[1] = -pos_out[1];
	
	//state and reset
	if(fc_state_take_off == 0)
	{
		pos_velocity_err_i[0] = 0;
		pos_velocity_err_i[1] = 0;
	}
}

