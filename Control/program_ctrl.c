#include "program_ctrl.h"
#include "myMath.h"
#include "speed_estimator.h"
#include "height_control.h"
#include "gcs.h"
#include "Ano_OF.h"

//程序移植映射表
//=====mapping=====
//STATE
#define MOTOR_PREPARATION    (1)
#define FC_STATE_FLYING      (fc_state_take_off)
#define FC_STATE_UNLOKED     (g_FMUflg.unlock)
#define RC_STATE_FAILSAFE    (0)
#define FC_STATE_FLIGHT_MODE (g_UAVinfo.UAV_Mode >= Altitude_Hold)//当前为定点模式
//EN
#define FC_STATE_POS_HOLD_EN (ANO_OF.STATE.of_fus)                //光流有效，使能定点。
#define AUTO_TAKE_OFF_EN     (g_UAVinfo.UAV_Mode >= Altitude_Hold) //当前为定点模式，使能自动起飞功能

//SET_DATA
//自动起飞目标高度
#define EXP_TAKEOFF_ALT_CM   (90)   //CM

//自动起飞的速度
#define AUTO_TAKE_OFF_SPEED  (30)  //CMPS

//自动降落速度
#define AUTO_LAND_SPEED      (30)   //CMPS
#define MAX_VELOCITY_XY      (200)  //CMPS
#define MAX_VELOCITY_ZP      (200)  //CMPS
#define MAX_VELOCITY_ZN      (150)  //CMPS
#define MAX_PAL_YAW          (100)  //DPS

//STA_DATA
#define ALT_CM             (HeightInfo.Z_Postion)
//FUNCTION
#define FC_CMD_UNLOCK      (FC_Unlock_Fun())
#define FC_CMD_LOCK        (g_FMUflg.unlock = 0)
//=====mapping=====

static void FC_Unlock_Fun()
{
    g_FMUflg.unlock = 1;
}

StandardControl_t StandardControl;
_program_ctrl_st program_ctrl;
_atol_sta_enum auto_takeoff_land_state;
/**********************************************************************************************************
*函 数 名: One_Key_Take_off_Land
*功能说明: 一键起飞、降落触发
*参    数: 周期（秒）
*返 回 值: null
**********************************************************************************************************/
static u8 one_key_takeoff_f;
void One_Key_Takeoff()
{
    //解锁
    FC_CMD_UNLOCK ;
    //标记需要起飞
    if(one_key_takeoff_f == 0)
    {
        one_key_takeoff_f = 1;
    }
}

void One_Key_Land()
{
    //
    if(auto_takeoff_land_state == AUTO_TAKE_OFF_FINISH)
    {
        auto_takeoff_land_state = AUTO_LAND ;
    }
}


/**********************************************************************************************************
*函 数 名: One_Key_Take_off_Land_Ctrl
*功能说明: 一键起飞、降落控制
*参    数: 周期（秒），目标高度(厘米)
*返 回 值: null
**********************************************************************************************************/
void One_Key_Take_off_Land_Ctrl_Task(u8 dT_ms)
{
    static s32 ref_height;
    static s16 exp_time_ms,runtime_ms;
    static s16 time_compensate_ms;
    
    //触发，通过一键起飞函数置位
    if(one_key_takeoff_f != 0)
    {
        //电机准备好，电机时刻准备好
        if(MOTOR_PREPARATION)
        {
            //判断自动起飞降落状态是否为自动切费null
            if(auto_takeoff_land_state == AUTO_TAKE_OFF_NULL)
            {
                //如果是，则给定自动起飞状态
                auto_takeoff_land_state = AUTO_TAKE_OFF;
            }                
        }
    }
    else    //
    {
        //如果自动起飞状态为初始，且飞机已经起飞，则认为已经起飞完成（手动起飞）
        if(auto_takeoff_land_state == AUTO_TAKE_OFF_NULL && FC_STATE_FLYING != 0)
        {
            auto_takeoff_land_state = AUTO_TAKE_OFF_FINISH;
        }
    }
    
    //==state and ctrl
    if(auto_takeoff_land_state == AUTO_TAKE_OFF_NULL)
    {
        //
        ref_height = ALT_CM;
        //
        runtime_ms = 0;
        //
        program_ctrl.auto_takeoff_land_velocity = 0;
    }
    //
    else if(auto_takeoff_land_state == AUTO_TAKE_OFF)
    {
        //
        exp_time_ms = (EXP_TAKEOFF_ALT_CM - ref_height) * 1000 / AUTO_TAKE_OFF_SPEED;
        //起飞上升时间限制为0-5秒
        exp_time_ms = LIMIT(exp_time_ms,0,5000);
        //
        runtime_ms += dT_ms;
        //
        program_ctrl.auto_takeoff_land_velocity = AUTO_TAKE_OFF_SPEED;
        //
        if(runtime_ms>exp_time_ms)
        {
            //
            runtime_ms = 0;
            //
            auto_takeoff_land_state = AUTO_TAKE_OFF_FINISH;
        }
    }
    //
    else if(auto_takeoff_land_state == AUTO_TAKE_OFF_FINISH)
    {
        //
        program_ctrl.auto_takeoff_land_velocity = 0;
    }
    else if(auto_takeoff_land_state == AUTO_LAND)
    {
        //
        program_ctrl.auto_takeoff_land_velocity = -AUTO_LAND_SPEED;
    }
    //==reset
    if(FC_STATE_UNLOKED == 0)
    {
        //
        program_ctrl.auto_takeoff_land_velocity = 0;
        //
        auto_takeoff_land_state = AUTO_TAKE_OFF_NULL ;
        //
        one_key_takeoff_f = 0;
    }
}

//程控指令控制部分

//设计成单一线程执行命令。
u16 val, spd;

void Program_Ctrl_DataAnl(u8 *data)
{
//    val = ((*(data+3))<<8) + (*(data+4));
//    spd = ((*(data+5))<<8) + (*(data+6));    
//    program_ctrl.cmd_state[0] = *(data+2);
}


static u8 cmd_take_off_f;
void Program_Ctrl_Task(u8 dT_ms)
{
    if(program_ctrl.cmd_state[0] != program_ctrl.cmd_state[1])
    {
        //指令更新，复位数据,停止之前操作
        FlyCtrlReset();
        //
//        if(!RC_STATE_FAILSAFE && FC_STATE_FLIGHT_MODE && FC_STATE_POS_HOLD_EN)
        if(!RC_STATE_FAILSAFE && FC_STATE_FLIGHT_MODE)
        {
            program_ctrl.state_ok = 1;
        }
        else
        {
            program_ctrl.state_ok = 0;
                //发送字符串

                //复位指令状态
            program_ctrl.cmd_state[0] = 0;
        }
    }
    
    switch(program_ctrl.cmd_state[0])
    {
        case (0x01)://起飞
        {
            if(program_ctrl.state_ok != 0)
            {
                //起飞状态为初始状态，且遥控有信号,且光流或者GPS有效
                if(auto_takeoff_land_state == AUTO_TAKE_OFF_NULL)
                {
                    if(cmd_take_off_f == 0)
                    {
                        //
                        cmd_take_off_f = 1;
                        //发送字符串
                        //ANO_DT_SendString("Take off!");
                        //一键起飞
                        One_Key_Takeoff();
                    }

                }
                else if(auto_takeoff_land_state == AUTO_TAKE_OFF_FINISH)
                {
                    //发送字符串
                    //ANO_DT_SendString("Take off OK!");
                    //复位指令状态
                    program_ctrl.cmd_state[0] = 0;
                }
                else if(auto_takeoff_land_state > AUTO_TAKE_OFF_FINISH)
                {
                    //发送字符串
                    //ANO_DT_SendString("CMD Error!");
                    //复位指令状态
                    program_ctrl.cmd_state[0] = 0;                
                }
            }

        }
        break;
        case (0x02):    //降落
        {
            if(auto_takeoff_land_state == AUTO_TAKE_OFF_FINISH)
            {
                //复位指令状态
                //ANO_DT_SendString("Landing!");
                //一键降落
                One_Key_Land();
            }
            else if(auto_takeoff_land_state == AUTO_TAKE_OFF_NULL)
            {
                //发送字符串
                //ANO_DT_SendString("Landing OK!");
                //
                program_ctrl.cmd_state[0] = 0;
            }
        }
        break;
        case (0xA0):    //紧急停机
        {
            if(FC_STATE_UNLOKED)
            {
                //
                //ANO_DT_SendString("Emergency stop OK!");
                //上锁
                FC_CMD_LOCK;
                //
                program_ctrl.cmd_state[0] = 0;
            }
            
        }
        break;
        case (0x03):    //上升
        {    
            //目标速度赋值
            program_ctrl.vel_cmps_ref[Z] = spd;
            //目标时间赋值
            if(spd != 0)
            {
                program_ctrl.exp_process_t_ms[Z] = val*1000/LIMIT(spd,0,MAX_VELOCITY_ZP);
            }
            else
            {
                program_ctrl.exp_process_t_ms[Z] = 0;
            }

            //判断开始和完成
            if(program_ctrl.fb_process_t_ms[Z] == 0)
            {
                //
                //ANO_DT_SendString("Go up!");            
            }
            else if(program_ctrl.exp_process_t_ms[Z] < program_ctrl.fb_process_t_ms[Z])
            {
                //
                //ANO_DT_SendString("Go up OK!");
                //
                program_ctrl.cmd_state[0] = 0;
            }

            //计时反馈
            program_ctrl.fb_process_t_ms[Z] += dT_ms;
        }
        break;
        case (0x04):    //下降
        {
            //目标速度赋值
            program_ctrl.vel_cmps_ref[Z] = -spd;
            //目标时间赋值
            if(spd != 0)
            {
                program_ctrl.exp_process_t_ms[Z] = val*1000/LIMIT(spd,0,MAX_VELOCITY_ZN);
            }
            else
            {
                program_ctrl.exp_process_t_ms[Z] = 0;
            }

            //判断开始和完成
            if(program_ctrl.fb_process_t_ms[Z] == 0)
            {
                //
                //ANO_DT_SendString("Go down!");                
            }
            else if(program_ctrl.exp_process_t_ms[Z] < program_ctrl.fb_process_t_ms[Z])
            {
                //
                //ANO_DT_SendString("Go down OK!");
                //
                program_ctrl.cmd_state[0] = 0;
            }    

            //计时反馈
            program_ctrl.fb_process_t_ms[Z] += dT_ms;
        }
        break;
        case (0x05):    //前进
        {
            //目标速度赋值
            program_ctrl.vel_cmps_ref[X] = spd;
            //目标时间赋值
            if(spd != 0)
            {
                program_ctrl.exp_process_t_ms[X] = val*1000/LIMIT(spd,0,MAX_VELOCITY_XY);
            }
            else
            {
                program_ctrl.exp_process_t_ms[X] = 0;
            }

            //判断开始和完成
            if(program_ctrl.fb_process_t_ms[X] == 0)
            {
                //
                //ANO_DT_SendString("Go ahead!");                
            }
            else if(program_ctrl.exp_process_t_ms[X] < program_ctrl.fb_process_t_ms[X])
            {
                //
                //ANO_DT_SendString("Go ahead OK!");
                //
                program_ctrl.cmd_state[0] = 0;
            }

            //计时反馈
            program_ctrl.fb_process_t_ms[X] += dT_ms;
        }
        break;
        case (0x06):    //后退
        {
            //目标速度赋值
            program_ctrl.vel_cmps_ref[X] = -spd;
            //目标时间赋值
            if(spd != 0)
            {
                program_ctrl.exp_process_t_ms[X] = val*1000/LIMIT(spd,0,MAX_VELOCITY_XY);
            }
            else
            {
                program_ctrl.exp_process_t_ms[X] = 0;
            }

            //判断开始和完成
            if(program_ctrl.fb_process_t_ms[X] == 0)
            {
                //
                //ANO_DT_SendString("Go back!");                
            }
            else if(program_ctrl.exp_process_t_ms[X] < program_ctrl.fb_process_t_ms[X])
            {
                //
                //ANO_DT_SendString("Go back OK!");
                //
                program_ctrl.cmd_state[0] = 0;
            }

            //计时反馈
            program_ctrl.fb_process_t_ms[X] += dT_ms;
        }
        break;
        case (0x07):    //向左
        {
            //目标速度赋值
            program_ctrl.vel_cmps_ref[Y] = spd;
            //目标时间赋值
            if(spd != 0)
            {
                program_ctrl.exp_process_t_ms[Y] = val*1000/LIMIT(spd,0,MAX_VELOCITY_XY);
            }
            else
            {
                program_ctrl.exp_process_t_ms[Y] = 0;
            }

            //判断开始和完成
            if(program_ctrl.fb_process_t_ms[Y] == 0)
            {
                //
                //ANO_DT_SendString("Go left!");                
            }
            else if(program_ctrl.exp_process_t_ms[Y] < program_ctrl.fb_process_t_ms[Y])
            {
                //
                //ANO_DT_SendString("Go left OK!");
                //
                program_ctrl.cmd_state[0] = 0;
            }

            //计时反馈
            program_ctrl.fb_process_t_ms[Y] += dT_ms;
        }
        break;
        case (0x08):    //向右
        {
            //目标速度赋值
            program_ctrl.vel_cmps_ref[Y] = -spd;
            //目标时间赋值
            if(spd != 0)
            {
                program_ctrl.exp_process_t_ms[Y] = val*1000/LIMIT(spd,0,MAX_VELOCITY_XY);
            }
            else
            {
                program_ctrl.exp_process_t_ms[Y] = 0;
            }

            //判断开始和完成
            if(program_ctrl.fb_process_t_ms[Y] == 0)
            {
                //
                //ANO_DT_SendString("Go right!");                
            }
            else if(program_ctrl.exp_process_t_ms[Y] < program_ctrl.fb_process_t_ms[Y])
            {
                //
                //ANO_DT_SendString("Go right OK!");
                //
                program_ctrl.cmd_state[0] = 0; 
            }

            //计时反馈
            program_ctrl.fb_process_t_ms[Y] += dT_ms;
        }
        break;
        case (0x09):    //左转
        {
            //目标速度赋值
            program_ctrl.yaw_pal_dps = spd;
            //目标时间赋值
            if(spd != 0)
            {
                program_ctrl.exp_process_t_ms[3] = val*1000/LIMIT(spd,0,MAX_PAL_YAW);
            }
            else
            {
                program_ctrl.exp_process_t_ms[3] = 0;
            }

            //判断开始和完成
            if(program_ctrl.fb_process_t_ms[3] == 0)
            {
                //
                //ANO_DT_SendString("Turn left!");                
            }
            else if(program_ctrl.exp_process_t_ms[3] < program_ctrl.fb_process_t_ms[3])
            {
                //
                //ANO_DT_SendString("Turn left OK!");
                //
                program_ctrl.cmd_state[0] = 0;
            }

            //计时反馈
            program_ctrl.fb_process_t_ms[3] += dT_ms;
        }
        break;
        case (0x0A):    //右转
        {
            //目标速度赋值
            program_ctrl.yaw_pal_dps = -spd;
            //目标时间赋值
            if(spd != 0)
            {
                program_ctrl.exp_process_t_ms[3] = val*1000/LIMIT(spd,0,MAX_PAL_YAW);
            }
            else
            {
                program_ctrl.exp_process_t_ms[3] = 0;
            }

            //判断开始和完成
            if(program_ctrl.fb_process_t_ms[3] == 0)
            {
                //
                //ANO_DT_SendString("Turn right!");                
            }
            else if(program_ctrl.exp_process_t_ms[3] < program_ctrl.fb_process_t_ms[3])
            {
                //
                //ANO_DT_SendString("Turn right OK!");
                //
                program_ctrl.cmd_state[0] = 0;
            }
            
            //计时反馈
            program_ctrl.fb_process_t_ms[3] += dT_ms;
        }
        break;
        case (0x0B):    //水平方向

        break;
        
        default:
        {

        }
        break;
    }
    
    //复位操作
    if(program_ctrl.cmd_state[0] == 0)
    {
        FlyCtrlReset();
    }
    
    //记录历史值
    program_ctrl.cmd_state[1] = program_ctrl.cmd_state[0];
    
    //输出程序控制数据
    program_ctrl.vel_cmps_h[Z] = program_ctrl.vel_cmps_ref[Z];
    program_ctrl.vel_cmps_h[X] = program_ctrl.vel_cmps_ref[X];
    program_ctrl.vel_cmps_h[Y] = program_ctrl.vel_cmps_ref[Y];
}

void StdControl(StandardControl_t *ptrS)
{
    for(int i = 0;i<NumofVectorDir;i++)
    {
        StandardControl.StandardControlDirction[i].Exp_Time = (StandardControl.StandardControlDirction[i].Distance * 1000) / LIMIT(StandardControl.StandardControlDirction[i].Speed,-MAX_VELOCITY_XY,MAX_VELOCITY_XY);
        StandardControl.StandardControlDirction[i].Cnt = 0;
    }
}

void UpdateCMD(uint16_t Distance, uint16_t Speed, uint8_t Cmd)
{
    val = Distance;
    spd = Speed;
    program_ctrl.cmd_state[0] = Cmd;
}

void FlyCtrlReset()
{
    //
    cmd_take_off_f = 0;
    //
    for(u8 i = 0;i<4;i++)
    {
        if(i<4)
        {
            //
            program_ctrl.vel_cmps_ref[i] = 0;
            program_ctrl.vel_cmps_w[i] = 0;
            program_ctrl.vel_cmps_h[i] = 0;
        }
        else
        {
            program_ctrl.yaw_pal_dps = 0;
        }
        
        program_ctrl.exp_process_t_ms[i] = 0;
        program_ctrl.fb_process_t_ms[i] = 0;
    }
    
}

