/*
本程序完成一个基本历程动作：
1.倒计时；
2.自动起飞；
3.画正方形
4.上锁；


如果用户认为已经掌握该文件使用方法，请删除此文件，然后添加FollowLine.c文件
*/

#include "FollowLine.h"
#include "HARDWARE_uart.h"
#include "stdbool.h"
#include "pid.h"
#include "timer_drv.h"
#include "myMath.h"
#include "gcs.h"
#include "program_ctrl.h"

extern Usart_t UsartGroup[Num_USART];
extern PIDInfo_t PIDGroup[emNum_Of_PID_List];
extern UAV_info_t g_UAVinfo;
extern u16 val, spd;
extern _program_ctrl_st program_ctrl;
bool FollowLine = false;
bool FollowApriTag = false;

void proControl(int16_t Distance, int16_t Speed);
void TimeoutCheck(void);
void UpdateStatus(void);
void UpdateAction(float dt);
void UpdateButton(void);
void ActionHoldPoint(int8_t Err, int16_t HoldTime, FSMList_t NextAction);
bool ActionFormChange(int8_t HoldTime, FormType_t TargetFormType, FSMList_t NextAction);
void UpdateDebugInfo(void);
void HoldCurrentPostion(float dt);

FollowManager_t FollowManager;
FollowManager_t1 FollowManager1;
SonarManager_t SonarManager;

/*
        |+X
        |
        |
+Y------------- -Y
        |
        |
        |-X
*/

void Follow_Init()
{
    FollowManager.ptrPIDInfoV = &PIDGroup[emPID_FolloLinePosVertically];
    FollowManager.ptrPIDInfoH = &PIDGroup[emPID_FolloLinePosHorizontally];

    FollowManager.ptrPIDInfoV->kp = 1.5f;
    FollowManager.ptrPIDInfoH->kp = 1.5f;

    FollowManager.ptrPIDInfoH->DeathArea = 3;
    FollowManager.ptrPIDInfoV->DeathArea = 3;

    //速度环的PID数据
    PIDGroup[emPID_FolloLineSpdVertically].kp = 0.45f;
    PIDGroup[emPID_FolloLineSpdVertically].ki = 0.13f;
    PIDGroup[emPID_FolloLineSpdVertically].kd = 0.014f;

    PIDGroup[emPID_FolloLineSpdHorizontally].kp = 0.45f;
    PIDGroup[emPID_FolloLineSpdHorizontally].ki = 0.13f;
    PIDGroup[emPID_FolloLineSpdHorizontally].kd = 0.014f;

    PIDGroup[emPID_FolloLinePosVertically].desired = 120 / 2;
    PIDGroup[emPID_FolloLinePosHorizontally].desired = 160 / 2;

    FollowManager.ptrPIDInfoH->OutLimitHigh = 20;
    FollowManager.ptrPIDInfoH->OutLimitLow = -20;
    FollowManager.ptrPIDInfoV->OutLimitHigh = 20;
    FollowManager.ptrPIDInfoV->OutLimitLow = -20;

    FollowManager.CountDownNumMs = MAX_COUNTDOWN;
    FollowManager.TargetAltitudeCM = TARGETALTITUDECM;

    FollowManager.ptrFrame = (OpenMVFrame_t *)UsartGroup[UART_A3].RxBuff;
	FollowManager.ptrFrame = (OpenMVFrame_t *)UsartGroup[UART_A1].RxBuff;
    FollowManager.ptrUAVInfo = &g_UAVinfo;

    P1DIR &= ~(1 << BIT1);
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN1);
    P1DIR &= ~(1 << BIT4);
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN4);

    for (int i = 0; i < 3; i++)
    {
        StandardControl.StandardControlDirction[i].Speed = 0;
    }
}

//以100hz的速度轮询 10ms
void UpdateCentControl(float dt)
{
    //判断OpenMV返回的数据是否可用，有的时候OpenMV会返回无效数据
    //if (FollowManager.ptrFrame->CentPoint.x1 > 200 || FollowManager.ptrFrame->CentPoint.y1 > 200)
        //return;

    //更新按钮控制实践
    UpdateButton();

    //更新程控状态线
    UpdateStatus();

    //更新程控动作线
    UpdateAction(dt);
}

//此函数只做状态判断和状态更新
void UpdateStatus()
{
    //根据ActionList的内容，进入不同的状态
    switch (FollowManager.ActionList)
    {
        //判断
        case ActionWaitting:
            //Do nothing;
            break;

        //倒计时状态
        case ActionCountdown:
        {
            //倒计时，数据初始填充位于Follow_Init中
            FollowManager.CountDownNumMs--;

            //当倒计时结束时候，状态变更为ActionTakeOff
            if (FollowManager.CountDownNumMs <= 0)
            {
                FollowManager.ActionList = ActionTakeOff;
            }
        }
        break;

        //自动起飞状态
        case ActionTakeOff:
        {
            //自动起飞动作持续时间为5s（500 * 10ms = 5000ms = 5s），然后跳到ActionHoverStartPoint动作；
            ActionHoldPoint(MAX_HOVER_ERR, 500, ActionHoverStartPoint);
        }
        break;

        //悬停到空中5S，然后跳到自动降落状态
        case ActionHoverStartPoint:
            ActionHoldPoint(MAX_HOVER_ERR, 300, ActionGoForward);
            break;

        case ActionGoForward:
					  ActionHoldPoint(MAX_HOVER_ERR,200,ActionGoLeft);
				    break;
				case ActionGoLeft:
					  ActionHoldPoint(MAX_HOVER_ERR,200,ActionGoBack);
					  break;
				case ActionGoBack:
					  ActionHoldPoint(MAX_HOVER_ERR,200,ActionGoRight);
				    break;
				case ActionGoRight:
					  ActionHoldPoint(MAX_HOVER_ERR,200,ActionLand);
				    break;
        //自动降落状态倒计时结束以后，进入上锁动作
        case ActionLand:
        {
            //倒计时逻辑
            static int Cnt = MAX_TIMEOUT1;

            if (Cnt-- < 0)
            {
                FollowManager.ActionList = ActionLock;
            }
        }
        break;

        //上锁动作
        case ActionLock:
            FollowManager.ActionList = ActionWaitting;
            break;
        default:
            break;
    }
}

//只执行动作
void UpdateAction(float dt)
{
    switch (FollowManager.ActionList)
    {
    //倒计时命令
    case ActionWaitting:
        //Do nothing
        break;

    //自动起飞命令
    case ActionTakeOff:
        UpdateCMD(0, 0, CmdTakeOff);
        break;

    //悬停命令
    case ActionHoverStartPoint:
        //起飞
        {
            program_ctrl.vel_cmps_h[Y] = 0;
            program_ctrl.vel_cmps_h[X] = 0;
        }
        break;
		case ActionGoBack:
		{
			  program_ctrl.vel_cmps_h[X] = -15;
        program_ctrl.vel_cmps_h[Y] = 0;
		}
		    break;
    case ActionGoForward:
		{
			  program_ctrl.vel_cmps_h[X] = 15;
        program_ctrl.vel_cmps_h[Y] = 0;
		}
		    break;
    case ActionGoLeft:
		{
			  program_ctrl.vel_cmps_h[X] = 0;
        program_ctrl.vel_cmps_h[Y] = -20;
		}
        break;
		case ActionGoRight:
		{
        program_ctrl.vel_cmps_h[X] = 0;
        program_ctrl.vel_cmps_h[Y] = 20;
		}
        break;
		//自动降落
    case ActionLand:
        //降落命令
        UpdateCMD(0, 0, CmdLand);
        break;

    //上锁动作
    case ActionLock:
        g_UAVinfo.FMUflg->unlock = 0;
        break;
    default:
        break;
    }
}

void HoldCurrentPostion(float dt)
{
    static float OldPos[2];
    
    //更新测量点
    PIDGroup[emPID_FolloLinePosVertically].measured = FollowManager.ptrFrame->CentPoint.y1;
    PIDGroup[emPID_FolloLinePosHorizontally].measured = FollowManager.ptrFrame->CentPoint.x1;

    PIDGroup[emPID_FolloLineSpdVertically].measured = (FollowManager.ptrFrame->CentPoint.y1 - OldPos[0]);
    PIDGroup[emPID_FolloLineSpdHorizontally].measured = (FollowManager.ptrFrame->CentPoint.x1 - OldPos[1]);
    
    OldPos[0] = FollowManager.ptrFrame->CentPoint.y1;
    OldPos[1] = FollowManager.ptrFrame->CentPoint.x1;
    //把这个数据更新完成了
    
    UpdatePID(FollowManager.ptrPIDInfoH, dt);  //PID
    UpdatePID(FollowManager.ptrPIDInfoV, dt);  //PID
    
    PIDGroup[emPID_FolloLineSpdVertically].desired = FollowManager.ptrPIDInfoV->out;
    PIDGroup[emPID_FolloLineSpdHorizontally].desired = FollowManager.ptrPIDInfoH->out;
    
    UpdatePID(&PIDGroup[emPID_FolloLineSpdHorizontally], dt);  //PID
    UpdatePID(&PIDGroup[emPID_FolloLineSpdVertically], dt);  //PID

    //这是我们速度控制点
    program_ctrl.vel_cmps_h[Y] = PIDGroup[emPID_FolloLineSpdHorizontally].out;
    program_ctrl.vel_cmps_h[X] = PIDGroup[emPID_FolloLineSpdVertically].out;
}

bool ActionFormChange(int8_t HoldTime, FormType_t TargetFormType, FSMList_t NextAction)
{
    static int cnt = 0;
    bool ChangeFinished = false;

    if (FollowManager.ptrFrame->FormType == TargetFormType)
    {
        cnt++;

        if (cnt > HoldTime)
        {
            cnt = 0;
            FollowManager.ActionList = NextAction;
            ChangeFinished = true;
        }
    }
    else
    {
        cnt = 0;
    }

    return ChangeFinished;
}

void ActionHoldPoint(int8_t Err, int16_t HoldTime, FSMList_t NextAction)
{
    static bool Enter = true;
    static uint16_t CountDown = 0;

    if (Enter)
    {
        CountDown = HoldTime;
        Enter = false;
    }
    else
    {
        CountDown--;
    }

    if (CountDown == 0)
    {
        Enter = true;
        FollowManager.ActionList = NextAction;
    }
}

void UpdateButton()
{
    //判定两个输入是否有效，其实是判断左右两个按键
    volatile static uint8_t input = 0;
    volatile static uint8_t input2 = 0;
    input = P1IN & BIT1;
    input2 = P1IN & BIT4;

    //判断巡线按钮是否按下
    if (input)
    {
    }
    else
    {
        FollowLine = true;
    }

    //判断寻找ApriTag按钮是否按下
    if (input2)
    {
    }
    else
    {
        FollowApriTag = true;
    }

    //判断当前是否被多按
    if (FollowApriTag == false && FollowLine == false)
    {
        return;
    }
    else
    {
        static bool CloseGate = true;

        //动作线进入倒计时状态
        if (CloseGate)
        {
            CloseGate = false;
            FollowManager.ActionList = ActionCountdown;
        }
    }
}
