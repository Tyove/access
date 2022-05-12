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
    
    PIDGroup[emPID_FolloLineSpdVertically].kp = 0.45f;
    PIDGroup[emPID_FolloLineSpdVertically].ki = 0.13f;
    PIDGroup[emPID_FolloLineSpdVertically].kd = 0.014f;

    PIDGroup[emPID_FolloLineSpdHorizontally].kp = 0.45f;
    PIDGroup[emPID_FolloLineSpdHorizontally].ki = 0.13f;
    PIDGroup[emPID_FolloLineSpdHorizontally].kd = 0.014f;
    
    PIDGroup[emPID_FolloLinePosVertically].desired = 60/2;
    PIDGroup[emPID_FolloLinePosHorizontally].desired = 80/2;
    
    FollowManager.ptrPIDInfoH->OutLimitHigh = 20;
    FollowManager.ptrPIDInfoH->OutLimitLow = -20;
    FollowManager.ptrPIDInfoV->OutLimitHigh = 20;
    FollowManager.ptrPIDInfoV->OutLimitLow = -20;

    FollowManager.CountDownNumMs = MAX_COUNTDOWN;
    FollowManager.TargetAltitudeCM = TARGETALTITUDECM;
    
    FollowManager.ptrFrame = (OpenMVFrame_t*)UsartGroup[UART_A3].RxBuff;
    FollowManager.ptrUAVInfo = &g_UAVinfo;
    
    P1DIR &= ~(1 << BIT1);
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN1);
    P1DIR &= ~(1 << BIT4);
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN4);
    
    for(int i = 0;i < 3; i++)
    {
        StandardControl.StandardControlDirction[i].Speed = 0; 
    }
}


//以100hz的速度轮询 10ms
void UpdateCentControl(float dt)
{    
    //判断OpenMV返回的数据是否可用，有的时候OpenMV会返回无效数据
    if(FollowManager.ptrFrame->CentPoint.x1 > 200 || FollowManager.ptrFrame->CentPoint.y1 > 200)
        return ;
    
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
    switch(FollowManager.ActionList)
    {
        //判断
        case ActionWaitting:
            //Do nothing;
            break;
        case ActionCountdown:
            FollowManager.CountDownNumMs--;
        
            if(FollowManager.CountDownNumMs <= 0)
            {
                FollowManager.ActionList = ActionTakeOff;
            }
            break;
        case ActionTakeOff:
            {
                //起飞后悬停到悬停点:
                if(FollowApriTag)
                {
                    ActionHoldPoint(MAX_HOVER_ERR, 500, ActionHoverStartPoint);
                }
            }
            break;
        case ActionHoverStartPoint:
            ActionHoldPoint(MAX_HOVER_ERR, 100, ActionHoldApriTag);
//            ActionHoldPoint(MAX_HOVER_ERR, MAX_HOVER_TIME, ActionGoRight);
            break;
        case ActionGoRight:
            {
                static ActionTimes_t ActionTimes = ActionTimes1;
                
                switch(ActionTimes)
                {
                    case ActionTimes1:
                        if(ActionFormChange(MAX_FORMCHANGE_TIME, MirrorFlipLtype, ActionHoldMirrorFlipTurnLtype))
                        {
                            ActionTimes++;
                        }
                        break;
                    case ActionTimes2:
                        if(ActionFormChange(MAX_FORMCHANGE_TIME, MirrorFlipLtype, ActionHoldTurnTtype))
                        {
                            ActionTimes++;
                        }
                        break;
                    default:
                        break;
                }
            }
            break;
        case ActionHoldMirrorFlipTurnLtype:
            ActionHoldPoint(MAX_HOVER_ERR, MAX_HOVER_TIME, ActionGoForward);
            break;
        case ActionGoForward:
            {
                if(FollowApriTag)
                {
                    if(FollowManager.ptrFrame->FormType == ApriTag)
                    {
                        static int cnt = 0;
                        
                        cnt++;
                        
                        if(cnt > 10)
                        {
                            FollowManager.ActionList = ActionHoldApriTag;
                        }
                    }
                    
//                    ActionHoldPoint(MAX_HOVER_ERR, 300, ActionGoBack);
                }
            }
            break;
        case ActionHoldMirrorFlipLtype:
            ActionHoldPoint(MAX_HOVER_ERR, MAX_HOVER_TIME, ActionLand);
            break;
        case ActionGoLeft:
            ActionFormChange(MAX_FORMCHANGE_TIME, TurnLtype, ActionHoldTurnLtype);
            break;
        case ActionHoldTurnLtype:
            ActionHoldPoint(MAX_HOVER_ERR, MAX_HOVER_TIME, ActionGoBack);
            break;
        case ActionGoBack:
            ActionHoldPoint(MAX_FORMCHANGE_TIME, 300, ActionLand);
            break;
        case ActionHoldLtype:
            ActionHoldPoint(MAX_HOVER_ERR, MAX_HOVER_TIME, ActionGoRight);
            break;
        case ActionHoldTurnTtype:
            ActionHoldPoint(MAX_HOVER_ERR, MAX_HOVER_TIME, ActionGoForward);
            break;
        case ActionHoldCross:
//            ActionHoldPoint(MAX_HOVER_ERR, MAX_HOVER_TIME * 10, ActionLand);
            break;
        case ActionHoldApriTag:
            //悬停时间 10s
//            ActionHoldPoint(MAX_HOVER_ERR, 1500, ActionLand);
            break;
        case ActionHoverStopPoint:
            {
                
            }
            break;
        case ActionFollowTarget:
            {
                
            }
            break;
        case ActionLand:
            {
                static int Cnt = MAX_TIMEOUT1;

                if(Cnt-- < 0)
                {
                    FollowManager.ActionList = ActionLock;
                }
            }
            break;
        case ActionLock:
            FollowManager.ActionList = ActionWaitting;
            break;
        case ActionTest:
            {
                static int cnt = 0;
                cnt++;
            }
            break;
        case ActionSonar:
            {

            }
            
            break;
        default:
            break;
    }
}

//只执行动作
void UpdateAction(float dt)
{
    switch(FollowManager.ActionList)
    {
        case ActionWaitting:
            //Do nothing
            break;
        case ActionTakeOff:
            UpdateCMD(0, 0, CmdTakeOff);
            break;
        case ActionHoverStartPoint:
            //起飞
            {
                program_ctrl.vel_cmps_h[Y] = 0;
                program_ctrl.vel_cmps_h[X] = 0;
            }
            break;
        case ActionGoForward:
        case ActionGoBack:
            //前后飞行
            {
                HoldCurrentPostion(dt);
                if(FollowManager.ActionList == ActionGoForward)
                {
                    program_ctrl.vel_cmps_h[X] = 15;
                }
                else if(FollowManager.ActionList == ActionGoBack)
                {
                    program_ctrl.vel_cmps_h[X] = -15;
                }
            }
            break;
        case ActionGoLeft:
        case ActionGoRight:
            {
                HoldCurrentPostion(dt);
                if(FollowManager.ActionList == ActionGoLeft)
                {    
                    program_ctrl.vel_cmps_h[Y] = 20;
                }
                else if(FollowManager.ActionList == ActionGoRight)
                {
                    program_ctrl.vel_cmps_h[Y] = -20;
                }
            }
            break;
            
        //悬停动作
        case ActionHoldLtype:
        case ActionHoldMirrorFlipLtype:
        case ActionHoldTurnLtype:
        case ActionHoldMirrorFlipTurnLtype:
        case ActionHoldCross:
        case ActionHoldTtype:
        case ActionHoldTurnTtype:
        case ActionHoldFeaturePoint:
        case ActionHoldApriTag:
            if(FollowManager.ptrFrame->FormType == ApriTag)
            {
                PIDGroup[emPID_FolloLinePosVertically].desired = 120/2;
                PIDGroup[emPID_FolloLinePosHorizontally].desired = 160/2;
                
                HoldCurrentPostion(dt);
            }else
            {
                program_ctrl.vel_cmps_h[Y] = 0;
                program_ctrl.vel_cmps_h[X] = 0;
            }
            break;
        case ActionHoverStopPoint:
            //控制
            break;
        case ActionLand:
            //降落
            UpdateCMD(0, 0, CmdLand);
            break;
        case ActionLock:
            g_UAVinfo.FMUflg->unlock = 0;
            break;
        case ActionTest:

            break;
        case ActionSonar:
            {
                
            }
            break;
        default:
            break;
    }

    {//此处为Debug信息
        static int cnt = 0;
        cnt++;
        
        if(cnt % 4 == 0)
        {
            UpdateDebugInfo();
        }
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
    
    UpdatePID(FollowManager.ptrPIDInfoH, dt);  //PID
    UpdatePID(FollowManager.ptrPIDInfoV, dt);  //PID
    
    PIDGroup[emPID_FolloLineSpdVertically].desired = FollowManager.ptrPIDInfoV->out;
    PIDGroup[emPID_FolloLineSpdHorizontally].desired = FollowManager.ptrPIDInfoH->out;
    
    UpdatePID(&PIDGroup[emPID_FolloLineSpdHorizontally], dt);  //PID
    UpdatePID(&PIDGroup[emPID_FolloLineSpdVertically], dt);  //PID

    program_ctrl.vel_cmps_h[Y] = PIDGroup[emPID_FolloLineSpdHorizontally].out;
    program_ctrl.vel_cmps_h[X] = PIDGroup[emPID_FolloLineSpdVertically].out;
}

#include "Ano_OF.h"
extern _ano_of_st ANO_OF;
extern HeightInfo_t HeightInfo;
void UpdateDebugInfo()
{
    UpdateToGCSLine2(PIDGroup[emPID_FolloLinePosVertically].measured,PIDGroup[emPID_FolloLinePosHorizontally].measured,PIDGroup[emPID_FolloLinePosHorizontally].desired,PIDGroup[emPID_FolloLinePosVertically].desired, 0,0,0,0);
}

bool ActionFormChange(int8_t HoldTime, FormType_t TargetFormType, FSMList_t NextAction)
{
    static int cnt = 0;
    bool ChangeFinished = false;
            
    if(FollowManager.ptrFrame->FormType == TargetFormType)
    {
        cnt++;
        
        if(cnt > HoldTime)
        {
            cnt = 0;
            FollowManager.ActionList = NextAction;
            ChangeFinished = true;
        }
    }else
    {
        cnt = 0;
    }
    
    return ChangeFinished;
}

void ActionHoldPoint(int8_t Err, int16_t HoldTime, FSMList_t NextAction)
{
    static bool Enter = true;
    static uint16_t CountDown = 0;
    
    if(Enter)
    {
        CountDown = HoldTime;
        Enter = false;
    }else
    {
        CountDown--;
    }
    
    if(CountDown == 0)
    {
        Enter = true; 
        FollowManager.ActionList = NextAction;
    }
}

//规定：参数Distance,Speed中，左为正方向，右为负方向
void proControl(int16_t Distance, int16_t Speed)
{   
    Distance = LIMIT(Distance, -8, 8);
    Speed = LIMIT(Speed, -10, 10);

    if(Distance > 0 && Speed > 0)
    {
        UpdateCMD(Distance, Speed, CmdLeft);
    }else if(Distance < 0 && Speed < 0)
    {
        UpdateCMD(Distance, Speed, CmdRight);
    }
}



//超时限制
void TimeoutCheck()
{
    FSMList_t tmpAction;
    static int Cnt = 0;
    
    if(tmpAction == FollowManager.ActionList)
    {
        Cnt++;
        
        if(Cnt > MAX_TIMEOUT2)
        {
            tmpAction = ActionEmergencyLand;
        }else if(Cnt > MAX_TIMEOUT1)
        {
            tmpAction = ActionLand;
        }
    }else
    {
        Cnt = 0;
        tmpAction = FollowManager.ActionList;
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
    if(input)
    {
        
    }else
    {
        FollowLine = true;
    }
    
    //判断寻找ApriTag按钮是否按下
    if(input2)
    {
        
    }else
    {
        FollowApriTag = true;
    }
    
    //判断当前是否被多按
    if(FollowApriTag == false && FollowLine == false)
    {
        return;
    }else
    {
        static bool CloseGate = true;
        
        //动作线进入倒计时状态
        if(CloseGate)
        {
            CloseGate = false;
            FollowManager.ActionList = ActionCountdown;
            
        }
    }

    
}

