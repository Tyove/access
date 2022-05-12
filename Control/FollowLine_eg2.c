/*
���������һ���������̶�����
1.����ʱ��
2.�Զ���ɣ�
3.��������
4.������


����û���Ϊ�Ѿ����ո��ļ�ʹ�÷�������ɾ�����ļ���Ȼ�����FollowLine.c�ļ�
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

    //�ٶȻ���PID����
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

//��100hz���ٶ���ѯ 10ms
void UpdateCentControl(float dt)
{
    //�ж�OpenMV���ص������Ƿ���ã��е�ʱ��OpenMV�᷵����Ч����
    //if (FollowManager.ptrFrame->CentPoint.x1 > 200 || FollowManager.ptrFrame->CentPoint.y1 > 200)
        //return;

    //���°�ť����ʵ��
    UpdateButton();

    //���³̿�״̬��
    UpdateStatus();

    //���³̿ض�����
    UpdateAction(dt);
}

//�˺���ֻ��״̬�жϺ�״̬����
void UpdateStatus()
{
    //����ActionList�����ݣ����벻ͬ��״̬
    switch (FollowManager.ActionList)
    {
        //�ж�
        case ActionWaitting:
            //Do nothing;
            break;

        //����ʱ״̬
        case ActionCountdown:
        {
            //����ʱ�����ݳ�ʼ���λ��Follow_Init��
            FollowManager.CountDownNumMs--;

            //������ʱ����ʱ��״̬���ΪActionTakeOff
            if (FollowManager.CountDownNumMs <= 0)
            {
                FollowManager.ActionList = ActionTakeOff;
            }
        }
        break;

        //�Զ����״̬
        case ActionTakeOff:
        {
            //�Զ���ɶ�������ʱ��Ϊ5s��500 * 10ms = 5000ms = 5s����Ȼ������ActionHoverStartPoint������
            ActionHoldPoint(MAX_HOVER_ERR, 500, ActionHoverStartPoint);
        }
        break;

        //��ͣ������5S��Ȼ�������Զ�����״̬
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
        //�Զ�����״̬����ʱ�����Ժ󣬽�����������
        case ActionLand:
        {
            //����ʱ�߼�
            static int Cnt = MAX_TIMEOUT1;

            if (Cnt-- < 0)
            {
                FollowManager.ActionList = ActionLock;
            }
        }
        break;

        //��������
        case ActionLock:
            FollowManager.ActionList = ActionWaitting;
            break;
        default:
            break;
    }
}

//ִֻ�ж���
void UpdateAction(float dt)
{
    switch (FollowManager.ActionList)
    {
    //����ʱ����
    case ActionWaitting:
        //Do nothing
        break;

    //�Զ��������
    case ActionTakeOff:
        UpdateCMD(0, 0, CmdTakeOff);
        break;

    //��ͣ����
    case ActionHoverStartPoint:
        //���
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
		//�Զ�����
    case ActionLand:
        //��������
        UpdateCMD(0, 0, CmdLand);
        break;

    //��������
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
    
    //���²�����
    PIDGroup[emPID_FolloLinePosVertically].measured = FollowManager.ptrFrame->CentPoint.y1;
    PIDGroup[emPID_FolloLinePosHorizontally].measured = FollowManager.ptrFrame->CentPoint.x1;

    PIDGroup[emPID_FolloLineSpdVertically].measured = (FollowManager.ptrFrame->CentPoint.y1 - OldPos[0]);
    PIDGroup[emPID_FolloLineSpdHorizontally].measured = (FollowManager.ptrFrame->CentPoint.x1 - OldPos[1]);
    
    OldPos[0] = FollowManager.ptrFrame->CentPoint.y1;
    OldPos[1] = FollowManager.ptrFrame->CentPoint.x1;
    //��������ݸ��������
    
    UpdatePID(FollowManager.ptrPIDInfoH, dt);  //PID
    UpdatePID(FollowManager.ptrPIDInfoV, dt);  //PID
    
    PIDGroup[emPID_FolloLineSpdVertically].desired = FollowManager.ptrPIDInfoV->out;
    PIDGroup[emPID_FolloLineSpdHorizontally].desired = FollowManager.ptrPIDInfoH->out;
    
    UpdatePID(&PIDGroup[emPID_FolloLineSpdHorizontally], dt);  //PID
    UpdatePID(&PIDGroup[emPID_FolloLineSpdVertically], dt);  //PID

    //���������ٶȿ��Ƶ�
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
    //�ж����������Ƿ���Ч����ʵ���ж�������������
    volatile static uint8_t input = 0;
    volatile static uint8_t input2 = 0;
    input = P1IN & BIT1;
    input2 = P1IN & BIT4;

    //�ж�Ѳ�߰�ť�Ƿ���
    if (input)
    {
    }
    else
    {
        FollowLine = true;
    }

    //�ж�Ѱ��ApriTag��ť�Ƿ���
    if (input2)
    {
    }
    else
    {
        FollowApriTag = true;
    }

    //�жϵ�ǰ�Ƿ񱻶ఴ
    if (FollowApriTag == false && FollowLine == false)
    {
        return;
    }
    else
    {
        static bool CloseGate = true;

        //�����߽��뵹��ʱ״̬
        if (CloseGate)
        {
            CloseGate = false;
            FollowManager.ActionList = ActionCountdown;
        }
    }
}
