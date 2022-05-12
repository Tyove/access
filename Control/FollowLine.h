#ifndef __FOLLOW_LINE_H
#define __FOLLOW_LINE_H

#include "stdint.h"
#include "stdbool.h"
#include "pid.h"
#include "gcs.h"

//��λ:10ms
#define MAX_COUNTDOWN       100 * 8
#define TARGETALTITUDECM    50
#define MAX_HOVER_ERR       10
#define MAX_FORMCHANGE_TIME 3
#define MAX_HOVER_TIME      300
#define MAX_TIMEOUT1        500
#define MAX_TIMEOUT2        1500
#define MAX_ALT_ERR         10

/*

*/

typedef enum
{
    CmdNone = 0,
    CmdTakeOff,
    CmdLand,
    CmdUp,
    CmdDown,
    CmdForward,
    CmdBack,
    CmdLeft,
    CmdRight,
    CmdCCW,
    CmdCW,
    CmdEmergencyShutDown,
    CmdSpeeedControl,
    
    NumofCmd,
}FMUCmd_t;

typedef enum
{
    Vertical  = 0,      //����
    Horizontal ,        //����
    Cross,              //ʮ��
    Ttype,              //T����
    TurnTtype,          //��T����
    Ltype,              //L����
    MirrorFlipLtype,    //����תL����
    TurnLtype,          //��L����
    MirrorFlipTurnLtype,//����ת��L����
    LeftTtype,
    ApriTag = 100,
    
    NumofForm,
}FormType_t;

typedef enum
{
    ActionWaitting = 0,
    ActionCountdown,
    ActionTakeOff,
    ActionHoverStartPoint,
    
    //��ͣ����
    ActionHoldLtype,
    ActionHoldMirrorFlipLtype,
    ActionHoldTurnLtype,
    ActionHoldMirrorFlipTurnLtype,
    ActionHoldCross,
    ActionHoldTtype,
    ActionHoldTurnTtype,
    ActionHoldFeaturePoint,
    ActionHoldLeftTtype,
    ActionHoldApriTag,
    
    //���ж���
    ActionGoForward,
    ActionGoRight,
    ActionGoLeft,
    ActionGoBack,
    ActionGoRound,
    ActionGoForward2,
    

    ActionRotateServer,
    ActionHoverStopPoint,
    ActionFollowTarget,
    ActionLostTargetInfo,
    ActionLand,
    ActionLock,
    ActionTest,
    ActionSonar,
    
    ActionEmergencyLand,

    NumofActionList,
}FSMList_t;

typedef enum
{
    ActionTimes1 = 0,
    ActionTimes2,
    ActionTimes3,
    ActionTimes4,
}ActionTimes_t;

typedef struct
{
    int16_t x1;
    int16_t y1;
    int16_t x2;
    int16_t y2;
}Line_t;

typedef struct
{
    int16_t x1;
    int16_t y1;
}Point_t;

//���ݽṹ����
#pragma pack (1)
typedef struct
{
    int16_t Start;
    //uint16_t Cnt;
    uint8_t cnt1;
    uint8_t Target;
    FormType_t FormType;
    Point_t CentPoint;
    int16_t End;
}OpenMVFrame_t;
#pragma pack ()    

typedef struct
{
    uint16_t SonarF;
    uint16_t SonarB;
    uint16_t SonarL;
    uint16_t SonarR;
    
}SonarManager_t;

typedef struct
{
    //OpenMV����֡   ͼ�����״��λ��
    OpenMVFrame_t *ptrFrame;
    
    //�ɻ���״̬
    UAV_info_t *ptrUAVInfo;
    
    //��������
    FSMList_t ActionList;
    
    //CountDownNumMs����ʱ����
    int16_t CountDownNumMs;
    int16_t TargetAltitudeCM;
    int16_t WatchDogCnt;
    
    //4��PID������
    PIDInfo_t   *ptrPIDInfoV;
    PIDInfo_t   *ptrPIDInfoH;
    
    bool ActionComplete;
}FollowManager_t;
typedef struct
{
    //OpenMV����֡   ͼ�����״��λ��
    OpenMVFrame_t *ptrFrame;
    
    //�ɻ���״̬
    UAV_info_t *ptrUAVInfo;
    
    //��������
    FSMList_t ActionList;
    
    //CountDownNumMs����ʱ����
    int16_t CountDownNumMs;
    int16_t TargetAltitudeCM;
    int16_t WatchDogCnt;
    
    //4��PID������
    PIDInfo_t   *ptrPIDInfoV;
    PIDInfo_t   *ptrPIDInfoH;
    
    bool ActionComplete;
}FollowManager_t1;

extern FollowManager_t FollowManager;
extern SonarManager_t SonarManager;

void UpdateCentControl(float dt);
void Follow_Init(void);
#endif
