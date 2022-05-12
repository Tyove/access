/*==============================================================================
                         ##### How to use this driver #####
==============================================================================
PID����ʹ�÷������£�
1.����һ��PIDInfo_t�ṹ�壬������Ҫ���Ƶ����ݴ�Ž�ȥ��
2.��UpdatePID����������PID������
3.����ֱ�ӵ���ClacCascadePIDֱ�Ӽ��㴮��PID


*/
//�ⲿ�ļ�����
#include "pid.h"
#include "myMath.h"    


//�궨����



//Extern����



//˽�к�����



//˽�б�����
/*PID���̱���*/
PIDInfo_t PIDGroup[emNum_Of_PID_List];


/******************************************************************************
  * �������ƣ�ResetPID
  * ������������λPID
  * ��    �룺PID�ṹ��ָ��
  * ��    ����void
  * ��    �أ�void
  * ��    ע��null
  *    
  *
******************************************************************************/
void ResetPID(void)
{
    for(int i = 0; i < emNum_Of_PID_List; i++)
    {
        PIDGroup[i].integ = 0;
        PIDGroup[i].prevError = 0;
        PIDGroup[i].out = 0;
        PIDGroup[i].offset = 0;
    }
    
    PIDGroup[emPID_Height_Pos].desired = 80;
}

/******************************************************************************
  * �������ƣ�UpdatePID
  * ��������������PID���ֵ
  * ��    �룺PIDInfo_t* pid��Ҫ�����PID�ṹ��ָ��
              float dt����λ����ʱ��
  * ��    ����void
  * ��    �أ�void
  * ��    ע��null
  *    
  *
******************************************************************************/
void UpdatePID(PIDInfo_t* pid, const float dt)
{
    float deriv;
    
    pid->Err = pid->desired - pid->measured + pid->offset; //��ǰ�Ƕ���ʵ�ʽǶȵ����

    
    if(pid->Err < pid->DeathArea && pid->Err > -pid->DeathArea)
    {
        pid->Err = 0;
    }
    
    if(pid->Err_LimitHigh != 0 && pid->Err_LimitLow != 0)
    {
        pid->Err = LIMIT(pid->Err, pid->Err_LimitLow, pid->Err_LimitHigh);
    }
    
    pid->integ += pid->Err * dt;    
    
    if(pid->IntegLimitHigh != 0 && pid->IntegLimitLow != 0)
    {
        pid->integ = LIMIT(pid->integ, pid->IntegLimitLow, pid->IntegLimitHigh);
    }
    
    //deriv = (pid->Err - pid->prevError)/dt;  
    deriv = -(pid->measured - pid->prevError)/dt;
		
    pid->out = pid->kp * pid->Err + pid->ki * pid->integ + pid->kd * deriv;//PID���
    
    if(pid->OutLimitHigh != 0 && pid->OutLimitLow != 0)
    {
        pid->out = LIMIT(pid->out, pid->OutLimitLow, pid->OutLimitHigh);
    }
    
    pid->prevError = pid->measured;//pid->Err;  ΢�����У���ʽ���÷�
}

/******************************************************************************
  * �������ƣ�ClacCascadePID
  * �������������㴮��PID
  * ��    �룺PIDInfo_t* pidRate��PID�ٶȻ�
              PIDInfo_t* pidAngE��PID�ǶȻ�
              const float dt����λ����ʱ��
  * ��    ����void
  * ��    �أ�void
  * ��    ע��null    
  *    
  *
******************************************************************************/
void ClacCascadePID(PIDInfo_t* pidRate, PIDInfo_t* pidAngE, const float dt)  //����PID
{     
    UpdatePID(pidAngE, dt);    //�ȼ����⻷
    pidRate->desired = pidAngE->out;
    UpdatePID(pidRate, dt);   
}

/******************* (C) ��Ȩ���� 2018 �����пƺƵ�Ƽ����޹�˾ *******************/
