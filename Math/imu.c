/*==============================================================================
                         ##### How to use this driver #####
==============================================================================
imu������ֱ�ӵ���GetAngle,��̬���ݴ洢��g_Attitude��


*/
//�ⲿ�ļ�����
#include "imu.h"
#include "myMath.h"
#include <math.h>
#include "height_control.h"

//�궨����



//Extern����



//˽�к�����



//˽�б�����
Attitude_t g_Attitude;    //��ǰ�Ƕ���ֵ̬
float NormAccz;


/******************************************************************************
  * �������ƣ�GetAngle
  * �������������ݴ��������ݼ���õ���ǰ��̬
  * ��    �룺const MPU6050Manager_t *pMpu��ԭʼ����������
               float dt����λ����ʱ��
  * ��    ����Attitude_t *pAngE������Ƕ�ֵ
  * ��    �أ�void
  * ��    ע��null    
  *    
  *
******************************************************************************/
static Quaternion NumQ = {1, 0, 0, 0};
static float vecxZ,vecyZ,veczZ;
static float wz_acc_tmp[2];
void ATT_Update(const MPU6050Manager_t *pMpu,Attitude_t *pAngE, float dt) 
{
  volatile struct V{
        float x;
        float y;
        float z;
        } Gravity,Acc,Gyro,AccGravity;

    static  float KpDef = 0.4f; //��Ԫ������ֵ

    float q0_t,q1_t,q2_t,q3_t;
    float NormQuat; 
    float HalfTime = dt * 0.5f;

    Gravity.x = 2 * (NumQ.q1 * NumQ.q3 - NumQ.q0 * NumQ.q2);                
    Gravity.y = 2 * (NumQ.q0 * NumQ.q1 + NumQ.q2 * NumQ.q3);              
    Gravity.z = 1 - 2 * (NumQ.q1 * NumQ.q1 + NumQ.q2 * NumQ.q2);  
    // ���ٶȹ�һ����
    NormQuat = Q_rsqrt(squa(g_MPUManager.accX)+ squa(g_MPUManager.accY) +squa(g_MPUManager.accZ));

    Acc.x = pMpu->accX * NormQuat; //��һ��ɻ�Ϊ��λ�����·������
    Acc.y = pMpu->accY * NormQuat;  
    Acc.z = pMpu->accZ * NormQuat;  

    //������˵ó���ֵ����˺���Եõ���ת����������������µļ��ٶȷ����ϵ�ƫ��
    AccGravity.x = (Acc.y * Gravity.z - Acc.z * Gravity.y);
    AccGravity.y = (Acc.z * Gravity.x - Acc.x * Gravity.z);
    AccGravity.z = (Acc.x * Gravity.y - Acc.y * Gravity.x);

    //���ٶ��ںϼ��ٶȱ�������ֵ�����������乲ͬ�γ���PI�������õ�������Ľ��ٶ�ֵ
    Gyro.x = pMpu->gyroX * Gyro_Gr + KpDef * AccGravity.x;//�����ƣ��˴��������ǽ��ٶȵ�Ư��
    Gyro.y = pMpu->gyroY * Gyro_Gr + KpDef * AccGravity.y;
    Gyro.z = pMpu->gyroZ * Gyro_Gr + KpDef * AccGravity.z; 
    
    // һ�����������, ������Ԫ��
    //������Ľ��ٶ�ֵ���֣��õ�������̬��������Ԫ��һ��ʵ��Q0�������鲿Q1~3��ֵ�ı仯
    q0_t = (-NumQ.q1 * Gyro.x - NumQ.q2 * Gyro.y - NumQ.q3 * Gyro.z) * HalfTime;
    q1_t = ( NumQ.q0 * Gyro.x - NumQ.q3 * Gyro.y + NumQ.q2 * Gyro.z) * HalfTime;
    q2_t = ( NumQ.q3 * Gyro.x + NumQ.q0 * Gyro.y - NumQ.q1 * Gyro.z) * HalfTime;
    q3_t = (-NumQ.q2 * Gyro.x + NumQ.q1 * Gyro.y + NumQ.q0 * Gyro.z) * HalfTime;

    //���ֺ��ֵ�ۼӵ��ϴε���Ԫ���У����µ���Ԫ��
    NumQ.q0 += q0_t;
    NumQ.q1 += q1_t;
    NumQ.q2 += q2_t;
    NumQ.q3 += q3_t;
    
    // ������Ԫ����һ�����õ���λ������
    NormQuat = Q_rsqrt(squa(NumQ.q0) + squa(NumQ.q1) + squa(NumQ.q2) + squa(NumQ.q3)); //�õ���Ԫ����ģ��
    NumQ.q0 *= NormQuat; //ģ��������Ԫ��ֵ
    NumQ.q1 *= NormQuat;
    NumQ.q2 *= NormQuat;
    NumQ.q3 *= NormQuat;
    
    /*��������ϵ�µ�Z��������*/
    vecxZ = 2 * NumQ.q0 *NumQ.q2 - 2 * NumQ.q1 * NumQ.q3 ;                                  /*����(3,1)��*///��������ϵ�µ�X�����������
    vecyZ = 2 * NumQ.q2 *NumQ.q3 + 2 * NumQ.q0 * NumQ.q1;                                   /*����(3,2)��*///��������ϵ�µ�Y�����������
    veczZ = NumQ.q0 * NumQ.q0 - NumQ.q1 * NumQ.q1 - NumQ.q2 * NumQ.q2 + NumQ.q3 * NumQ.q3;  /*����(3,3)��*///��������ϵ�µ�Z����������� 

    NormAccz = -pMpu->accX* vecxZ + pMpu->accY * vecyZ + pMpu->accZ * veczZ;  /*Z�ᴹֱ�����ϵļ��ٶȣ���ֵ��������бʱ��Z����ٶȵ������ͣ����ǵ���������Ӧ�ó���ֵ*/        
    wz_acc_tmp[0] = (NormAccz - 2048) * 0.479f;// cm/ss //0.1196f;
    wz_acc_tmp[1] += 0.1f *(wz_acc_tmp[0] - wz_acc_tmp[1]);//LPF
    HeightInfo.Z_Acc = wz_acc_tmp[1];
}

void GetAngle(Attitude_t *pAngE) 
{
    //����ŷ����
    pAngE->yaw = atan2f((2 * NumQ.q1 *NumQ.q2 + 2 * NumQ.q0 * NumQ.q3),(1 - 2 * (NumQ.q2 * NumQ.q2 + NumQ.q3 * NumQ.q3))) * RtA;
    pAngE->pitch  =  asin(vecxZ)* RtA;   //������          
    pAngE->roll  = atan2f(vecyZ,veczZ) * RtA;  //�����
}

//��λ��Ԫ��
void IMU_Reset()
{
    NumQ.q0 = 1;
    NumQ.q1 = 0;
    NumQ.q2 = 0;
    NumQ.q3 = 0;
}

/******************************************************************************
  * �������ƣ�GetNormAccz
  * ��������������Z�᷽��ļ��ٶ�ֵ
  * ��    �룺void
  * ��    ��������Z�᷽��ļ��ٶ�ֵ
  * ��    �أ�void
  * ��    ע��null    
  *
  *
******************************************************************************/
float GetNormAccz(void)
{
   return NormAccz;
}

/******************* (C) ��Ȩ���� 2018 �����пƺƵ�Ƽ����޹�˾ *******************/
