/*==============================================================================
                         ##### How to use this driver #####
==============================================================================
���к����Զ����ã�����ѵϵͳ�е���
1.FlightPidControl����3msʱ�����һ��
2.MotorControl����3ms����һ��

*/
//�ⲿ�ļ�����
#include "include.h" 
#include "control.h"
#include "pid.h"
#include "math.h"
#include "fmuConfig.h"
#include "led.h"
#include "remote.h"
#include "spl06.h"
#include "imu.h"
#include "myMath.h"
#include "gcs.h"
#include "speed_estimator.h"
#include "remote.h"

//�궨����
#define EMERGENT    0
#define MOTOR1      motor[0] 
#define MOTOR2      motor[1] 
#define MOTOR3      motor[2] 
#define MOTOR4      motor[3]
#define ClearMotor  memset(motor, 0, sizeof(int16_t) * 4)

//Extern����



//˽�к�����



//˽�б�����
int16_t motor[4];
FMUflg_t g_FMUflg;      //ϵͳ��־λ������������־λ��

/******************************************************************************
  * �������ƣ�FlightPidControl
  * ��������������PID���ƺ���
  * ��    �룺float dt����λ����ʱ��
  * ��    ����void
  * ��    �أ�void 
  * ��    ע��null  
  *
  *
******************************************************************************/
extern uint8_t fc_state_take_off;
float PIDGroup_desired_yaw_pos_tmp,fb_gyro_lpf[3];
void FlightPidControl(float dt)
{
    volatile static uint8_t status = WAITING_1;

    //״̬����switch
    switch(status)
    {
        //�ȴ�״̬
        case WAITING_1:
            if(g_FMUflg.unlock)
            {
                status = READY_11;    
            }
            break;
        //׼��״̬
        case READY_11:
            ResetPID();                             //������λPID���ݣ���ֹ�ϴ�����������Ӱ�챾�ο���
            IMU_Reset();
            g_Attitude.yaw = 0;
            PIDGroup_desired_yaw_pos_tmp = g_Attitude.yaw;
            PIDGroup[emPID_Yaw_Pos].measured = 0;
            status = PROCESS_31;
            break;
        //��ʽ�������
        case PROCESS_31:                
			fb_gyro_lpf[0] += 0.5f *(g_MPUManager.gyroX * Gyro_G - fb_gyro_lpf[0]); //�ڻ�����ֵ �Ƕ�/��
			fb_gyro_lpf[1] += 0.5f *(g_MPUManager.gyroY * Gyro_G - fb_gyro_lpf[1]); //�ڻ�����ֵ �Ƕ�/��
			fb_gyro_lpf[2] += 0.5f *(g_MPUManager.gyroZ * Gyro_G - fb_gyro_lpf[2]); //�ڻ�����ֵ �Ƕ�/��

            //�ٶȻ�PID����ֵ�������Ǹ���
            PIDGroup[emPID_Roll_Spd].measured = fb_gyro_lpf[0];
            PIDGroup[emPID_Pitch_Spd].measured = fb_gyro_lpf[1];
            PIDGroup[emPID_Yaw_Spd].measured = fb_gyro_lpf[2];
        
            //λ�û�PID����ֵ�ɽ����������̬����
            PIDGroup[emPID_Pitch_Pos].measured = g_Attitude.pitch; //�⻷����ֵ ��λ���Ƕ�
            PIDGroup[emPID_Roll_Pos].measured = g_Attitude.roll;
            
            //YAW�������⴦��
            PIDGroup[emPID_Yaw_Pos].measured = 0;
            PIDGroup[emPID_Yaw_Pos].desired = (PIDGroup_desired_yaw_pos_tmp - g_Attitude.yaw);
            if(PIDGroup[emPID_Yaw_Pos].desired>=180)
            {
                PIDGroup[emPID_Yaw_Pos].desired -= 360;
            }
            else if(PIDGroup[emPID_Yaw_Pos].desired<=-180)
            {
                PIDGroup[emPID_Yaw_Pos].desired += 360;
            }

            ClacCascadePID(&PIDGroup[emPID_Roll_Spd],  &PIDGroup[emPID_Roll_Pos],  dt);      //X��
            ClacCascadePID(&PIDGroup[emPID_Pitch_Spd], &PIDGroup[emPID_Pitch_Pos], dt);     //Y��
//            UpdatePID(&PIDGroup[emPID_Yaw_Spd], dt);
            ClacCascadePID(&PIDGroup[emPID_Yaw_Spd],   &PIDGroup[emPID_Yaw_Pos],   dt);       //Z��

            break; 
        case EXIT_255:                  //�˳�����
            ResetPID();
            IMU_Reset();
            status = WAITING_1;         //���صȴ�����
          break;
        default:
            status = EXIT_255;
            break;
    }
    
    if(g_FMUflg.unlock == EMERGENT)     //�����ƶ�
    {
        status = EXIT_255;
    }
}



/******************************************************************************
  * �������ƣ�MotorControl
  * �������������µ�������߼�
  * ��    �룺void
  * ��    ����void
  * ��    �أ�void 
  * ��    ע��null    
  *    
  *
******************************************************************************/
void MotorControl(void)
{    
    volatile static uint8_t status = WAITING_1;

    //��������ж�
    if(g_FMUflg.unlock == EMERGENT)
    {
        status = EXIT_255;
    }
    
    //���״̬����
    switch(status)
    {
        //�ȴ�״̬1
        case WAITING_1: 
            if(g_FMUflg.unlock)
            {
                g_FMUflg.take_off = 0;    
                g_FMUflg.height_lock = 0; 
                status = WAITING_2;
            }
        //�ȴ�״̬2
        case WAITING_2:                               //������ɺ��ж�ʹ�����Ƿ�ʼ����ң�˽��з��п���
            {
                if(fc_state_take_off && !g_FMUflg.take_off) //�ս���ʱ�����������һ����ɲ���Ŀ��Z�ٶ�С��0����Ϊ�����߻��������
                {
                    status = PROCESS_31;
                }
                else if(g_FMUflg.take_off)
                {
                    g_FMUflg.height_lock = 1;
                    status = PROCESS_31;                            
                }
                
                //���ٵ��ת��
                MOTOR1 = 100;
                MOTOR2 = 100;
                MOTOR3 = 100;
                MOTOR4 = 100;
                break;
            }
        //������
        case PROCESS_31:
            {
                int16_t temp = 0;
                
                //���ƫ�ƽ����ƶ�������̬��С��ĳһЩֵʱ���ƶ�
                if(g_Attitude.pitch < -MAX_ISFD_ATTITUDE 
                || g_Attitude.pitch > MAX_ISFD_ATTITUDE
                || g_Attitude.roll  < -MAX_ISFD_ATTITUDE
                || g_Attitude.roll  > MAX_ISFD_ATTITUDE)
                {
                    g_FMUflg.unlock = 0;
                    status = EXIT_255;
                    ResetAlt();
                    ResetPID();
                    IMU_Reset();
                    ClearMotor;
                    break;
                }
                
                //��ͬ����ģʽʱ��������ͬ�������
                if(g_UAVinfo.UAV_Mode == Stabilize_Mode)
                {
                    temp = Remote.thr - 1000; 
                }

                if(g_UAVinfo.UAV_Mode >= Altitude_Hold)
                {
                    temp = HeightInfo.Thr;
                     RCReceiveHandle();
                }
                
             
                
                //������ֵ��Ϊ����ֵ��PWM
                MOTOR1 = LIMIT(temp, 0, MOTOR_MAX_INIT_VALUE); 
                MOTOR2 = LIMIT(temp, 0, MOTOR_MAX_INIT_VALUE); 
                MOTOR3 = LIMIT(temp, 0, MOTOR_MAX_INIT_VALUE); 
                MOTOR4 = LIMIT(temp, 0, MOTOR_MAX_INIT_VALUE); 
                
                //�������
                MOTOR1 += +PIDGroup[emPID_Roll_Spd].out - PIDGroup[emPID_Pitch_Spd].out + PIDGroup[emPID_Yaw_Spd].out;
                MOTOR2 += +PIDGroup[emPID_Roll_Spd].out + PIDGroup[emPID_Pitch_Spd].out - PIDGroup[emPID_Yaw_Spd].out;
                MOTOR3 += -PIDGroup[emPID_Roll_Spd].out + PIDGroup[emPID_Pitch_Spd].out + PIDGroup[emPID_Yaw_Spd].out;
                MOTOR4 += -PIDGroup[emPID_Roll_Spd].out - PIDGroup[emPID_Pitch_Spd].out - PIDGroup[emPID_Yaw_Spd].out;
                
                //�������
                MOTOR1 = LIMIT(MOTOR1, 100, MOTOR_MAX_VALUE); 
                MOTOR2 = LIMIT(MOTOR2, 100, MOTOR_MAX_VALUE); 
                MOTOR3 = LIMIT(MOTOR3, 100, MOTOR_MAX_VALUE); 
                MOTOR4 = LIMIT(MOTOR4, 100, MOTOR_MAX_VALUE); 
            }
            break;
        case EXIT_255:
            status = WAITING_1;    //���صȴ�����
            ClearMotor;
            break;
        default:
            break;
    }
    
    //���µ�����
    UpdateMotor(MOTOR1, MOTOR2, MOTOR3, MOTOR4);
}

/******************* (C) ��Ȩ���� 2018 �����пƺƵ�Ƽ����޹�˾ *******************/
