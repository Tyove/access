/*==============================================================================
                         ##### How to use this driver #####
==============================================================================
MPU6050������ʹ�÷������£�
1.����MPU6050Init�������鿴��ǰMPU6050�Ƿ��ʼ���ɹ���
2.�̶����ڵ���GetMPU6050Data���Ի�ȡ���������ݣ�

PS�����������ݴ����g_MPUManager��


*/
//�ⲿ�ļ�����
#include "include.h"
#include "mpu6050.h"
#include "filter.h"
#include <string.h>
#include "LED.h"
#include "myMath.h"
#include "kalman.h"
#include "HARDWARE_i2c.h"
#include "timer_drv.h"


//�궨����
#define SMPLRT_DIV          0x19    //�����ǲ����ʣ�����ֵ��0x07(125Hz)
#define CONFIGL             0x1A    //��ͨ�˲�Ƶ�ʣ�����ֵ��0x06(5Hz)
#define GYRO_CONFIG         0x1B    //�������Լ켰������Χ������ֵ��0x18(���Լ죬2000deg/s)
#define ACCEL_CONFIG        0x1C    //���ټ��Լ졢������Χ����ͨ�˲�Ƶ�ʣ�����ֵ��0x01(���Լ죬2G��5Hz)
#define ACCEL_ADDRESS       0x3B
#define ACCEL_XOUT_H        0x3B
#define ACCEL_XOUT_L        0x3C
#define ACCEL_YOUT_H        0x3D
#define ACCEL_YOUT_L        0x3E
#define ACCEL_ZOUT_H        0x3F
#define ACCEL_ZOUT_L        0x40
#define TEMP_OUT_H          0x41
#define TEMP_OUT_L          0x42
#define GYRO_XOUT_H         0x43
#define GYRO_XOUT_L         0x44    
#define GYRO_YOUT_H         0x45
#define GYRO_YOUT_L         0x46
#define GYRO_ZOUT_H         0x47
#define GYRO_ZOUT_L         0x48
#define PWR_MGMT_1          0x6B    //��Դ��������ֵ��0x00(��������)
#define WHO_AM_I            0x75    //IIC��ַ�Ĵ���(Ĭ����ֵ0x68��ֻ��)
#define MPU6050_PRODUCT_ID  0x68
#define MPU6052C_PRODUCT_ID 0x72
#define MPU6050_ADDRESS     0xD0    //0x68

#define Acc_Read()          I2C_Read_Bytes(MPU6050_ADDRESS, 0x3B, buffer, 6) //��ȡ���ٶ�
#define Gyro_Read()         I2C_Read_Bytes(MPU6050_ADDRESS, 0x43, &buffer[6], 6)  //  ��ȡ���ٶ�
//Extern����
extern uint8_t I2C_Read_Byte(uint8_t Slaveaddr, uint8_t REG_Address);


//˽�к�����



//˽�б�����
MPU6050Manager_t g_MPUManager;   //g_MPUManagerԭʼ����
int16_t *pMpu = (int16_t *)&g_MPUManager;
/******************************************************************************
  * �������ƣ�MPU6050Init
  * ����������g_MPUManager�ĳ�ʼ��
  * ��    �룺void
  * ��    ����g_MPUManager��ʼ�����   
              0:��ʼ���ɹ�
              1:��ʼ��ʧ��
  * ��    �أ� 
  * ��    ע��    
  *    
  *
******************************************************************************/
bool MPU6050Init(void) //��ʼ��
{
    uint8_t check = 0;

    check = I2C_Read_Byte(MPU6050_ADDRESS, 0x75);  //�ж�g_MPUManager��ַ
    
    if(check != MPU6050_PRODUCT_ID) //�����ַ����ȷ
    {
        g_MPUManager.Check = false;
        return false;
    }
    else
    {
        Delay_ms(200);
        I2C_Write_Byte(MPU6050_ADDRESS, PWR_MGMT_1,    0x80);   //��λ
        Delay_ms(200);
        I2C_Write_Byte(MPU6050_ADDRESS, SMPLRT_DIV,   0x00);   //�����ǲ����ʣ�0x00(1000Hz)
        Delay_ms(10);
        I2C_Write_Byte(MPU6050_ADDRESS, PWR_MGMT_1,   0x03);   //�����豸ʱ��Դ��������Z��
        Delay_ms(10);
        I2C_Write_Byte(MPU6050_ADDRESS, CONFIGL,      0x04);   //��ͨ�˲�Ƶ�ʣ�0x03(42Hz)
        Delay_ms(10);
        I2C_Write_Byte(MPU6050_ADDRESS, GYRO_CONFIG,  0x18);   //+-2000deg/s
        Delay_ms(10);
        I2C_Write_Byte(MPU6050_ADDRESS, ACCEL_CONFIG, 0x18);   //+-16
        Delay_ms(10);
        
        GetMPU6050Offset(); //����У׼����
        g_MPUManager.Check = true;
        return true;
    }
}

/******************************************************************************
  * �������ƣ�GetMPU6050Data
  * ������������ȡ�����Ǻͼ��ٶȼƵ����ݲ����˲�����
  * ��    �룺void
  * ��    ����void
  * ��    �أ�void
  * ��    ע��null
  *    
  *
******************************************************************************/
#include "timer_drv.h"
uint32_t test_6050time[3];
void GetMPU6050Data(void) 
{
    static float mpu_filter[2][6];
    int16_t mpu_filter_tmp[6];
    uint8_t buffer[12];

    //��ȡ���ٶȼ����ݺ�����������
    Acc_Read();
    Gyro_Read();

    
    for(int i = 0; i < 6; i++)
    {
        //ƴ�Ӷ�ȡ����ԭʼ����
        mpu_filter_tmp[i] = (((int16_t)buffer[i << 1] << 8) | buffer[(i << 1) + 1])
                - g_MPUManager.Offset[i];

        //ԭʼ����LPF
        mpu_filter[0][i] += 0.3f *(mpu_filter_tmp[i] - mpu_filter[0][i]);
        mpu_filter[1][i] += 0.3f *(mpu_filter[0][i]  - mpu_filter[1][i]);

        //��ֵ���ṹ��
        pMpu[i] = (int16_t)mpu_filter[1][i];
    }
}

/******************************************************************************
  * �������ƣ�GetMPU6050Offset
  * ������������ȡg_MPUManager��̬�´�����ƫ��
  * ��    �룺void
  * ��    ����void
  * ��    �أ�void
  * ��    ע��null
  *    
  *
******************************************************************************/
void GetMPU6050Offset(void) //У׼
{
    int32_t buffer[6] = { 0 };
    int16_t i = 0;  
    const int8_t MAX_GYRO_QUIET = 5;

    int16_t LastGyro[3] = {0};          /*wait for calm down*/
    int16_t ErrorGyro[3] = {0};         /*set offset initial to zero*/
    
    memset(g_MPUManager.Offset, 0, 12);
    g_MPUManager.Offset[2] = 2048;   //�����ֲ������趨���ٶȱ궨ֵ 

    //����ǰ300������
    for(int i = 0;i < 300;i++)
    {   
        Delay_ms(2);
        GetMPU6050Data();
    }
    
    //�жϷɻ��Ƿ�ƽ������ֹб�ſ���
    while(1)
    {
        static int cnt = 0;
        cnt++;
        
        Delay_ms(2);
        GetMPU6050Data();
        
        if(g_MPUManager.accX < 400 && g_MPUManager.accX > -400 && 
           g_MPUManager.accY < 400 && g_MPUManager.accY > -400 && 
           g_MPUManager.accZ < 400 && g_MPUManager.accZ > -400)
        {
            if (cnt > 2 * 500 * 2)
            {
                break;
            }
        }else
        {
            cnt = 0;
        }
    }
    
    //�ж��ɻ��Ƿ��Ѿ��ȶ�
    while(1)
    {
        if(ABS(g_MPUManager.gyroX) != 0 ||  
           ABS(g_MPUManager.gyroY) != 0 || 
           ABS(g_MPUManager.gyroZ) != 0)
        {
            for(i = 0; i < 3; i++)
            {
                ErrorGyro[i] = pMpu[i + 3] - LastGyro[i];
                LastGyro[i] = pMpu[i + 3];    
            }
            
            if(ABS(ErrorGyro[0]) < MAX_GYRO_QUIET && 
               ABS(ErrorGyro[1]) < MAX_GYRO_QUIET && 
               ABS(ErrorGyro[2]) < MAX_GYRO_QUIET)
            {
                break;
            }
        }
    }

    //ȡ��100����356���ƽ��ֵ��ΪУ׼ֵ
    for(i = 0; i < 356; i++)  
    {
        GetMPU6050Data();
        
        if(100 <= i)
        {
            for(int k = 0; k < 6; k++)
            {
                buffer[k] += pMpu[k];
            }
        }
    }

    //����У׼ֵ
    for(i = 0; i < 6; i++)
    {
        //����8λ�����ݳ���256
        g_MPUManager.Offset[i] = buffer[i] >> 8;
    }
}

/******************* (C) ��Ȩ���� 2018 �����пƺƵ�Ƽ����޹�˾ *******************/
