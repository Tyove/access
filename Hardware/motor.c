/*==============================================================================
                         ##### How to use this driver #####
==============================================================================
����������÷�ʽ���£�
1.���ú���UpdateMotorֱ�����������
���PWM��ʼ���Ѿ���main��������ɡ�


*/
//�ⲿ�ļ�����
#include "motor.h"
#include "myMath.h"
#include "timer_drv.h"
#include "eeprom.h"
//�궨����


//Extern����


//˽�к�����
void Motor_Calc(void);

//˽�б�����
Timer_A_PWMConfig pwmConfig =
{
        TIMER_A_CLOCKSOURCE_SMCLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_48,
        2500,
        TIMER_A_CAPTURECOMPARE_REGISTER_1,
        TIMER_A_OUTPUTMODE_RESET_SET,
        1000
};

/******************************************************************************
  * �������ƣ�UpdateMotor
  * �����������������ת����������ȡֵ0-1000
  * ��    �룺
  * int16_t M1:���1���ȡֵ
  * int16_t M2:���2���ȡֵ
  * int16_t M3:���3���ȡֵ
  * int16_t M4:���4���ȡֵ
  * ��    ����void
  * ��    �أ�void
  * ��    ע��null
  *    
  *
******************************************************************************/
void UpdateMotor(int16_t M1, int16_t M2, int16_t M3, int16_t M4)
{
    M1 = LIMIT(M1, 0, 999);   //���ȡֵ�޷� 0-999
    M2 = LIMIT(M2, 0, 999); 
    M3 = LIMIT(M3, 0, 999);
    M4 = LIMIT(M4, 0, 999);
        
#ifdef MSP432    
    M1 = M1 + 1000;
    M2 = M2 + 1000;
    M3 = M3 + 1000;
    M4 = M4 + 1000;
    
    Timer_A_setCompareValue (TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1, M2);
    Timer_A_setCompareValue (TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1, M4);//M4
    Timer_A_setCompareValue (TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_3, M1);//M1
    Timer_A_setCompareValue (TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_4, M3);
#endif    
    
#ifdef STM32
    TIM2->CCR1 = M1;
    TIM2->CCR2 = M2;
    TIM2->CCR3 = M3;
    TIM2->CCR4 = M4;
#endif
}

/******************************************************************************
  * �������ƣ�Motor_Init
  * ������������ʼ��PWM�����
  * ��    �룺void
  * ��    ����void
  * ��    �أ�void
  * ��    ע��null
  *    950 = 5%
  *    2000 = 10%
******************************************************************************/
void Motor_Init()
{
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN4
                                                                + GPIO_PIN6
//                                                                + GPIO_PIN5
                                                                + GPIO_PIN7, 
                                                                  GPIO_PRIMARY_MODULE_FUNCTION);
    
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P5, GPIO_PIN6,
                                                                  GPIO_PRIMARY_MODULE_FUNCTION);
    
    MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig);
    MAP_Timer_A_generatePWM(TIMER_A2_BASE, &pwmConfig);

    pwmConfig.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_4;
    MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig);

    pwmConfig.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_3;
    MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig);

    pwmConfig.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_1;
    MAP_Timer_A_generatePWM(TIMER_A2_BASE, &pwmConfig);

//    pwmConfig.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_2;
//    MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig);
//    Motor_Calc();
}

void Motor_Unlock()
{

}

void Motor_Lock()
{
    
}

#include "eeprom.h"
void Motor_Calc()
{
    uint8_t CalcCompleateFlag = 0;
    
    //�˴����밴ť���,Ȼ��д1��ESCУ��λ,�˴���Ҫ��鰴ť�˿ںͰ�ť����ʱ��ƽ�ߵ�
    P1DIR &= ~GPIO_PIN4;
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN4);
    if(!(P1IN & GPIO_PIN4))
    {
        while(!(P1IN & GPIO_PIN4));
        
        CalcCompleateFlag = 0;
        HAL_EEPROM_Write(PARAMETER_FMU_ESC_CORRECTION_FLAG_SHIFT, &CalcCompleateFlag, 1);
    }
    
    HAL_EEPROM_Read(PARAMETER_FMU_ESC_CORRECTION_FLAG_SHIFT, PARAMETER_FMU_ESC_CORRECTION_FLAG_LENGTH, &CalcCompleateFlag);
    
    if(CalcCompleateFlag != 1)
    {
        CalcCompleateFlag = 1;
        HAL_EEPROM_Write(PARAMETER_FMU_ESC_CORRECTION_FLAG_SHIFT, &CalcCompleateFlag, 1);
    }else
    {
        return ;
    }
    
    Timer_A_setCompareValue (TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1, 2000);
    Timer_A_setCompareValue (TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1, 2000);
    Timer_A_setCompareValue (TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_3, 2000);
    Timer_A_setCompareValue (TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_4, 2000);
    Delay_ms(1500);
    Timer_A_setCompareValue (TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1, 1000);
    Timer_A_setCompareValue (TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1, 1000);
    Timer_A_setCompareValue (TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_3, 1000);
    Timer_A_setCompareValue (TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_4, 1000);
    Delay_ms(1500);
}

/******************* (C) ��Ȩ���� 2018 �����пƺƵ�Ƽ����޹�˾ *******************/

