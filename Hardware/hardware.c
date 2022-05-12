#include "hardware.h"
#include "include.h"
#include "msp.h"
#include "HARDWARE_i2c.h"
#include "HARDWARE_uart.h"
#include "timer_drv.h"


/*MCU初始化*/
void MCU_Init(void)
{
    //停止监督看门狗定时器
    MAP_WDT_A_holdTimer();
    
    //配置系统时钟到48M
    SystemCoreClockUpdate();
    
    //滴答计时器初始化
    SysTick_Configuration();

    //电机PWM初始化
    Motor_Init();
    
    //100ms延时，用于等待光流初始化，不要删除此语句！
    Delay_ms(100);
    
    //IIC初始化
    I2C_Init();

    //串口初始化
    USART_Init();
    
    //MCU开总中断
    MAP_Interrupt_enableMaster();  
}

/*LED区*/
void Hardware_LED_Red_ON(uint8_t luminance)
{
    P2OUT |= GPIO_PIN0;
}


void Hardware_LED_Red_OFF(void)
{
    P2OUT &= ~GPIO_PIN0;
}


void Hardware_LED_Red_TOGGLE(void)
{
    
}

void Hardware_LED_Green_ON(uint8_t luminance)
{
    P1OUT |= GPIO_PIN0;
}

void Hardware_LED_Green_OFF(void)
{
    P1OUT &= ~GPIO_PIN0;
}


void Hardware_LED_Green_TOGGLE(void)
{

}


void Hardware_LED_Blue_ON(uint8_t luminance)
{

}


void Hardware_LED_Blue_OFF(void)
{

}


void Hardware_LED_Blue_TOGGLE(void)
{

}


void Hardware_LED_MOTOR1_ON(void)
{

}

void Hardware_LED_MOTOR1_OFF(void)
{

}

void Hardware_LED_MOTOR1_TOGGLE(void)
{

}

void Hardware_LED_MOTOR2_ON(void)
{

}

void Hardware_LED_MOTOR2_OFF(void)
{

}

void Hardware_LED_MOTOR2_TOGGLE(void)
{

}

void Hardware_LED_MOTOR3_ON(void)
{

}

void Hardware_LED_MOTOR3_OFF(void)
{

}

void Hardware_LED_MOTOR3_TOGGLE(void)
{

}

void Hardware_LED_MOTOR4_ON(void)
{

}

void Hardware_LED_MOTOR4_OFF(void)
{

}

void Hardware_LED_MOTOR4_TOGGLE(void)
{

}

//void Delay_ms(uint16_t ms)
//{
//    uint32_t curTicks;
//    curTicks = msTicks;
//    while ((msTicks - curTicks) < ms);
//}

