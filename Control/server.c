#include "server.h"
#include "include.h"
#include "timer_drv.h"

/**
 * @函数名称: server_init
 * @函数描述: 舵机函数初始化，生成50HZ的PWM波
 * @输入: void
 * @输出: void
 * @返回: void
 * @备注: null
 */
void server_init(void)
{

    Timer_A_PWMConfig pwmConfig =
        {
            TIMER_A_CLOCKSOURCE_SMCLK,
            TIMER_A_CLOCKSOURCE_DIVIDER_48,
            20000, //48000000 / 48 /
            TIMER_A_CAPTURECOMPARE_REGISTER_4,
            TIMER_A_OUTPUTMODE_RESET_SET,
            1500};

    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P6, GPIO_PIN6 + GPIO_PIN7,
                                                    GPIO_PRIMARY_MODULE_FUNCTION);

    MAP_Timer_A_generatePWM(TIMER_A2_BASE, &pwmConfig);

    pwmConfig.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_3;
    MAP_Timer_A_generatePWM(TIMER_A2_BASE, &pwmConfig);
}

/**
 * @函数名称: set_server_angle
 * @函数描述: 设置两个舵机角速度
 * @输入: serverID_t ID：选择舵机ID
 *        float palstance：舵机角速度
 * @输出: void
 * @返回: void
 * @备注: 注意输出的是角速度
 */
void set_server_angle(serverID_t ID, float palstance)
{
    switch (ID)
    {
        case ServerV:
            Timer_A_setCompareValue(TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_3, palstance * 25 + 1500);
            break;
        case ServerH:
            Timer_A_setCompareValue(TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_4, palstance * 25 + 1500);
            break;
        default:
            break;
    }
}
