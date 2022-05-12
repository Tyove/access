#include "battery.h"
#include "myMath.h"

#define MAX_VALUE       2600
#define MIN_VALUE       2265

BatteryInfo_t g_BatteryInfo;

void update_battery_value()
{
    //HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&g_BatteryInfo.ADC_Value, 1);
}

void BatteryAlarmHandle()
{
    
    if(g_BatteryInfo.ADC_Value > 50)
    {        
        g_BatteryInfo.BatteryConnected = true;
        g_BatteryInfo.BatteryPercent = LIMIT((int8_t)(((g_BatteryInfo.ADC_Value - MIN_VALUE * 1.0f)/(MAX_VALUE - MIN_VALUE))*100), 0, 99);
    }else
    {
        g_BatteryInfo.BatteryConnected = false;
        g_BatteryInfo.BatteryPercent = 0;
    }
}

//void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
//{
//    BatteryAlarmHandle();
//}

