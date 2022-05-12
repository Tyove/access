#include "ExDevice.h"
#include "Remote.h"

extern PreRemote_t PreRemote;

void ExDeviceHandle()
{
    HAL_UART_Transmit_IT(&huart3, (uint8_t*)&PreRemote.Start, sizeof(PreRemote_t));
}

