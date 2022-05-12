#include "driverlib.h"

#include <stdbool.h>
#include <math.h>

extern uint8_t SBusRxBuff[30];

void SysTick_Handler(void);
void PendSV_Handler(void);
void SVC_Handler(void);
void HardFault_Handler(void);
void EUSCIB0_IRQHandler(void);
