#include "include.h"
extern void Hadrware_Init(void);

int main()
{
    //MCU初始化
    MCU_Init();
    
    //相关硬件初始化
    Hadrware_Init();
    
    while(true)
    {
        //核心轮训
        KernelPolling();
    }
}
