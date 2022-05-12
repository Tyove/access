#ifndef _SDK_H
#define _SDK_H

#include <stdint.h>
#include "FollowLine.h"

//T432 API接口

//T432速度控制
/*
       (+X)
        |
        |
(+Y)---------(-Y)
        |
        |
       (-X)

Cmd:
CmdTakeOff,
CmdLand,
CmdSpeeedControl,

Speed:
控制飞机横纵方向速度

//起飞命令
T432_API_Speed(CmdTakeOff,0,0);

//降落命令
T432_API_Speed(CmdLand,0,0);

飞机向左边飞行，我们建议速度不超过30
T432_API_Speed(CmdSpeeedControl,0,20);
*/
#define MAX_SPEED   30

void T432_API_Speed(uint8_t Cmd ,int16_t X_Speed, int16_t Y_Speed);


#endif
