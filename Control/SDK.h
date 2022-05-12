#ifndef _SDK_H
#define _SDK_H

#include <stdint.h>
#include "FollowLine.h"

//T432 API�ӿ�

//T432�ٶȿ���
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
���Ʒɻ����ݷ����ٶ�

//�������
T432_API_Speed(CmdTakeOff,0,0);

//��������
T432_API_Speed(CmdLand,0,0);

�ɻ�����߷��У����ǽ����ٶȲ�����30
T432_API_Speed(CmdSpeeedControl,0,20);
*/
#define MAX_SPEED   30

void T432_API_Speed(uint8_t Cmd ,int16_t X_Speed, int16_t Y_Speed);


#endif
