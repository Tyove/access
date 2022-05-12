#include "SDK.h"
#include "program_ctrl.h"
#include "FollowLine.h"
#include "myMath.h"


void T432_API_Speed(uint8_t Cmd ,int16_t X_Speed, int16_t Y_Speed)
{
    switch(Cmd)
    {
        case CmdTakeOff:
            UpdateCMD(0, 0, CmdTakeOff);
            break;
        case CmdLand:
            UpdateCMD(0, 0, CmdLand);
            break;
        case CmdSpeeedControl:
            program_ctrl.vel_cmps_h[Y] = LIMIT(Y_Speed, -MAX_SPEED, MAX_SPEED);
            program_ctrl.vel_cmps_h[X] = LIMIT(X_Speed, -MAX_SPEED, MAX_SPEED);
            break;
        default:
            break;
    }
}


