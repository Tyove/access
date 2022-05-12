#ifndef _SERVER_H
#define _SERVER_H

#include "stdint.h"

typedef enum
{
    ServerV = 0,
    ServerH,
    
    Numofserver,
}serverID_t;

void server_init(void);
void set_server_angle(serverID_t, float);

#endif
