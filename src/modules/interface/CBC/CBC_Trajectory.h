#ifndef _CBC_Trajectory_h_
#define _CBC_Trajectory_h_
// #include "math.h"
#include "stabilizer_types.h"
// #include "debug.h"




typedef struct CB_DesireState_s
{
    float x;
    float y;
    float z;
    float yaw;
}CB_DesireState_t;

bool CB_Planner_Init(uint32_t tick);
bool CB_NextState(uint32_t tick,CB_DesireState_t *desireState);

#endif