#ifndef _CBC_Trajectory_h_
#define _CBC_Trajectory_h_
// #include "math.h"
#include "stabilizer_types.h"
#include "debug.h"

bool CB_Planner_Init(uint32_t tick);
typedef struct CB_DesireState_s
{
    float x;
    float y;
    float z;
    float yaw;
}CB_DesireState_t;

#endif