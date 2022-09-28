#ifndef _CBC_Trajectory_h_
#define _CBC_Trajectory_h_
// #include "math.h"
#include "stabilizer_types.h"




typedef struct CB_State_s
{
    float x;
    float y;
    float z;
    float yaw;
}CB_State_t;

bool CB_Planner_Init(uint32_t tick,CB_State_t state);
bool CB_Planner_test();
bool CB_NextState(uint32_t tick,CB_State_t *desireState);

#endif