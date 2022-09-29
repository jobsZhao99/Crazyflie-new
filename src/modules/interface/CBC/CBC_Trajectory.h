#ifndef _CBC_Trajectory_h_
#define _CBC_Trajectory_h_
// #include "math.h"
#include "stabilizer_types.h"

typedef enum CB_StateOfFliht_s
{
    CB_Idel,
    CB_TakeOff,
    CB_Cruise,
    CB_Landing
}CB_StateOfFliht_t;


typedef struct CB_State_s
{
    float x;
    float y;
    float z;
    float yaw;
}CB_State_t;

bool CB_Planner_Init(uint32_t tick,CB_State_t state);
bool CB_Planner_DisAbled(float z);
bool CB_Planner_test();
bool CB_NextState(uint32_t tick,CB_State_t *desireState,float z);

#endif