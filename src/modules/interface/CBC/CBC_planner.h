#ifndef _CBC_planner_h_
#define _CBC_planner_h_
#define CruiseStartTime 5.0f

// #include "math.h"
#include "stabilizer_types.h"

typedef enum CB_StateOfFliht_s
{
    CB_Idel,
    CB_TakeOff,
    CB_Cruise,
    CB_Landing
}CB_StateOfFliht_t;


bool CB_planner_Init(uint32_t tick,state_t state);
bool CB_planner_DisAbled(float z);
bool CB_planner_test();
bool CB_plannerGetSetpoint(setpoint_t *setpoint,const state_t *state,uint32_t tick);
float GetRuntime(uint32_t tick);
#endif /*_CBC_planner_h_*/