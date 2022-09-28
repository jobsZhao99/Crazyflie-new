#include "CBC_Trajectory.h"
#include "math.h"
#include "debug.h"
static float TimeRate = RATE_MAIN_LOOP;
static bool TrajIsInit=false;
static CB_State_t StartState;
static uint32_t StartTime;

/// @brief 
/// @param tick 
/// @param state Start state
/// @return 
bool CB_Planner_Init(uint32_t tick,CB_State_t state)
{
    if(TrajIsInit)
    {
        DEBUG_PRINT("CBC WARNING! Drone is flying in last trajector, could not Start new Trajecory!");
        return TrajIsInit=false;
    }
    else
    {
        StartTime=tick;
        TimeRate=RATE_MAIN_LOOP;
        return TrajIsInit=true;
    }
}

bool CB_Planner_test()
{
    return TrajIsInit;
}
bool CB_NextState(uint32_t tick,CB_State_t *desireState)
{
    // float runtime = (float)(StartTime-tick)/1000.0f;
    desireState->x=StartState.x;
    desireState->y=StartState.y;
    desireState->y=0.5;
    desireState->yaw=0;
    return true;
}





