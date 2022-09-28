#include "CBC_Trajectory.h"
#include "math.h"
#include "debug.h"
static float TimeRate = RATE_MAIN_LOOP;
static bool TrajIsInit=false;
static uint32_t StartTime;

bool CB_Planner_Init(uint32_t tick)
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
bool CB_NextState(uint32_t tick,CB_DesireState_t *desireState)
{
    float runtime = (float)(StartTime-tick)/1000;
    desireState->x=sin(runtime);
    return true;
}

