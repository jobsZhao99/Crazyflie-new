#include "CBC_Trajectory.h"


bool isInit=false;
uint32_t StartTime;
bool CB_Planner_Init(uint32_t tick)
{
    if(isInit)
    {
        DEBUG_PRINT("CBC WARNING! Drone is flying in last trajector, could not Start new Trajecory!");
        return false;
    }
    else
    {
        StartTime=tick;
        return isInit=true;
    }
    
}
bool CB_NextState(uint32_t tick,CB_DesireState_t *desireState)
{

}

