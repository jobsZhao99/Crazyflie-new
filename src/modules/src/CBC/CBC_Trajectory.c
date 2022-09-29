#include "CBC_Trajectory.h"
#include "math.h"
#include "debug.h"
static float TimeRate = RATE_MAIN_LOOP;
static bool TrajIsInit=false;
static CB_State_t StartState;
static uint32_t StartTime;
static CB_StateOfFliht_t SoF=CB_Idel;
/// @brief 
/// @param tick 
/// @param state Start state
/// @return 
bool CB_Planner_Init(uint32_t tick,CB_State_t state)
{
    if(TrajIsInit)
    {
        DEBUG_PRINT("CBC WARNING! Drone is flying in last trajector, could not Start new Trajecory!\n");
        return TrajIsInit=false;
    }
    else
    {
        StartState=state;
        StartTime=tick;
        TimeRate=RATE_MAIN_LOOP;
        SoF=CB_TakeOff;
		DEBUG_PRINT("CBC NOTICE: Trajectory Initialized at (%ld)!\nTrajecory origin will base on (%f)\t(%f)\t(%f)\t(%f)\n",tick,(double)StartState.x,(double)StartState.y,(double)StartState.z,(double)StartState.yaw);
        return TrajIsInit=true;
    }
}

bool CB_Planner_DisAbled(float z)
{
    if(TrajIsInit)
    {
        if(SoF==CB_Landing&&z<0.04f)
        {
            TrajIsInit=false;
            StartState=(CB_State_t){.x=0.0f,.y=0.0f,.z=0.0f,.yaw=0.0f};
            DEBUG_PRINT("CBC NOTICE: Trajectory is disabled!\n");
            SoF=CB_Idel;
            return true;
        }
        {
            SoF=CB_Landing;
            return false;
        }
        
    }   
    else
    {
        DEBUG_PRINT("CBC NOTICE: Try to disable rajectory fail!\n");
        return false;
    }
}

bool CB_Planner_test()
{
    return TrajIsInit;
}
bool CB_NextState(uint32_t tick,CB_State_t *desireState,float z)
{
    // float runtime = (float)(StartTime-tick)/1000.0f;
    desireState->x=StartState.x;
    desireState->y=StartState.y;
    switch (SoF)
    {
    case CB_Idel:
        /* code */
        break;
    case CB_TakeOff:
        desireState->z=0.5f;
        break;
    case CB_Cruise:
        break;
    case CB_Landing:
        desireState->z=0.5f*z;
        break;
    default:
        break;
    }
    
    desireState->yaw=StartState.yaw;
    
    return true;
}






