#include "CBC_Trajectory.h"
#include "math.h"
#include "debug.h"
static float TimeRate = RATE_MAIN_LOOP;
static bool TrajIsInit=false;
static CB_State_t StartState;
static uint32_t StartTime;
/// @brief state of Drone
static CB_StateOfFliht_t SoF=CB_Idel;

static float runtime;
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

       float temp_t=GetRuntime(tick)-CruiseStartTime;
    switch (SoF)
    {
    case CB_Idel:
        /* code */
        break;
    case CB_TakeOff:
        desireState->x=0.0f;
        desireState->y=0.0f;
        desireState->z=0.5f;
        desireState->yaw=0.0f;
        
		if(temp_t>0.0f)
		{
		    SoF=CB_Cruise;
            // DEBUG_PRINT("system is from takeoff to cruise\n");
		}
        else
        {
            break;
        }
    case CB_Cruise:
        
        desireState->x=0.3f*(cosf(2*temp_t)-1.0f);
        desireState->y=0.3f*sinf(2*temp_t);
        desireState->z=0.5f;
        desireState->yaw=0.0f;
        // if(tick%200==0)
        // {
        //     DEBUG_PRINT("test time %f\n",(double)temp_t);

        // DEBUG_PRINT("system is crusing desire state is (%f)\t(%f)\t(%f)\t(%f)\n",
        // (double)desireState->x,
        // (double)desireState->y,
        // (double)desireState->z,
        // (double)desireState->yaw);
        // }
        
        break;
    case CB_Landing:
        desireState->x=0.0f;
        desireState->y=0.0f;
        desireState->z=0.5f*z;
        desireState->yaw=0.0f;
        break;
    default:
        break;
    }
    
    desireState->yaw+=StartState.yaw;
    desireState->x+=StartState.x;
    desireState->y+=StartState.y;
    desireState->z+=StartState.z;

    return true;
}

float GetRuntime(uint32_t tick)
{
    return runtime= (float)(tick-StartTime)/1000.0f;
}