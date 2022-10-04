#include "CBC_planner.h"
#include "math.h"
#include "debug.h"
#include "stabilizer_types.h"
#include "math3d.h"
static float TimeRate = RATE_MAIN_LOOP;
static bool TrajIsInit=false;
static uint32_t StartTime;
static state_t StartState;
/// @brief state of Drone
static CB_StateOfFliht_t SoF=CB_Idel;

static float runtime;
/// @brief 
/// @param tick 
/// @param state Start state
/// @return 
bool CB_planner_Init(uint32_t tick,state_t state)
{
    if(TrajIsInit)
    {
        DEBUG_PRINT("CBC WARNING! Drone is flying in last trajector, could not Start new Trajecory!\n");
        return TrajIsInit=false;
    }
    else
    {
        StartState=state;
        StartState.attitude.yaw*=M_PI_F/180.0f;
        StartTime=tick;
        TimeRate=RATE_MAIN_LOOP;
        SoF=CB_TakeOff;
		DEBUG_PRINT("CBC NOTICE: Trajectory Initialized at (%ld)!\nTrajecory origin will base on (%f)\t(%f)\t(%f)\t(%f)\n",tick,(double)StartState.position.x,(double)StartState.position.y,(double)StartState.position.z,(double)StartState.attitude.yaw);
        return TrajIsInit=true;
    }
}

bool CB_planner_DisAbled(float z)
{
    if(TrajIsInit)
    {
        if(SoF==CB_Landing&&z<0.04f)
        {
            TrajIsInit=false;
            // StartState=(setpoint_t){.x=0.0f,.y=0.0f,.z=0.0f,.yaw=0.0f};
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

bool CB_planner_test()
{
    return TrajIsInit;
}


bool CB_plannerGetSetpoint(setpoint_t *setpoint,const state_t *state,uint32_t tick)
{
    float temp_t=GetRuntime(tick)-CruiseStartTime;
    switch (SoF)
    {
    case CB_Idel:
        /* code */
        break;
    case CB_TakeOff:
        setpoint->position.x=0.0f;
        setpoint->position.x=0.0f;
        setpoint->position.y=0.0f;
        setpoint->position.z=0.5f;
        setpoint->attitude.yaw=0.0f;
        
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
        
        setpoint->position.x=0.3f*(cosf(2*temp_t)-1.0f);
        setpoint->position.y=0.3f*sinf(2*temp_t);
        setpoint->position.z=0.5f;
        setpoint->acceleration.x=0.3f*2.0f*(-sinf(2*temp_t));
        setpoint->acceleration.y=0.3f*2.0f*cosf(2*temp_t);
        setpoint->acceleration.z=0.0f;
        setpoint->attitude.yaw=0.0f;        
        break;
    case CB_Landing:
        setpoint->position.x=0.0f;
        setpoint->position.y=0.0f;
        setpoint->position.z=0.5f*state->position.z;
        setpoint->attitude.yaw=0.0f;  
        break;
    default:
        break;
    }

    setpoint->position.x+=StartState.position.x;
    setpoint->position.y+=StartState.position.y;
    setpoint->position.z+=StartState.position.z;
    setpoint->attitude.yaw+=StartState.attitude.yaw;
    return true;
}

float GetRuntime(uint32_t tick)
{
    return runtime= (float)(tick-StartTime)/1000.0f;
}