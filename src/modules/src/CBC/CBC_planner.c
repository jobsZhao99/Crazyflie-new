#include "CBC_planner.h"
#include "math.h"
#include "debug.h"
#include "stabilizer_types.h"
#include "math3d.h"
static float TimeRate = RATE_MAIN_LOOP;
static bool TrajIsInit = false;
static uint32_t StartTime;
// State of Start
static state_t State_Start;

// State of Land
static state_t State_Land;
/// @brief state of Drone
static CB_StateOfFliht_t SoF = CB_Idel;
float LandingStartTime;
float r = 0.3f;

static float runtime;
/// @brief
/// @param tick
/// @param state Start state
/// @return
bool CB_planner_Init(uint32_t tick, state_t state)
{
    if (TrajIsInit)
    {
        DEBUG_PRINT("CBC WARNING! Drone is flying in last trajector, could not Start new Trajecory!\n");
        return TrajIsInit = false;
    }
    else
    {
        State_Start = state;
        State_Start.attitude.yaw *= M_PI_F / 180.0f;
        StartTime = tick;
        TimeRate = RATE_MAIN_LOOP;
        SoF = CB_TakeOff;
        DEBUG_PRINT("CBC NOTICE: Trajectory Initialized at (%ld)!\nTrajecory origin will base on (%f)\t(%f)\t(%f)\t(%f)\n", tick, (double)State_Start.position.x, (double)State_Start.position.y, (double)State_Start.position.z, (double)State_Start.attitude.yaw);
        return TrajIsInit = true;
    }
}

bool CB_planner_DisAbled(state_t state, uint32_t tick)
{
    if (TrajIsInit)
    {
        if (SoF == CB_Cruise)
        {
            State_Land = state;
            LandingStartTime = GetRuntime(tick);
            SoF = CB_Landing;
            return false;
        }
        else if (SoF == CB_Landing && state.position.z < (0.04f+State_Start.position.z))
        {
            TrajIsInit = false;
            // State_Start=(setpoint_t){.x=0.0f,.y=0.0f,.z=0.0f,.yaw=0.0f};
            DEBUG_PRINT("CBC NOTICE: Trajectory is disabled!\n");
            SoF = CB_Idel;
            return true;
        }
        else
        {
            SoF = CB_Landing;
            return false;
        }
    }
    else
    {
        DEBUG_PRINT("CBC NOTICE: Try to disable rajectory fail!\n");
        return false;
    }
}

void CB_planner_EmergencyStop()
{
    TrajIsInit = false;
}

float Direction2Yaw(const state_t *state)
{

    float DesireX = -r + State_Start.position.x - state->position.x;
    float DesireY = 0 + State_Start.position.y - state->position.y;
    return atan2f(DesireY, DesireX);
}

bool CB_planner_test()
{
    return TrajIsInit;
}
float ToF;
float expCoeff;
bool CB_plannerGetSetpoint(setpoint_t *setpoint, const state_t *state, uint32_t tick)
{

    // initialize Value
    setpoint->position.x = State_Start.position.x;
    setpoint->position.y = State_Start.position.y;
    setpoint->position.z = State_Start.position.z;
    setpoint->attitude.yaw = State_Start.attitude.yaw + setpoint->attitude.roll;

    // velocity
    setpoint->velocity.x = 0.0f;
    setpoint->velocity.y = 0.0f;
    setpoint->velocity.z = 0.0f;
    setpoint->attitudeRate.yaw = 0.0f;

    // accelaration
    //  setpoint->acceleration.x=0.3f*4.0f*(-cosf(2.0f*ToF));
    setpoint->acceleration.x = 0.0f;
    setpoint->acceleration.y = 0.0f;
    setpoint->acceleration.z = 0.0f;
    ToF = GetRuntime(tick) - CruiseStartTime;
    float w = 1.0f;
    switch (SoF)
    {
    case CB_Idel:
        /* code */
        break;
    case CB_TakeOff:

        // position
        expCoeff = expf(-(ToF + CruiseStartTime));
        setpoint->position.z += HoF * (1.0f - expCoeff);
        setpoint->velocity.z = HoF * expCoeff;
        if (ToF < -(CruiseStartTime / 10.0f))
        {
            // setpoint->attitude.yaw = State_Start.attitude.yaw * (1.0f - expCoeff);
            // setpoint->attitudeRate.yaw = State_Start.attitude.yaw * expCoeff;
        }
        else if (ToF < 0.0f)
        {
            setpoint->attitude.yaw = State_Start.attitude.yaw * (1.0f - expCoeff);
            setpoint->attitudeRate.yaw = State_Start.attitude.yaw * expCoeff;
        }
        else if (ToF > 0.0f)
        {
            SoF = CB_Cruise;
            // DEBUG_PRINT("system is from takeoff to cruise\n");
        }
        break;
    case CB_Cruise:

        // position
        setpoint->position.x += r * (cosf(w * ToF) - 1.0f);
        setpoint->position.y += r * sinf(w * ToF);
        setpoint->position.z += HoF;
        setpoint->attitude.yaw = ToF * w;

        // velocity
        setpoint->velocity.x = r * w * (-sinf(w * ToF));
        setpoint->velocity.y = r * w * (cosf(w * ToF));
        setpoint->attitudeRate.yaw = w;

        // accelaration
        setpoint->acceleration.x = r * w * w * (-cosf(w * ToF));
        setpoint->acceleration.y = r * w * w * (-sinf(w * ToF));

        break;
    case CB_Landing:
        setpoint->position.z += 0.1f*(state->position.z -State_Start.position.z);
        setpoint->attitude.yaw = State_Land.attitude.yaw;
        setpoint->position.x=State_Land.position.x;
        setpoint->position.y=State_Land.position.y;
        break;
    default:
        break;
    }

    float temp = setpoint->attitude.yaw;

    while (temp > M_PI_F)
    {
        temp -= (2.0f * M_PI_F);
    }
    while (temp < -M_PI_F)
    {
        temp += (2.0f * M_PI_F);
    }

    setpoint->attitude.yaw = temp;
    return true;
}

float GetRuntime(uint32_t tick)
{
    return runtime = (float)(tick - StartTime) / 1000.0f;
}
