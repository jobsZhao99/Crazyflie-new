// #include "controller.h"
#include "CBC_Controller.h"
#include "CBC_planner.h"
#include "CBC_Vector.h"
#include "CBC_Quaternion.h"
#include "stabilizer_types.h"
#include "math3d.h"
#include "math.h"
#include "debug.h"

static CB_Vector3_t DirectionF;
void CB_YawControl(CB_control_t *CB_control, setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const uint32_t tick)
{

	float desire_yaw = setpoint->attitude.yaw;
	CB_Vector3_t u1, u2, u3, u2t;
	u3 = DirectionF;
	u2t = (CB_Vector3_t){.x = -sin(desire_yaw), .y = cos(desire_yaw), .z = 0.0f};
	u1 = Vector3Cross(u2t, u3);
	Vector3Normalize(&u1);
	u2 = Vector3Cross(u3, u1);
	Vector3Normalize(&u2);
	// float R[3][3]={{u1.x,u2.x,u3.x},{u1.y,u2.y,u3.y},{u1.z,u2.z,u3.z}};
	quaternion_t desire_q = DCM2UnitQuat(u1, u2, u3);
	quaternion_t real_q = state->attitudeQuaternion;
	setpoint->attitudeQuaternion = desire_q;

	// q / qd
	quaternion_t delta_q = quaternionMultiply(quaternionConjugate(real_q), desire_q);
	QuaternionShortArc(&delta_q);

	// //qd * q*
	// quaternion_t delta_q = quaternionDivide(desire_q,real_q);
	// QuaternionShortArc(&delta_q);
	// delta_q=QuaternionRotation(quaternionConjugate(real_q),delta_q);

	float wx = sensors->gyro.x * M_PI_F / 180.0f;
	float wy = sensors->gyro.y * M_PI_F / 180.0f;
	float wz = sensors->gyro.z * M_PI_F / 180.0f;


    setpoint->attitudeRate.roll = 0.0f;
    setpoint->attitudeRate.pitch = 0.0f;
	// lastTime=tick;

	float r = Kq_Roll * delta_q.x + Kw_Roll * (setpoint->attitudeRate.roll - wx);
	float p = Kq_Pitch * delta_q.y + Kw_Pitch * (setpoint->attitudeRate.pitch - wy);
	float y = Kq_Yaw * delta_q.z + Kw_Yaw * (setpoint->attitudeRate.yaw - wz);
	// w*Jw
	CB_Vector3_t w = {.x = wx, .y = wy, .z = wz};
	CB_Vector3_t Jw = {.x = wx * Ixx + wy * Ixy + wz * Ixz,
					   .y = wx * Ixy + wy * Iyy + wz * Iyz,
					   .z = wx * Ixz + wy * Iyz + wz * Izz};
	CB_Vector3_t WjW = Vector3Cross(w, Jw);

	// Tau=I*alpha
	CB_control->roll = r * Ixx + p * Ixy + y * Ixz + WjW.x;
	CB_control->pitch = r * Ixy + p * Iyy + y * Iyz + WjW.y;
	CB_control->yaw = r * Ixz + p * Iyz + y * Izz + WjW.z;
}

void CB_PositionControl(CB_control_t *CB_control, setpoint_t *setpoint, const state_t *state, const uint32_t tick)
{
	float x = Kp_X * (setpoint->position.x - state->position.x) + Kd_X * (setpoint->velocity.x - state->velocity.x) + setpoint->acceleration.x;
	float y = Kp_Y * (setpoint->position.y - state->position.y) + Kd_Y * (setpoint->velocity.y - state->velocity.y) + setpoint->acceleration.y;
	float z = Kp_Z * (setpoint->position.z - state->position.z) + Kd_Z * (setpoint->velocity.z - state->velocity.z) + setpoint->acceleration.z;

	// Saturation
	x = fminf(fmaxf(x, -MaxAcc), MaxAcc);
	y = fminf(fmaxf(y, -MaxAcc), MaxAcc);
	z = fminf(fmaxf(z, -MaxAcc), MaxAcc);

	DirectionF.x = x;
	DirectionF.y = y;
	DirectionF.z = z + gravity;
	// this two step can not be exchange
	CB_control->thrust = Vector3Norm(DirectionF) * mass;
	Vector3Normalize(&DirectionF);
}

void CB_Controller(CB_control_t *CB_control, setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const uint32_t tick)
{


	// Emergency stop
	if (setpoint->attitude.pitch != 0||fabs (state->attitude.roll)>70.0||fabs(state->attitude.pitch)>70.0)
	{
		CB_planner_EmergencyStop();
		CB_control->thrust = 0.0f;
		CB_control->roll = 0.0f;
		CB_control->pitch = 0.0f;
		CB_control->yaw = 0.0f;
		return;
	}

	if (setpoint->thrust > 15000.0f || CB_planner_test())
	{

		// Initial CB Trajectory
		if (!CB_planner_test())
		{

			// CB_State_t tempState={.x=state->position.x,.y=state->position.y,.z=state->position.z,.yaw=(state->attitude.yaw/180.0f*M_PI_F)};
			CB_planner_Init(tick, *state);
		}

		if (setpoint->thrust < 2000.f)
		{
			CB_planner_DisAbled(*state,tick);
		}
		CB_plannerGetSetpoint(setpoint, state, tick); ////need change
		CB_PositionControl(CB_control, setpoint, state, tick);
		CB_YawControl(CB_control, setpoint, sensors, state, tick);
	}
	else
	{
		// start Landing
		if (CB_planner_test())
		{
			CB_planner_DisAbled(*state,tick);
		}
		CB_control->thrust = 0.0f;
		CB_control->roll = 0.0f;
		CB_control->pitch = 0.0f;
		CB_control->yaw = 0.0f;
	}

}
