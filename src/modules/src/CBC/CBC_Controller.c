// #include "controller.h"
#include "CBC_Controller.h"
#include "CBC_Vector.h"
#include "CBC_Quaternion.h"
#include "stabilizer_types.h"
#include "math3d.h"
#include "debug.h"
#include "math.h"

void CB_YawControl(CB_control_t *CB_control,CB_Vector3_t *directionF, const sensorData_t *sensors, const state_t *state,const uint32_t tick)
{
    CB_Vector3_t u1,u2,u3,u2t;
    u3=*directionF;
    u2t=(CB_Vector3_t){.x=-sin( desire_yaw ),.y=cos(desire_yaw),.z=0.0f};
	u1=Vector3Cross(u2t,u3);
    u1=Vector3Normalize(u1);
	u2=Vector3Cross(u3,u1);
	u1=Vector3Normalize(u1);
    // float R[3][3]={{u1.x,u2.x,u3.x},{u1.y,u2.y,u3.y},{u1.z,u2.z,u3.z}};
	quaternion_t desire_q= DCM2UnitQuat(u1,u2,u3);
	
	//qd * q*
	quaternion_t delta_q = quaternionDivide(desire_q,state->attitudeQuaternion);

	// if(tick%1000==0)
	// 	DEBUG_PRINT("(%f)\t(%f)\t(%f)\t(%f)\n",(double)state->attitudeQuaternion.w,(double)state->attitudeQuaternion.x,(double)state->attitudeQuaternion.y,(double)state->attitudeQuaternion.z);
	

	float wx=sensors->gyro.x*M_PI_F/180.0f;
	float wy=sensors->gyro.y*M_PI_F/180.0f;
	float wz=sensors->gyro.z*M_PI_F/180.0f;
	float r     = Kq_Roll   * delta_q.x + Kw_Roll   * (0.0f - wx);
	float p     = Kq_Pitch  * delta_q.y + Kw_Pitch  * (0.0f - wy);
	float y     = Kq_Yaw    * delta_q.z + Kw_Yaw    * (0.0f - wz);
	//w*Jw
	CB_Vector3_t w = {.x=wx,.y=wy,.z=wz};
	CB_Vector3_t Jw={	.x = wx*Ixx+wy*Ixy+wz*Ixz,
						.y = wx*Ixy+wy*Iyy+wz*Iyz,
						.z = wx*Ixz+wy*Iyz+wz*Izz};
	CB_Vector3_t WjW=Vector3Cross(w,Jw);

	//Tau=I*alpha
	CB_control->torque_roll =r*Ixx+p*Ixy+y*Ixz+WjW.x;
	CB_control->torque_pitch=r*Ixy+p*Iyy+y*Iyz+WjW.y;
	CB_control->torque_yaw  =r*Ixz+p*Iyz+y*Izz+WjW.z;
}


void CB_PositionControl(CB_control_t *CB_control,CB_Vector3_t *directionF, const state_t *state,const uint32_t tick)
{
	float x=Kp_X*(desire_x-state->position.x)+Kd_X*(0.0f-state->velocity.x);
	float y=Kp_Y*(desire_y-state->position.y)+Kd_Y*(0.0f-state->velocity.y);
	float z=Kp_Z*(desire_z-state->position.z)+Kd_Z*(0.0f-state->velocity.z);
  
	//Saturation   	
   	x = fminf(fmaxf(x,-MaxAcc),MaxAcc); 
   	y = fminf(fmaxf(y,-MaxAcc),MaxAcc);
   	z = fminf(fmaxf(z,-MaxAcc),MaxAcc);

     
	directionF->x=x;
	directionF->y=y;
	directionF->z=z+gravity;

	// this two step can not be exchange
	CB_control->thrust=Vector3Norm(*directionF)*mass;
	*directionF = Vector3Normalize(*directionF);
	
}

const float thrust;
const float roll;
const float pitch;
const float yaw;
void CB_Controller(CB_control_t *CB_control, setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state,const uint32_t tick)
{
		if(setpoint->thrust>2000.0f)
		{
			CB_Vector3_t DirectionF;
			CB_PositionControl(CB_control,&DirectionF,state,tick);
			CB_YawControl(CB_control,&DirectionF, sensors, state,tick);
			
		}
		else
		{
			CB_control->thrust=0;
			CB_control->torque_pitch=0;
			CB_control->torque_pitch=0;
			CB_control->torque_yaw=0;
		}
	
	
	
}
