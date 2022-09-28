
#ifndef _CBC_CONTROLLER_H_
#define _CBC_CONTROLLER_H_
#include "stabilizer_types.h"

// #define kp 36
// #define kd 12

#define desire_yaw 0.0f
#define desire_x 0.0f
#define desire_y 0.0f
#define desire_z 0.5f

#define kp 36
#define kd 12
#define kq 800.0f
#define kw 60.0f

// position
#define Kp_X kp
#define Kp_Y kp
#define Kp_Z kp
//velocity
#define Kd_X kd
#define Kd_Y kd
#define Kd_Z kd
//attitude
#define Kq_Roll kq
#define Kq_Pitch kq
#define Kq_Yaw kq
//angular velocity in body frame
#define Kw_Roll kw
#define Kw_Pitch  kw
#define Kw_Yaw kw

#define mass 0.032f
#define gravity 9.81f

#define MaxAcc 0.5f*gravity
// #define MaxZAcc 15.0f




//Inertia
#define Ixx 1.6571710e-5f
#define Iyy 1.6655602e-5f
#define Izz 2.9261652e-5f
#define Ixy 0.830806e-6f
#define Ixz 0.718277e-6f
#define Iyz 1.800197e-6f

typedef struct CB_control_s {
  float torque_roll;
  float torque_pitch;
  float torque_yaw;
  float thrust;
}CB_control_t;


void CB_Controller(CB_control_t *CB_control, setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state,const uint32_t tick);
#endif