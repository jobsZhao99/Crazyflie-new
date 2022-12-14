#ifndef _CBC_CONTROLLER_H_
#define _CBC_CONTROLLER_H_
#include "stabilizer_types.h"
#define kp 11.0f
#define kd 8.0f
#define kq 600.0f
#define kw 55.0f
// #define kp 0.0f
// #define kd 0.0f
// #define kq 0.0f
// #define kw 0.0f
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

#define mass 0.036f
#define gravity 9.81f
// #define gravity 0.0f
#define MaxAcc 0.5f*gravity
// #define MaxZAcc 15.0f




//Inertia
#define Ixx 1.6571710e-5f
#define Iyy 1.6655602e-5f
#define Izz 2.9261652e-5f
#define Ixy 0.830806e-6f
#define Ixz 0.718277e-6f
#define Iyz 1.800197e-6f
// #define CBHZ 100


void CB_Controller(CB_control_t *CB_control, setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state,const uint32_t tick);
quaternion_t quaternionSubtract(quaternion_t q, quaternion_t p);
#endif