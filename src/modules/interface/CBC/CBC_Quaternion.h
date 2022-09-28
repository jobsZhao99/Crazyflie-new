
#ifndef _CB_QUATERNION_H_
#define _CB_QUATERNION_H_
#include "stabilizer_types.h"
#include "CBC_Vector.h"
quaternion_t quaternionMultiply(quaternion_t q, quaternion_t p);
quaternion_t quaternionDivide(quaternion_t q, quaternion_t p);
quaternion_t QuaternionNormalize(quaternion_t q);

quaternion_t DCM2UnitQuat(CB_Vector3_t u1,CB_Vector3_t u2,CB_Vector3_t u3);
#endif