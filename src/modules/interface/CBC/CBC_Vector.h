#ifndef _CB_Vector_H_
#define _CB_Vector_H_
#include "stabilizer_types.h"
#include "num.h"
#include "math3d.h"


typedef struct CB_Vector3_s 
{
  float x;
  float y;
  float z;
} CB_Vector3_t;
float Vector3Norm(CB_Vector3_t input);


// output=VA*VB
CB_Vector3_t Vector3Cross(CB_Vector3_t VA,CB_Vector3_t VB);
CB_Vector3_t Vector3Normalize(CB_Vector3_t input);
#endif