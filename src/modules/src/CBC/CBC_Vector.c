#include "CBC_Vector.h"

float Vector3Norm(CB_Vector3_t input)
{
	return sqrtf(input.x*input.x+input.y*input.y+input.z*input.z);
}


// output=VA*VB
CB_Vector3_t Vector3Cross(CB_Vector3_t VA,CB_Vector3_t VB)
{
	CB_Vector3_t output;
	output.x=VA.y*VB.z-VA.x*VB.y;
	output.y=VA.z*VB.x-VA.x*VB.z;
	output.z=VA.x*VB.y-VA.y*VB.x;
	return output;
}

CB_Vector3_t Vector3Normalize(CB_Vector3_t input)
{

	float norm=Vector3Norm(input);
	input.x/=norm;
	input.y/=norm;
	input.z/=norm;
	return input;
}

