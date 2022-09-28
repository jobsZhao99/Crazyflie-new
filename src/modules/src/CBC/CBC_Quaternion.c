#include "CBC_Quaternion.h"
// #include "CBC_Vector.h"
#include "debug.h"
#include "math.h"
quaternion_t quaternionMultiply(quaternion_t q, quaternion_t p)
{
	float w =   q.w * p.w - q.x * p.x - q.y * p.y - q.z * p.z;
	float x =   q.w * p.x + q.x * p.w + q.y * p.z - q.z * p.y;
	float y =   q.w * p.y - q.x * p.z + q.y * p.w + q.z * p.x;
	float z =   q.w * p.z + q.x * p.y - q.y * p.x + q.z * p.w;
	return (quaternion_t){.w = w,.x = x, .y = y, .z = z};
}


quaternion_t quaternionDivide(quaternion_t q, quaternion_t p)
{
	p.x*=-1;
	p.y*=-1;
	p.z*=-1;
	return quaternionMultiply(q,p);
}

quaternion_t QuaternionNormalize(quaternion_t q)
{
    float norm = sqrtf( q.w*q.w+q.x*q.x+q.y*q.y+q.z*q.z);
    if (norm==0)
        DEBUG_PRINT("CBC Error! The magnitude of the Quaternion is 0 !");
    else if(norm!=1)
    {
        q.w/=norm;
        q.x/=norm;
        q.y/=norm;
        q.z/=norm;
    }
    return q;
}
int FindMax(float a11,float a22,float a33);
quaternion_t DCM2UnitQuat(CB_Vector3_t u1,CB_Vector3_t u2,CB_Vector3_t u3)
{
	quaternion_t output;

    output.w= 0.5f*sqrtf(u1.x+u2.y+u3.z+1.0f); 
    if(output.w>1e-6f||output.w<-1e-6f)
    {
    output.x=(u2.z-u3.y)/(4.0f*output.w);
    output.y=(u3.x-u1.z)/(4.0f*output.w);
    output.z=(u1.y-u2.x)/(4.0f*output.w);
    }
    else
    {
        float t;
        switch (FindMax(u1.x,u2.y,u3.z))
        {
        case 1:
            t=sqrt(1+u1.x-u2.y-u3.z);
            output.w=(u2.z-u3.y)/t;
            output.x=t/4;
            output.y=(u3.x+u1.z)/t;
            output.w=(u2.x+u1.y)/t;
            break;
        case 2:
            t=sqrt(1-u1.x+u2.y-u3.z);
            output.w=(u3.x-u1.z)/t;
            output.x=(u2.x+u1.y)/t;            
            output.y=t/4;
            output.w=(u2.z+u3.y)/t;
            break;
        case 3:
            t=sqrt(1-u1.x-u2.y+u3.z);
            output.w=(u1.y-u2.x)/t;
            output.x=(u3.x+u1.z)/t;            
            output.y=(u3.y-u2.z)/t;
            output.w=t/4;
            break;
        default:
            break;
        }
    }
    
    
    output=QuaternionNormalize(output);

	// DEBUG_PRINT("MOTOR_M (%f)\t(%f)\t(%f)\t(%f)\n", (double)u1.x,(double)u2.y,(double)u3.z,(double)output.w);

	return output;
}

int FindMax(float a11,float a22,float a33)
{
    if(a11>a22)
    {
        if (a11>a33)
        {
            return 1;
        }
        else
            return 3;
        
    }
    else
    {
          if (a22>a33)
        {
            return 2;
        }
        else
            return 3;
    }
}