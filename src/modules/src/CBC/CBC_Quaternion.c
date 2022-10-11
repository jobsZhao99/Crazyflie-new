#include "CBC_Quaternion.h"
// #include "CBC_Vector.h"
#include "debug.h"
#include "math.h"
void QuaternionNormalize(quaternion_t *q);
void QuaternionShortArc(quaternion_t *q)
{
    if(q->w<0.0f)
    {
        q->w*=-1.0f;
        q->x*=-1.0f;
        q->y*=-1.0f;
        q->z*=-1.0f;
    }
}

quaternion_t quaternionMultiply(quaternion_t q, quaternion_t p)
{
    quaternion_t Q;

	Q.w =   q.w * p.w - q.x * p.x - q.y * p.y - q.z * p.z;
	Q.x =   q.w * p.x + q.x * p.w + q.y * p.z - q.z * p.y;
	Q.y =   q.w * p.y - q.x * p.z + q.y * p.w + q.z * p.x;
	Q.z =   q.w * p.z + q.x * p.y - q.y * p.x + q.z * p.w;
    QuaternionNormalize(&Q);
	return Q;
}

quaternion_t quaternionMultiplyNoNormalize(quaternion_t q, quaternion_t p)
{
    quaternion_t Q;

	Q.w =   q.w * p.w - q.x * p.x - q.y * p.y - q.z * p.z;
	Q.x =   q.w * p.x + q.x * p.w + q.y * p.z - q.z * p.y;
	Q.y =   q.w * p.y - q.x * p.z + q.y * p.w + q.z * p.x;
	Q.z =   q.w * p.z + q.x * p.y - q.y * p.x + q.z * p.w;
	return Q;
}

/// @brief q-p
/// @param q 
/// @param p 
/// @return 
quaternion_t quaternionSubtract(quaternion_t q, quaternion_t p)
{
     quaternion_t Q;

	Q.w =   q.w-p.w;
	Q.x =   q.x-p.x;
	Q.y =   q.y-p.y;
	Q.z =   q.z-p.z;
    return Q;
}

quaternion_t quaternionConjugate(quaternion_t q)
{
    return (quaternion_t){.w=q.w,.x=-q.x,.y=-q.y,.z=-q.z};
}

quaternion_t quaternionDivide(quaternion_t q, quaternion_t p)
{
    QuaternionShortArc(&p);
	p.x*=-1.0f;
	p.y*=-1.0f;
	p.z*=-1.0f;
	return quaternionMultiply(q,p);
}

void QuaternionNormalize(quaternion_t *q)
{
    float norm = sqrtf( q->w*q->w+q->x*q->x+q->y*q->y+q->z*q->z);
    if (norm==0)
        DEBUG_PRINT("CBC Error! The magnitude of the Quaternion is 0 !");
    else if(norm!=1)
    {
        q->w/=norm;
        q->x/=norm;
        q->y/=norm;
        q->z/=norm;
    }
    QuaternionShortArc(q);
}
int FindMax(float a11,float a22,float a33);
quaternion_t DCM2UnitQuat(CB_Vector3_t u1,CB_Vector3_t u2,CB_Vector3_t u3)
{
	quaternion_t output;

    output.w= 0.5f*sqrtf(u1.x+u2.y+u3.z+1.0f); 
    if(output.w!=0.0f)
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
    
    
    QuaternionNormalize(&output);

	// DEBUG_PRINT("MOTOR_M (%f)\t(%f)\t(%f)\t(%f)\n", (double)u1.x,(double)u2.y,(double)u3.z,(double)output.w);

	return output;
}
quaternion_t QuaternionRotation(quaternion_t q,quaternion_t r)
{
    return quaternionDivide(quaternionMultiply(q,r),q);
    
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