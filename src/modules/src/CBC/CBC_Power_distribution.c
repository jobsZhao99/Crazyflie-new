#include "CBC_Power_distribution.h"
#include "CBC_Controller.h"
#include <string.h>
#include "log.h"
#include "param.h"
#include "num.h"
#include "autoconf.h"
#include "config.h"
#include "math.h"

#ifndef CONFIG_MOTORS_DEFAULT_IDLE_THRUST
#  define DEFAULT_IDLE_THRUST 0
#else
#  define DEFAULT_IDLE_THRUST CONFIG_MOTORS_DEFAULT_IDLE_THRUST
#endif

static uint32_t idleThrust = DEFAULT_IDLE_THRUST;



bool CB_powerDistributionTest(void)
{
  bool pass = true;
  return pass;
}

#define limitThrust(VAL) limitUint16(VAL)

void CB_powerDistribution(motors_thrust_t* motorPower, const CB_control_t *control)
{


  	if(control->thrust==0)
	{
		motorPower->m1=0;
		motorPower->m2=0;
		motorPower->m3=0;
		motorPower->m4=0;
	}
	else
	{

		
		float a=0.25f,b=7.7f,c=41.9143f;
		float term1=a*control->thrust;
		float term2=b*control->roll;
		float term3=b*control->pitch;
		float term4=c*control->yaw;

		motorPower->m1=limitThrust(4.0f*(term1-term2-term3-term4)/(60.0f*9.81f*0.001f)*65535);
		motorPower->m2=limitThrust(4.0f*(term1-term2+term3+term4)/(60.0f*9.81f*0.001f)*65535);
		motorPower->m3=limitThrust(4.0f*(term1+term2+term3-term4)/(60.0f*9.81f*0.001f)*65535);
		motorPower->m4=limitThrust(4.0f*(term1+term2-term3+term4)/(60.0f*9.81f*0.001f)*65535);

		motorPower->m1=fmax(motorPower->m1,idleThrust);
		motorPower->m2=fmax(motorPower->m2,idleThrust);
		motorPower->m3=fmax(motorPower->m3,idleThrust);
		motorPower->m4=fmax(motorPower->m4,idleThrust);
    
	}
}

