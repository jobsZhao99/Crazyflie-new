#ifndef __CBC_POWER_DISTRIBUTION_H__
#define __CBC_POWER_DISTRIBUTION_H__

#include "stabilizer_types.h"
#include "CBC_Controller.h"

bool CB_powerDistributionTest(void);
void CB_powerDistribution(motors_thrust_t* motorPower, const CB_control_t *control);

#endif //__POWER_DISTRIBUTION_H__
