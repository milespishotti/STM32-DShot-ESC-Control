#ifndef INC_CONTROLLER_H_
#define INC_CONTROLLER_H_

#include "main.h"

void Controller_Init(void);
void Controller_UpdateSensors(void);
uint16_t Controller_ComputeThrottle(void);

extern float controller_angle;
extern float controller_error;
extern float controller_gyro;
extern float controller_integral_term;
extern float controller_correction;

#endif
