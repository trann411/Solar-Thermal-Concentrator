/* ---------- *
Object file containing settings, variables, and functions used in conjunction with the concentrator's onboard tilt, azimuth motor controller(s)
* ---------- */

#ifndef AzimuthTiltController_h
#define AzimuthTiltController_h

#include "Arduino.h"
#include "Helper.h"


/* Target Motor & Direction Control Function */
void ATControl_PinConfigure(char targetMotor, char direction, double dutyCycle);

#endif