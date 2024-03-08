/* ---------- *
Object file containing settings, variables, and functions used in conjunction with the concentrator's onboard shutter controller(s)
* ---------- */

#ifndef ShutterController_h
#define ShutterController_h

#include "Arduino.h"
#include "Helper.h"

/* Target Actuator & Direction Control Function */
void SHControl_PinConfigure(char targetMotor, char direction, double dutyCycle);

#endif