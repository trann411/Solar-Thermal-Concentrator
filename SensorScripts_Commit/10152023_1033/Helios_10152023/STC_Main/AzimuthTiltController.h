#ifndef AzimuthTiltController_h
#define AzimuthTiltController_h

#include "Arduino.h"
#include "Helper.h"


void ATControl_PWMPulse(int channel, int dutyCycle);
void ATControl_PinConfigure(char targetMotor, char direction);
void ATControl_Rotate(char targetMotor, char direction, double targetAngle);

#endif