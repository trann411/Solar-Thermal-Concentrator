#ifndef ShutterController_h
#define ShutterController_h

#include "Arduino.h"
#include "Helper.h"


void SHControl_PWMPulse(int channel, int dutyCycle);
void SHControl_PinConfigure(char targetMotor, char direction);

#endif