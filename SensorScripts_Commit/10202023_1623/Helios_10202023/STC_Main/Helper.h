#ifndef Helper_h
#define Helper_h

#include "Arduino.h"

int PWMDutyCycle(double percentDutyCycle);
void SetPinModes();
double analogRead_Calibrated(int _pin);
double Compass_Angle(double Mag_X, double Mag_Y);

#endif