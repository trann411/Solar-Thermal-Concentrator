#ifndef Helper_h
#define Helper_h

#include "Arduino.h"

int PWMDutyCycle(double percentDutyCycle);
void SetPinModes();
double analogRead_Calibrated(int _pin);

#endif