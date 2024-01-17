#ifndef Helper_h
#define Helper_h

#include "Arduino.h"

/* Conversion of Percent Input to 16-Bit Number */
int PWMDutyCycle(double percentDutyCycle);

/* Master Function Containing All pinMode Setup Functions (Initializes LOW) */
void SetPinModes();

/* Thermocouple AnalogRead Function Intended to Eliminate Error */
double analogRead_Calibrated(int _pin);

#endif