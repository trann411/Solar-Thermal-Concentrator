#ifndef Thermo_h
#define Thermo_h

#include "Arduino.h"
#include "Helper.h"

/* Read in Analog Amp. Board Voltage & Convert to Temperature (Celcius) */
double Thermo_ReadTemp(int _pin);

#endif