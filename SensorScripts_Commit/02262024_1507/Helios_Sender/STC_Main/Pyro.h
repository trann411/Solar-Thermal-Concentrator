/* ---------- *
Object file containing settings, variables, and functions used in conjunction with the concentrator's onboard pyronometer(s)
* ---------- */

#ifndef Pyro_h
#define Pyro_h

#include "Arduino.h"
#include "Helper.h"

/* Read in Analog Amp. Board Voltage & Convert to Temperature (Celcius) */
double Pyro_ReadTemp(int _pin);

#endif