/* ---------- *
Object file containing settings, variables, and functions used in conjunction with the concentrator's onboard rotary encoder(s)
* ---------- */

#ifndef RotaryEncode_h
#define RotaryEncode_h

#include "Arduino.h"
#include "Helper.h"

/* Rotary Encoder Azimuth Angle Change & Increment Logic */
double RotaryEncode_Tracking(double initialAngle, double targetAngle);

#endif