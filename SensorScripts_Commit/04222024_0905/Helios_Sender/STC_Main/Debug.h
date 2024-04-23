/* ---------- *
Object file containing settings, variables, and functions used in conjunction with the debugging other concentrator module's code
  Primarily used only for ECEN 403 and are effectively antiquated.
* ---------- */

#ifndef Debug_h
#define Debug_h

#include "Arduino.h"
#include "AzimuthTiltController.h"
#include "Helper.h"

/* Debug Pattern Cycling through Azimuth/Tilt Control States */
void Debug_Blink_ATControl();

/* Debug Pattern Cycling through Shutter Control States */
void Debug_Blink_SHControl();

/* Debug Pattern Blinking Onboard Feather LED */
void Debug_Blink_Comm();

/* Sweep Input to Find Minimum Duty Cycle of Azimuth Motor */
void Debug_Azimuth_MinDutyCycle(double dutyCycleToTest);

#endif