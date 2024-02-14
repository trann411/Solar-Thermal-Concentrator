/* ---------- *
Object file containing settings, variables, and functions used in conjunction with the concentrator's onboard magnotometer(s)
* ---------- */

#ifndef Magno_h
#define Magno_h

#include "Arduino.h"
#include "Helper.h"
#include <Adafruit_MMC56x3.h>

/* Calculate Compass Angle From & WRT Magnetometer */
double Magno_CompassSample(Adafruit_MMC5603 mag);

#endif