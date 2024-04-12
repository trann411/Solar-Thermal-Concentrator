/* ---------- *
Object file containing settings, variables, and functions used in conjunction with the concentrator's onboard magnotometer(s).
  Mostly deprecated since magnotometer is no longer used.
* ---------- */

#ifndef Magno_h
#define Magno_h

#include "Arduino.h"
#include "Helper.h"
#include "Kalman.h"
#include <Adafruit_MMC56x3.h>

/* Calculate Compass Angle From & WRT Magnetometer */
double Magno_CompassSample(Adafruit_MMC5603 mag);

/* Post-Processing to Derive More Accurate Reading from Noisy Angle Data */
void Magno_StartKalmanFilter(Adafruit_MMC5603 mag);

float Magno_KalmanFilter(float magnoCompassAngle, float timeBetweenSamples);

#endif