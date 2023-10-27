#ifndef AzimuthTiltController_h
#define AzimuthTiltController_h

#include "Arduino.h"
#include "Helper.h"
#include "Magno.h"


/* Target Motor & Direction Control Function */
void ATControl_PinConfigure(char targetMotor, char direction, double dutyCycle);

/* Azimuth Motor Control & Feedback Cycle (Magnetometer-Based) */
void ATControl_MagnoRotate(double targetAngle, Adafruit_MMC5603 mag, double dutyCycle);

#endif