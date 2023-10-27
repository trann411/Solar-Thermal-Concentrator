#ifndef Accel_h
#define Accel_h

#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>

/* Calculate Tilt Angle of Single Accelerometer (Z-axis WRT Gravity) */
double Accel_TiltAngle(Adafruit_MMA8451 mma); 

#endif