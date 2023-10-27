#ifndef Accel_h
#define Accel_h

#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>


double Accel_Angle(Adafruit_MMA8451 mma);

#endif