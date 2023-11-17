#ifndef Accel_h
#define Accel_h

#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>

#define TCAADDR 0x70

/* MUX Pin Select Function */
void tcaselect(uint8_t i);

/* Calculate Tilt Angle of Single Accelerometer (Z-axis WRT Gravity) */
double Accel_TiltAngle(Adafruit_MMA8451 mma, double MUX_Channel); 

/* Calculate Tilt Angle from Three-Module Array of Accelerometers */
double Accel_ArrayTiltAngle(Adafruit_MMA8451 mma);

/* Post-Processing to Derive More Accurate Reading from Noisy Angle Data */
double Accel_KalmannFilter(double tiltArrayAngle);

#endif