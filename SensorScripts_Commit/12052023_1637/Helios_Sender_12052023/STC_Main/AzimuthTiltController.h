#ifndef AzimuthTiltController_h
#define AzimuthTiltController_h

#include "Arduino.h"
#include "Helper.h"
#include "Magno.h"
#include "Accel.h"

#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>


/* Target Motor & Direction Control Function */
void ATControl_PinConfigure(char targetMotor, char direction, double dutyCycle);

/* Azimuth Motor Control & Feedback Cycle (Magnetometer-Based) */
void ATControl_MagnoRotate(double targetAngle, Adafruit_MMC5603 mag, double dutyCycle);

void ATControl_AccelRotate(double targetAngle, Adafruit_MMA8451 mma2, Adafruit_MMA8451 mma3, Adafruit_MMA8451 mma4, double dutyCycle);
#endif