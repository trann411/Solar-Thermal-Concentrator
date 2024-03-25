/* ---------- *
Object file containing settings, variables, and functions used in conjunction with the concentrator's onboard tilt, azimuth motor controller(s)
* ---------- */

#ifndef AzimuthTiltController_h
#define AzimuthTiltController_h

#include "Arduino.h"
#include "Helper.h"
#include "Magno.h"
#include "Accel.h"
#include "RotaryEncode.h"
#include "Pyro.h"

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>


/* Target Motor & Direction Control Function */
void ATControl_PinConfigure(char targetMotor, char direction, double dutyCycle);

/* Azimuth Motor Control & Feedback Cycle (Magnetometer-Based) */
double ATControl_MagnoRotate(double targetAngle, double presentAngle);

void ATControl_AccelRotate(double targetAngle, Adafruit_MMA8451 mma2, Adafruit_MMA8451 mma3, Adafruit_MMA8451 mma4, int desiredStage);

double ATControl_PyroSweep(int _pyroPin, double presentAngle);

#endif