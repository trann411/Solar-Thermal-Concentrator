#ifndef Magnetometer_h
#define Magnetometer_h

#include "Arduino.h"
#include "Helper.h"
#include <Adafruit_MMC56x3.h>


void Magnetometer_Setup(Adafruit_MMC5603 mmc);
void Magnetometer_Read(Adafruit_MMC5603 mmc);

#endif