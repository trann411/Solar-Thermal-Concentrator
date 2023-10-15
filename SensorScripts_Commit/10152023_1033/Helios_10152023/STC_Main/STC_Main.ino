#include "Helper.h"
#include "Thermocouple.h"
#include "Pyranometer.h"
#include "AzimuthTiltController.h"
#include "Debug.h"
#include "Magnetometer.h"
#include <Adafruit_MMC56x3.h>

Adafruit_MMC5603 mmc = Adafruit_MMC5603(12345);

void setup() {
  // put your setup code here, to run once:

  Magnetometer_Setup(mmc);

  SetPinModes(); // Set pins to input/output & intialize to LOW
  Serial.begin(115200);
  while(!Serial){
    delay(10);
  }
}

void loop() {
  // put your main code here, to run repeatedly:

  //double temp = Thermocouple_ReadTemp(A0);
  //Serial.println(temp);
  //delay(250);

  Debug_Blink_Comm(); // Blink RED LED by USB port
  Debug_Blink_ATControl(); // Cycle through DROK control pulses
  Debug_I2C_Comm(); // Query what devices are detected on I2C bus
  
}
