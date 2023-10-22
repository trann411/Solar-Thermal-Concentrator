#include "Arduino.h"
#include "Helper.h"
#include <Adafruit_MMC56x3.h>

double Magno_CompassSample(Adafruit_MMC5603 mag){
  sensors_event_t event;
  mag.getEvent(&event);
  
  double Mag_X = event.magnetic.x;
  double Mag_Y = event.magnetic.y;

  Serial.print("Mag_X:");
  Serial.println(Mag_X);
  
  Serial.print("Mag_Y:");
  Serial.println(Mag_Y);

  return(Compass_Angle(Mag_X, Mag_Y));
}