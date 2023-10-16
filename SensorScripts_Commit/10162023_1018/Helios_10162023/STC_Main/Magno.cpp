#include "Arduino.h"
#include "Helper.h"
#include <Adafruit_MMC56x3.h>

double Magno_CompassSample(Adafruit_MMC5603 mag){
  sensors_event_t event;
  mag.getEvent(&event);
  
  double Mag_X = event.magnetic.x;
  double Mag_Y = event.magnetic.y;

  return(Compass_Angle(Mag_X, Mag_Y));
}