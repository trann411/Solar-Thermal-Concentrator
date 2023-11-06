#include "Arduino.h"
#include "Helper.h"
#include <Adafruit_MMC56x3.h>

/* Calculate Compass Angle From & WRT Magnetometer */
double Magno_CompassSample(Adafruit_MMC5603 mag){
  sensors_event_t event;
  mag.getEvent(&event);
  
  double Mag_X = event.magnetic.x;
  double Mag_Y = event.magnetic.y;

  //Serial.print("Mag_X:");
  //Serial.println(Mag_X);
  
  //Serial.print("Mag_Y:");
  //Serial.println(Mag_Y);

  float heading = ((atan2(Mag_Y, Mag_X) * 180) / PI) -;

  //heading -= 82.8; //Calibration constant, where X-axis is facing due north
  // Normalize to 0-360
  if (heading < 0)
  {
    heading = 360 + heading; 
  }

  Serial.print("Compass Heading: ");
  Serial.println(heading);

  return(heading);
}
/* /Calculate Compass Angle From & WRT Magnetometer */
