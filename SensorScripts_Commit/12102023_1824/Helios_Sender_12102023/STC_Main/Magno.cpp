#include "Arduino.h"
#include "Helper.h"
#include <Adafruit_MMC56x3.h>

/* Calculate Compass Angle From & WRT Magnetometer */
double Magno_CompassSample(Adafruit_MMC5603 mag){
  sensors_event_t event;
  mag.getEvent(&event);
  
  double Mag_X = event.magnetic.x;
  double Mag_Y = event.magnetic.y;



  float heading = (atan2(Mag_Y, Mag_X)) * (180 / PI) - 90;

  // Normalize to 0-360
  if (heading < 0)
  {
    heading = 360 + heading; 
  }

  //Serial.print("Compass Heading: ");
  Serial.println(Mag_X);
  Serial.println(Mag_Y);
  Serial.println(heading);

  return(heading);
}
/* /Calculate Compass Angle From & WRT Magnetometer */
