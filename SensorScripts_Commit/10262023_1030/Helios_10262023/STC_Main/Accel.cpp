#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>

/* Calculate Tilt Angle of Single Accelerometer (Z-axis WRT Gravity) */
double Accel_TiltAngle(Adafruit_MMA8451 mma) {
  mma.read(); // Read the 'raw' data in 14-bit counts

  sensors_event_t event; //Get a new sensor event 
  mma.getEvent(&event);

  /* Display the results (acceleration is measured in m/s^2)  */
  double x = event.acceleration.x;
  double y = event.acceleration.y;
  double z = event.acceleration.z;
  double angle = atan( (sqrt( (x*x) + (y*y))) / z ) * (180 / PI);

  Serial.print("X: \t"); Serial.print(x); Serial.print("\t");
  Serial.print("Y: \t"); Serial.print(y); Serial.print("\t");
  Serial.print("Z: \t"); Serial.print(z); Serial.print("\t");
  Serial.println("m/s^2 ");
  
  Serial.print("Angle (wrt gravity): ");
  Serial.print(angle);
  Serial.println(" degrees");
  /* /Display the results (acceleration is measured in m/s^2)  */
  
  return(angle);
}
/* /Calculate Tilt Angle of Single Accelerometer (Z-axis WRT Gravity) */
