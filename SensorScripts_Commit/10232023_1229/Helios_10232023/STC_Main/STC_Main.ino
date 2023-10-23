#include "AzimuthTiltController.h"
#include "Debug.h"
#include "Helper.h"
#include "Magno.h"
#include "Thermocouple.h"
#include <Wire.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>
#include "Accel.h"
#include "ShutterController.h"

/* Magnetometer Variables */
Adafruit_MMC5603 mag = Adafruit_MMC5603(12345);
/* /Magnetometer Variables */

/* Accelerometer Variables */
Adafruit_MMA8451 mma = Adafruit_MMA8451();
/* /Accelerometer Variables */


void setup() {
  Serial.begin(115200);
  delay(500);

  SetPinModes();  // Set pins to input/output & intialize to LOW
  
  /* Initialize Magnetometer */
  if (!mag.begin(MMC56X3_DEFAULT_ADDRESS, &Wire)) {  // I2C mode
    /* There was a problem detecting the MMC5603 ... check your connections */
    Serial.println("Ooops, no MMC5603 detected ... Check your wiring!");
    while (1) delay(10);
  }
  /* /Initialize Magnetometer */


  /* Initialize Accelerometer */
    if (! mma.begin()) {
      Serial.println("Couldnt start");
      while (1) delay (10);
    }
    Serial.println("MMA8451 found!");
    mma.setRange(MMA8451_RANGE_2_G);
  /* /Initialize Accelerometer */

}

void loop() {

  /* Sample Magnetometer *
  sensors_event_t event;
  mag.getEvent(&event);
  
  Mag_X = event.magnetic.x;
  Mag_Y = event.magnetic.y;
  /* /Sample Magnetometer */

  /* Sample Accelerometer 
    // Read the 'raw' data in 14-bit counts
  mma.read();

  //Get a new sensor event 
  sensors_event_t event; 
  mma.getEvent(&event);

  /* Display the results (acceleration is measured in m/s^2)  
  double x = event.acceleration.x;
  double y = event.acceleration.y;
  double z = event.acceleration.z;
  Serial.print("X: \t"); Serial.print(x); Serial.print("\t");
  Serial.print("Y: \t"); Serial.print(y); Serial.print("\t");
  Serial.print("Z: \t"); Serial.print(z); Serial.print("\t");
  Serial.println("m/s^2 ");

  double angle = atan( (sqrt( (x*x) + (y*y))) / z ) * (180 / PI);
  Serial.print("Angle (wrt gravity): ");
  Serial.print(angle);
  Serial.println(" degrees");
   /Sample Accelerometer */

  /* Debug Console */
  //Thermocouple_ReadTemp(A0);
  //Serial.println(Thermocouple_ReadTemp(A0));
  //delay(250);

  //Debug_Blink_Comm();  // Blink RED LED by USB port
  //Debug_Blink_ATControl(); // Cycle through DROK control pulses
  //Compass_Angle(Mag_X, Mag_Y);

  //ATControl_PinConfigure('A', 'F'); //SET PWM PULSE TO 40 for lin act
  //ATControl_Rotate(90, mag);
  /* /Debug Console */

  //Accel_Angle(mma);
  
  Debug_Blink_SHControl();
  
  delay(250);
}
