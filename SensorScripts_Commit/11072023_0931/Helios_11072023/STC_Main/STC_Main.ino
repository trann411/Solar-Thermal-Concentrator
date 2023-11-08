//* Internal Scripts */
#include "Accel.h"
#include "AzimuthTiltController.h"
#include "Debug.h"
#include "Helper.h"
#include "Magno.h"
#include "Pyro.h"
#include "RotaryEncode.h"
#include "ShutterController.h"
#include "Thermo.h"
/* /Internal Scripts */

/* External Libraries */
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
/* /External Libraries */

#define TCAADDR 0x70


void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}


/* Magnetometer Variables */
Adafruit_MMC5603 mag = Adafruit_MMC5603(12345);
/* /Magnetometer Variables */

/* Accelerometer Variables */
Adafruit_MMA8451 mma2 = Adafruit_MMA8451(2);
Adafruit_MMA8451 mma3 = Adafruit_MMA8451(3);
Adafruit_MMA8451 mma4 = Adafruit_MMA8451(4);
/* /Accelerometer Variables */

double angle = 0;

/* Code to Run on Initialization */
void setup() {
  Serial.begin(115200);
  delay(500);

  SetPinModes();

  /* Initialize Magnetometer */
  if (!mag.begin(MMC56X3_DEFAULT_ADDRESS, &Wire)) {  // I2C mode
    /* There was a problem detecting the MMC5603 ... check your connections */
    Serial.println("Ooops, no MMC5603 detected ... Check your wiring!");
    
  }
  /* /Initialize Magnetometer */


  /* Initialize Accelerometer */
  tcaselect(2);
  if (!mma2.begin()) {
    while(1){
      Serial.println("MMA2 Couldnt start");
      delay(1000);
    }
  }
  Serial.println("MMA8451 (2) found!");
  mma2.setRange(MMA8451_RANGE_2_G);

  tcaselect(3);
  if (!mma3.begin()) {
    while(1){
      Serial.println("MMA3 Couldnt start");
      delay(1000);
    }
  }
  Serial.println("MMA8451 (3) found!");
  mma3.setRange(MMA8451_RANGE_2_G);

  tcaselect(4);
  if (!mma4.begin()) {
    while(1){
      Serial.println("MMA4 Couldnt start");
      delay(1000);
    }
  }
  Serial.println("MMA8451 (4) found!");
  mma4.setRange(MMA8451_RANGE_2_G);

  /* /Initialize Accelerometer */
}
/* /Code to Run on Initialization */

/* Code to Run continuously */
void loop() {
  //angle += RotaryEncode_AngleIncrement('F');
  //Serial.println(angle);
  //Magno_CompassSample(mag);
  //Accel_TiltAngle(mma3);
  //ATControl_PinConfigure('A', 'B', 35);
  Serial.println(Thermo_ReadTemp(A0));
  delay(100);
}
/* /Code to Run continuously */