#include "Accel.h"
#include "AzimuthTiltController.h"
#include "Debug.h"
#include "Helper.h"
#include "Magno.h"
#include "Pyro.h"
#include "RotaryEncode.h"
#include "ShutterController.h"
#include "Thermo.h"

#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

/* Magnetometer Variables */
Adafruit_MMC5603 mag = Adafruit_MMC5603(12345);
/* /Magnetometer Variables */

/* Accelerometer Variables */
Adafruit_MMA8451 mma = Adafruit_MMA8451();
/* /Accelerometer Variables */

void setup() {
  Serial.begin(115200);
  delay(500);

  SetPinModes();

  /* Initialize Magnetometer */
  if (!mag.begin(MMC56X3_DEFAULT_ADDRESS, &Wire)) {  // I2C mode
    /* There was a problem detecting the MMC5603 ... check your connections */
    Serial.println("Ooops, no MMC5603 detected ... Check your wiring!");
    while (1) delay(10);
  }
  /* /Initialize Magnetometer */

  /* Initialize Accelerometer */
  if (!mma.begin()) {
    Serial.println("Couldnt start");
    while (1) delay(10);
  }
  Serial.println("MMA8451 found!");
  mma.setRange(MMA8451_RANGE_2_G);
  /* /Initialize Accelerometer */
}

void loop() {
  ATControl_PinConfigure('A', 'F', 25);
}
