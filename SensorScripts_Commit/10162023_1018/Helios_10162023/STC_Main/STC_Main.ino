#include "AzimuthTiltController.h"
#include "Debug.h"
#include "Helper.h"
#include "Magno.h"
#include "Thermocouple.h"


/* Magnetometer Variables */
Adafruit_MMC5603 mag = Adafruit_MMC5603(12345);
double Mag_X = 0;
double Mag_Y = 0;
/* Magnetometer Variables */

void setup() {
  // put your setup code here, to run once:
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

}

void loop() {

  /* Sample Magnetometer *
  sensors_event_t event;
  mag.getEvent(&event);
  
  Mag_X = event.magnetic.x;
  Mag_Y = event.magnetic.y;
  /* /Sample Magnetometer */

  /* Debug Console */
  //double temp = Thermocouple_ReadTemp(A0);
  //Serial.println(temp);
  //delay(250);

  //Debug_Blink_Comm();  // Blink RED LED by USB port
  //Debug_Blink_ATControl(); // Cycle through DROK control pulses
  //Compass_Angle(Mag_X, Mag_Y);

  ATControl_Rotate(180, mag);

  /* /Debug Console */
  
  
  delay(50);
}
