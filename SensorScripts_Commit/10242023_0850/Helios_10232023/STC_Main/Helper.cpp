#include "Arduino.h"

int PWMDutyCycle(double percentDutyCycle) {
  return floor((percentDutyCycle / 100) * 255);
}

void SetPinModes(){
  // Set pinmode for Analog, Digital ports & initialize to 0 where appropriate
  //pinMode(LED_BUILTIN, OUTPUT); // LED Blink to confirm that i"m not insane & code actually uploaded
  // Above is conflicting with B4 declaration

  pinMode(A0, INPUT); // Thermocouple
  
  pinMode(A1, INPUT); // Pyranometer

  /* Azimuth/Tilt Control */
  //pinMode(A2, OUTPUT);  // EN21
  //digitalWrite(A2, LOW);
  ledcSetup(0, 5000, 8);
  ledcAttachPin(A2, 0);

  pinMode(A3, OUTPUT);  // F1
  digitalWrite(A3, LOW);

  pinMode(A4, OUTPUT);  // B1
  digitalWrite(A4, LOW);

  //pinMode(A5, OUTPUT);  // EN2
  //digitalWrite(A5, LOW);
  ledcSetup(2, 5000, 8);
  ledcAttachPin(A5, 2);

  // Since not using SPI, overwrite for GPIO control
  pinMode(SCK, OUTPUT);  // F2
  digitalWrite(SCK, LOW);

  pinMode(MOSI, OUTPUT);  // B2
  digitalWrite(MOSI, LOW);
  /* /Azimuth/Tilt Control */

  /* Shutter Control */
  //pinMode(6, OUTPUT); // EN3 (LEFT)
  //digitalWrite(6, LOW);
  ledcSetup(4, 5000, 8);
  ledcAttachPin(6, 4);

  pinMode(9, OUTPUT); // F3
  digitalWrite(9, LOW);

  pinMode(10, OUTPUT); // B3
  digitalWrite(10, LOW);

  //pinMode(11, OUTPUT); // EN4 (RIGHT)
  //digitalWrite(11, LOW);
  ledcSetup(6, 5000, 8);
  ledcAttachPin(11, 6);

  pinMode(12, OUTPUT); // F4
  digitalWrite(12, LOW);

  pinMode(13, OUTPUT); // B4
  digitalWrite(13, LOW);
  /* /Shutter Control */

}

double analogRead_Calibrated(int _pin){
  double rollingSum = 0;
  double numberOfEntries = 2;
  for(int i = 0; i < 10; i++){
    rollingSum += analogRead(_pin);
    numberOfEntries += 1;
  }
  return( rollingSum / numberOfEntries ); // From linear interpolation excel spreadsheet; calibrated from 0 to 3.3V (usually within +/- 0.25% accuracy (+/- 30mv, nominal))
}

double Compass_Angle(double Mag_X, double Mag_Y){
  float Pi = 3.14159;

  // Calculate the angle of the vector y,x
  float heading = ((atan2(Mag_Y, Mag_X) * 180) / Pi) - 90;

  //heading = heading - 82.8; //Calibration constant, where X-axis is facing due north
  // Normalize to 0-360
  if (heading < 0)
  {
    heading = 360 + heading;
  }

  Serial.print("Compass Heading: ");
  Serial.println(heading);
  return heading;
}