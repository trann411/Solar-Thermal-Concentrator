#include "Arduino.h"

int PWMDutyCycle(double percentDutyCycle) {
  return round((percentDutyCycle / 100) * 255);
}

void SetPinModes(){
  // Set pinmode for Analog, Digital ports & initialize to 0 where appropriate
  pinMode(LED_BUILTIN, OUTPUT); // LED Blink to confirm that i"m not insane & code actually uploaded
  
  pinMode(A0, INPUT); // Thermocouple
  
  pinMode(A1, INPUT); // Pyranometer

  /* Azimuth/Tilt Control */
  //pinMode(A2, OUTPUT);  // ENA1
  //digitalWrite(A2, LOW);
  ledcSetup(0, 5000, 8);
  ledcAttachPin(A2, 0);

  pinMode(A3, OUTPUT);  // IN1
  digitalWrite(A3, LOW);

  pinMode(A4, OUTPUT);  // IN2
  digitalWrite(A4, LOW);

  //pinMode(A5, OUTPUT);  // ENA2
  //digitalWrite(A5, LOW);
  ledcSetup(2, 5000, 8);
  ledcAttachPin(A5, 2);

  // Since not using SPI, overwrite for GPIO control
  pinMode(SCK, OUTPUT);  // IN3
  digitalWrite(SCK, LOW);

  pinMode(MOSI, OUTPUT);  // IN4
  digitalWrite(MOSI, LOW);
  /* /Azimuth/Tilt Control */

}

double analogRead_Calibrated(int _pin){
  double rollingSum = 0;
  double numberOfEntries = 2;
  for(int i = 0; i < 10; i++){
    rollingSum += analogRead(_pin);
    numberOfEntries += 1;
  }
  return( rollingSum / numberOfEntries ); // From linear interpolation excel spreadsheet; calibrated from 0 to 3.3V (usually within +/- 0.25% accuracy (+/- 30mv, nominal))
};