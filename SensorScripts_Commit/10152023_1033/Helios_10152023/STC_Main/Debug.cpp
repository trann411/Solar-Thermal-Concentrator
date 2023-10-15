#include "Arduino.h"
#include "AzimuthTiltController.h"
#include "Helper.h"
#include "Wire.h"

void Debug_Blink_ATControl(){
  ATControl_PinConfigure('a', 'f');
  //Serial.println("Motor: A, Direction: F");
  delay(500);

  ATControl_PinConfigure('a', 'b');
  //Serial.println("Motor: A, Direction: B");
  delay(500);

  ATControl_PinConfigure('t', 'f');
  //Serial.println("Motor: T, Direction: F");
  delay(500);

  ATControl_PinConfigure('t', 'b');
  //Serial.println("Motor: T, Direction: B");
  delay(500);
}

void Debug_Blink_Comm(){
  //Serial.println("LED ON");
  digitalWrite(LED_BUILTIN, LOW);
  delay(200);
  
  digitalWrite(LED_BUILTIN, HIGH);
  delay(200);
  //Serial.println("LED OFF");
}

void Debug_I2C_Comm(){
  byte error, address;
  int nDevices;
  Serial.println("Scanning...");
  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
      nDevices++;
    }
    else if (error==4) {
      Serial.print("Unknow error at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  }
  else {
    Serial.println("done\n");
  }
  delay(5000);      
}