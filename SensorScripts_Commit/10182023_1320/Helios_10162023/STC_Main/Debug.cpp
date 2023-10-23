#include "Arduino.h"
#include "AzimuthTiltController.h"
#include "Helper.h"

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
