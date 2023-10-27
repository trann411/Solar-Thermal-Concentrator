#include "Arduino.h"
#include "AzimuthTiltController.h"
#include "Helper.h"
#include "ShutterController.h"

/* Debug Pattern Cycling through Azimuth/Tilt Control States */
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
/* /Debug Pattern Cycling through Azimuth/Tilt Control States */


/* Debug Pattern Cycling through Shutter Control States */
void Debug_Blink_SHControl(){
  SHControl_PinConfigure('l', 'f');
  //Serial.println("Motor: L, Direction: F");
  delay(500);

  SHControl_PinConfigure('l', 'b');
  //Serial.println("Motor: L, Direction: B");
  delay(500);

  SHControl_PinConfigure('r', 'f');
  //Serial.println("Motor: R, Direction: F");
  delay(500);

  SHControl_PinConfigure('r', 'b');
  //Serial.println("Motor: R, Direction: B");
  delay(500);
}
/* /Debug Pattern Cycling through Shutter Control States */


/* Debug Pattern Blinking Onboard Feather LED */
void Debug_Blink_Comm(){
  //Serial.println("LED ON");
  digitalWrite(LED_BUILTIN, LOW);
  delay(200);
  
  digitalWrite(LED_BUILTIN, HIGH);
  delay(200);
  //Serial.println("LED OFF");
}
/* /Debug Pattern Blinking Onboard Feather LED */
