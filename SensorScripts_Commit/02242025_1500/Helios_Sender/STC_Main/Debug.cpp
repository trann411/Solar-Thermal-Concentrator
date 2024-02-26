/* ---------- *
Object file containing settings, variables, and functions used in conjunction with the debugging other concentrator module's code
* ---------- */

#include "Arduino.h"
#include "AzimuthTiltController.h"
#include "Helper.h"
#include "ShutterController.h"

/* Debug Pattern Cycling through Azimuth/Tilt Control States */
void Debug_Blink_ATControl(){
  ATControl_PinConfigure('a', 'f', 50);
  //Serial.println("Motor: A, Direction: F");
  delay(500);

  ATControl_PinConfigure('a', 'b', 50);
  //Serial.println("Motor: A, Direction: B");
  delay(500);

  ATControl_PinConfigure('t', 'f', 50);
  //Serial.println("Motor: T, Direction: F");
  delay(500);

  ATControl_PinConfigure('t', 'b', 50);
  //Serial.println("Motor: T, Direction: B");
  delay(500);
}
/* /Debug Pattern Cycling through Azimuth/Tilt Control States */


/* Debug Pattern Cycling through Shutter Control States */
void Debug_Blink_SHControl(){
  SHControl_PinConfigure('l', 'f', 50);
  //Serial.println("Motor: L, Direction: F");
  delay(500);

  SHControl_PinConfigure('l', 'b', 50);
  //Serial.println("Motor: L, Direction: B");
  delay(500);

  SHControl_PinConfigure('r', 'f', 50);
  //Serial.println("Motor: R, Direction: F");
  delay(500);

  SHControl_PinConfigure('r', 'b', 50);
  //Serial.println("Motor: R, Direction: B");
  delay(500);
} // With direct battery input, min. is 25%
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


/* Sweep Input to Find Minimum Duty Cycle of Azimuth Motor */
void Debug_Azimuth_MinDutyCycle(double dutyCycleToTest){
  int max = dutyCycleToTest / 5;
  
  for (int i = 0; i < max; i++){
    Serial.print("Testing Duty Cycle of: ");
    Serial.println(dutyCycleToTest);
    ATControl_PinConfigure('A', 'F', dutyCycleToTest);
    delay(7000);
    ATControl_PinConfigure('A', 'S', dutyCycleToTest);
    delay(3000);

    dutyCycleToTest -= 5;
  }
  
}
/* /Sweep Input to Find Minimum Duty Cycle of Azimuth Motor */
