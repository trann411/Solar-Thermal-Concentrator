#include "Helper.h"
#include "Thermocouple.h"
#include "Pyranometer.h"
#include "AzimuthTiltController.h"
#include "Debug.h"

void setup() {
  // put your setup code here, to run once:
  SetPinModes();

  digitalWrite(LED_BUILTIN, LOW); //Debug Interface
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:

  double temp = Thermocouple_ReadTemp(A0);
  Serial.println(temp);

  Debug_Blink_Comm();
  Debug_Blink_ATControl();
}
