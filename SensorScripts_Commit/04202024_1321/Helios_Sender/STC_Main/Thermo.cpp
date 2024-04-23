/* ---------- *
Object file containing settings, variables, and functions used in conjunction with the concentrator's onboard thermocouple(s)
* ---------- */

#include "arduino.h"
#include "Helper.h"

/* Read in Analog Amp. Board Voltage & Convert to Temperature (Celcius) */
double Thermo_ReadTemp(int _pin){
  float temp = ((analogRead_Calibrated(_pin)) - 1250) / 5; // Some error in A2D converter, but thermocouple inherently has wide error margin anyways.
  Serial.println(temp);
  return ( temp );
}
/* /Read in Analog Amp. Board Voltage & Convert to Temperature (Celcius) */
