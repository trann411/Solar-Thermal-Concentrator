/* ---------- *
Object file containing settings, variables, and functions used in conjunction with the concentrator's onboard pyronometer(s)
* ---------- */


#include "arduino.h"
#include "Helper.h"

/* Read in Pyranometer Voltage & Convert to Temperature (Celcius) */
double Pyro_ReadTemp(int _pin){
  // NEED TO ADJUST FOR ERROR IN A2D CONVERTER (See helper)
  double pyroIn = analogRead(_pin);
  pyroIn = (pyroIn / 4095) * 3300;
  return (pyroIn);
}
/* /Read in Pyranometer Board Voltage & Convert to Temperature (Celcius) */
