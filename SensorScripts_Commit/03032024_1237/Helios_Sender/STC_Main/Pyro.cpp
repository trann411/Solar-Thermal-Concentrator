/* ---------- *
Object file containing settings, variables, and functions used in conjunction with the concentrator's onboard pyronometer(s)
* ---------- */


#include "arduino.h"
#include "Helper.h"

/* Read in Pyranometer Voltage & Convert to Temperature (Celcius) */
double Pyro_ReadTemp(int _pin){
  // NEED TO ADJUST FOR ERROR IN A2D CONVERTER (See helper)
  return ( ((analogRead_Calibrated(_pin)) - 1250) / 5 );
}
/* /Read in Pyranometer Board Voltage & Convert to Temperature (Celcius) */
