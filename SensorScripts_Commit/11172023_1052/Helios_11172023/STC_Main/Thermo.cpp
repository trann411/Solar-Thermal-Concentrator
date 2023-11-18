#include "arduino.h"
#include "Helper.h"

/* Read in Analog Amp. Board Voltage & Convert to Temperature (Celcius) */
double Thermo_ReadTemp(int _pin){
  // NEED TO ADJUST FOR ERROR IN A2D CONVERTER (See helper)
  return ( ((analogRead_Calibrated(_pin)) - 1250) / 5 );
}
/* /Read in Analog Amp. Board Voltage & Convert to Temperature (Celcius) */
