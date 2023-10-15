#include "arduino.h"
#include "Helper.h"

double Thermocouple_ReadTemp(int _pin){
  // NEED TO ADJUST FOR ERROR IN A2D CONVERTER (See helper)
  return ( ((analogRead_Calibrated(_pin)) - 1250) / 5 );
}