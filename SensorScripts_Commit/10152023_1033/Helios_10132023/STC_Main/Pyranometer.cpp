#include "arduino.h"

double Pyranometer_ReadLight(int _pin){
  return ((analogRead(_pin))); //add conversion formula);
}