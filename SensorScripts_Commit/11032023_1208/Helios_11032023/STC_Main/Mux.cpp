#include "Arduino.h"
#include "Helper.h"
#include "Wire.h"

int I2CMux_Select(int i) {

  Wire.beginTransmission(0x70);
  Wire.write(1 << i);
  Wire.endTransmission();

  return(i);
}