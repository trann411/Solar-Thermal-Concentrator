#include "Arduino.h"
#include "Helper.h"

/* Rotary Encoder Azimuth Angle Change & Increment Logic */
double RotaryEncode_AngleIncrement() {
  double angleDelta = 0;

  /* Record First State */
  double presA = digitalRead(MOSI);
  double presB = digitalRead(TX);
  double presAnot = digitalRead(RX);
  double presBnot = digitalRead(5);
  /* /Record First State */

  delay(10); // Wait for next sample

  /* Record Second State & Push First State */
  double pastA = presA; 
  double pastB = presB;
  double pastAnot = presAnot;
  double pastBnot = presBnot;

  presA = digitalRead(MOSI);  // Sample current Values
  presB = digitalRead(TX);
  presAnot = digitalRead(RX);
  presBnot = digitalRead(5);
  /* /Record Second State & Push First State */

  /* If any states have changed, increment */
  if ((pastA != presA) || (pastB != presB) || (pastAnot != presAnot) || (pastBnot != presBnot)) {
    angleDelta += (360 / 4096);
  }
  /* /If any states have changed, increment */
  
  return angleDelta;
}
/* /Rotary Encoder Azimuth Angle Change & Increment Logic */
