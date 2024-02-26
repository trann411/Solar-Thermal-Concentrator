/* ---------- *
Object file containing settings, variables, and functions used in conjunction with the concentrator's onboard rotary encoder(s)
* ---------- */

#include "Arduino.h"
#include "Helper.h"

/* Rotary Encoder Azimuth Angle Change & Increment Logic */
double RotaryEncode_AngleIncrement(char direction) {
  double angleDelta = 0;
  direction = toupper(direction);

  /* Record First State */
  double presA = digitalRead(MOSI);
  double presB = digitalRead(TX);
  double presAnot = digitalRead(RX);
  double presBnot = digitalRead(5);
  /* /Record First State */

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
    if (direction = 'F'){
      angleDelta += (360 / 4096);
    }
    if (direction = 'B'){
      angleDelta -= (360 / 4096);
    }
  }
  else{
    angleDelta = 0;
  }
  /* /If any states have changed, increment */
  
  return angleDelta;
}
/* /Rotary Encoder Azimuth Angle Change & Increment Logic */
