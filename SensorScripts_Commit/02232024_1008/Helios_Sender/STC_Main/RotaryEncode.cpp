/* ---------- *
Object file containing settings, variables, and functions used in conjunction with the concentrator's onboard rotary encoder(s)
* ---------- */

#include "Arduino.h"
#include "Helper.h"

/* Rotary Encoder Azimuth Angle Change & Increment Logic */
int[] RotaryEncoder_RecordPulse(){
  int presA = digitalRead(MOSI);
  int presB = digitalRead(TX);
  int presAnot = digitalRead(RX);
  int presBnot = digitalRead(5);
  int[] pulseStates = [presA, presB, presAnot, presBnot];

  return(pulseStates);
}

double RotaryEncode_AngleIncrement(char direction, int[] presState, int[] lastState, double currentAngle) {
  double angleDelta = 0;
  direction = toupper(direction);
  /* /Record Second State & Push First State */

  /* If any states have changed, increment */
  int pastA = lastState[0];
  int pastB = lastState[1];
  int pastAnot = lastState[2];
  int pastb = lastState[3];

  int presA = lastState[0];
  int presB = lastState[1];
  int pastAnot = lastState[2];
  int pastb = lastState[3];


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

  currentAngle += angleDelta

  return(currentAngle);
  /* /If any states have changed, increment */
}
/* /Rotary Encoder Azimuth Angle Change & Increment Logic */
