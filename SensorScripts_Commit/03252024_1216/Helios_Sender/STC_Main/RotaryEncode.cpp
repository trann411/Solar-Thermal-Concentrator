/* ---------- *
Object file containing settings, variables, and functions used in conjunction with the concentrator's onboard rotary encoder(s)
* ---------- */

#include "Arduino.h"
#include "Helper.h"

double RotaryEncode_Tracking(int numberOfSamples) {
  int pastA = -1;
  int pastB = -1;
  int presA = -1;
  int presB = -1;

  float analogpastA = -1;
  float analogpastB = -1;
  float analogpresA = -1;
  float analogpresB = -1;

  String direction = "NULL";
  double angleDelta = 0;
  double currentAngleTravel = 0;
  double PPR = 40480;  // Points per revolution, found value of 40647 experimentally and split the difference between two nearest states (40000 & 40960)
                       // Results in inaccuracy of +/- 0.000052734375, well-within bounds of +/- 0.1 degree accuracy of spec.
  bool isRotaryTargetReached;

  double samplesTaken = 0;
  double absSamplesTaken = 0;

  double currentAngle = 0;

  /* Seed Initial Values */
  isRotaryTargetReached = false;

  analogpresA = analogRead(RX);  // Was accidently declared as mosi, be sure to change in STC_Main
  analogpresB = analogRead(TX);

  if (analogpresA >= 2) {
    presA = 1;
  }
  if (analogpresA < 2) {
    presA = 0;
  }

  if (analogpresB >= 2) {
    presB = 1;
  }
  if (analogpresB < 2) {
    presB = 0;
  }

  /* /Seed Initial Values */

  while (isRotaryTargetReached == false) {
    /* Save Old Values & Get New Values */
    pastA = presA;
    pastB = presB;

    analogpresA = analogRead(RX);  // Was accidently declared as mosi, be sure to change in STC_Main
    analogpresB = analogRead(TX);

    if (analogpresA >= 2) {
      presA = 1;
    }
    if (analogpresA < 2) {
      presA = 0;
    }

    if (analogpresB >= 2) {
      presB = 1;
    }
    if (analogpresB < 2) {
      presB = 0;
    }
    /* /Save Old Values & Get New Values */

    /* Determine Direction of Rotation */
    // Tilting "Forward" (Encoder Turning Clockwise)
    // State | A | B
    // 0       0   0
    // 1       1   0
    // 2       1   1
    // 3       0   1
    // 4       0   0
    if ((pastA == 0) && (pastB == 0) && (presA == 1) && (presB == 0)) {
      direction = "Clock";
    }

    if ((pastA == 1) && (pastB == 0) && (presA == 1) && (presB == 1)) {
      direction = "Clock";
    }

    if ((pastA == 1) && (pastB == 1) && (presA == 0) && (presB == 1)) {
      direction = "Clock";
    }

    if ((pastA == 0) && (pastB == 1) && (presA == 0) && (presB == 0)) {
      direction = "Clock";
    }

    // Tilting "Backwards" (Encoder turning Counter-Clockwise)
    // State | A | B
    // 0       0   0
    // 1       0   1
    // 2       1   1
    // 3       1   0
    // 4       0   0

    if ((pastA == 0) && (pastB == 0) && (presA == 0) && (presB == 1)) {
      direction = "Counter";
    }

    if ((pastA == 0) && (pastB == 1) && (presA == 1) && (presB == 1)) {
      direction = "Counter";
    }

    if ((pastA == 1) && (pastB == 1) && (presA == 1) && (presB == 0)) {
      direction = "Counter";
    }

    if ((pastA == 1) && (pastB == 0) && (presA == 0) && (presB == 0)) {
      direction = "Counter";
    }
    /* /Determine Direction of Rotation */

    /* If Difference in States Detected: Increment */
    if ((pastA != presA) || (pastB != presB)) {
      //Serial.println("DIFFERENCE DETECTED");
      //delay(2000);
      if (direction == "Clock") {
        angleDelta -= (90 / PPR);
        samplesTaken -= 1;
      }
      if (direction == "Counter") {
        angleDelta += (90 / PPR);
        samplesTaken += 1;
      }
    } else {
      angleDelta = 0;
    }
    /* /If Difference in States Detected: Increment */

    /* Update Current Azimuth Position & Check if Need to Continue */
    currentAngle += angleDelta;
    absSamplesTaken = abs(samplesTaken);

    if(absSamplesTaken >= numberOfSamples){
      isRotaryTargetReached = true;
    }
    /* /Update Current Azimuth Position & Check if Need to Continue */

    /* Serial Debug Output *
    Serial.print(millis());
    Serial.print(", pastA = ");
    Serial.print(pastA);
    Serial.print(", pastB = ");
    Serial.print(pastB);
    Serial.print(", presA = ");
    Serial.print(presA);
    Serial.print(", presB = ");
    Serial.print(presB);
    Serial.print(", Direction of Rotation = ");
    Serial.print(direction);
    Serial.print(", Samples Taken = ");
    Serial.print(samplesTaken);
    Serial.print(", Number of desired samples = ");
    Serial.print(numberOfSamples);
    Serial.print(", absSamples Taken = ");
    Serial.print(absSamplesTaken);
    Serial.print(", Degrees Traveled = ");
    Serial.print(currentAngle);
    Serial.print(", isRotaryTargetReached = ");
    Serial.println(isRotaryTargetReached);
    /* /Serial Debug Output */
  }

  return(currentAngle); // It took me way too long to realize that the reason this function wouldn't stop is because it did not have
                        // a return statement. Thank you, Arduino IDE, for your invaluable error-checking and flagging contribution.
}

/* Rotary Encoder Azimuth Angle Change & Increment Logic *
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

/* If any states have changed, increment *
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
  /* /If any states have changed, increment *
}
/* /Rotary Encoder Azimuth Angle Change & Increment Logic */
