#include "Arduino.h"

int pastA = -1;
int pastB = -1;
int presA = -1;
int presB = -1;
char direction = 'F';
double angleDelta = 0;
double currentAngleTravel = 0;
double PPR = 96;  // Points per revolution, determined by 4 DIP Switches on encoder (on other side of connector)



void setup() {
  // put your setup code here, to run once:
  pinMode(TX, INPUT);  // TX in arduino maps to RTX on PCB
  pinMode(RX, INPUT);  // RX in arduino maps to TX on PCB

  /* Seed with initial value */
  presA = digitalRead(RX);  // Was accidently declared as mosi, be sure to change in STC_Main
  presB = digitalRead(TX);
}

void loop() {
  // put your main code here, to run repeatedly:

  /* Store previous values for comparison */
  pastA = presA;
  pastB = presB;

  /* Get new values */
  presA = digitalRead(RX);  // Was accidently declared as mosi, be sure to change in STC_Main
  presB = digitalRead(TX);

  /* Check to see if change and increment */
  if ((pastA != presA) || (pastB != presB)) {
    //Serial.println("DIFFERENCE DETECTED");
    //delay(2000);
    angleDelta += (90 / PPR);

  } else {
    angleDelta = 0;
  }

  /*
    // Tilting "Forward"
// State | A | B
// 0       0   0
// 1       1   0
// 2       1   1
// 3       0   1
// 4       0   0
  if ( (pastA == 0) && (pastB == 0) && (presA == 1) && (presB == 0) ) {
    angleDelta += (90 / PPR);
  }

  else if ( (pastA == 1) && (pastB == 0) && (presA == 1) && (presB == 1)) {
    angleDelta += (90 / PPR);
  }

  else if ( (pastA == 1) && (pastB == 1) && (presA == 0) && (presB == 1)) {
    angleDelta += (90 / PPR);
  }

  else if ( (pastA == 0) && (pastB == 1) && (presA == 0) && (presB == 0)) {
    angleDelta += (90 / PPR);
  }



  // Tilting "Backwards"
  // State | A | B
  // 0       0   0
  // 1       0   1
  // 2       1   1
  // 3       1   0
  // 4       0   0

  else if ( (pastA == 0) && (pastB == 0) && (presA == 0) && (presB == 1)) {
    angleDelta -= (90 / PPR);
  }

  else if ((pastA == 0) && (pastB == 1) && (presA == 1) && (presB == 1)) {
    angleDelta -= (90 / PPR);
  }

  else if ((pastA == 1) && (pastB == 1) && (presA == 1) && (presB == 0)) {
    angleDelta -= (90 / PPR);
  }

  else if ((pastA == 1) && (pastB == 0) && (presA == 0) && (presB == 0)) {
    angleDelta -= (90 / PPR);
  } 
  */

  currentAngleTravel += angleDelta;

  Serial.print(millis());
  Serial.print(", pastA = ");
  Serial.print(pastA);
  Serial.print(", pastB = ");
  Serial.print(pastB);
  Serial.print(", presA = ");
  Serial.print(presA);
  Serial.print(", presB = ");
  Serial.print(presB);
  Serial.print(", Degrees Traveled = ");
  Serial.println(currentAngleTravel);
}
