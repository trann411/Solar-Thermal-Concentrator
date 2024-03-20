#include "Arduino.h"

int pastA = -1;
int pastB = -1;
int presA = -1;
int presB = -1;
String direction = "NULL";
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

  if ((pastA != presA) || (pastB != presB)) {
    //Serial.println("DIFFERENCE DETECTED");
    //delay(2000);
    if (direction == "Clock") {
      angleDelta += (90 / PPR);
    }
    if (direction == "Counter") {
      angleDelta -= (90 / PPR);
    }

  } else {
    angleDelta = 0;
  }

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
  Serial.print(", Degrees Traveled = ");
  Serial.println(currentAngleTravel);
}
