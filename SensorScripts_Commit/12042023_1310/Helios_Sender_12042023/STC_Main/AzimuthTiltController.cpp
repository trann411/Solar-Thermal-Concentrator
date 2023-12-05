#include "Arduino.h"
#include "Helper.h"
#include "Magno.h"
#include "Accel.h"

#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

/* Target Motor & Direction Control Function */
void ATControl_PinConfigure(char targetMotor, char direction, double dutyCycle) {  // Duty Cycle (25% to 100%)
  targetMotor = toupper(targetMotor);
  direction = toupper(direction);

  /* Force Minimum & Maximum Duty Cycles */
  if (dutyCycle < 25) {
    dutyCycle = 25;
  }

  if (dutyCycle > 100) {
    dutyCycle = 100;
  }
  /* Force Minimum & Maximum Duty Cycles */

  if (targetMotor == 'A') {
    // Set ENA1 high, ENA2 low; break before make to prevent magic smoke
    digitalWrite(SCK, LOW);
    digitalWrite(MOSI, LOW);

    if (direction == 'F') {
      // SET IN1/IN3 high, IN2/IN4 low; break before make to prevent magic smoke
      digitalWrite(A4, LOW);
      digitalWrite(A3, HIGH);
    }

    if (direction == 'B') {
      // SET IN1/IN3 high, IN2/IN4 low; break before make to prevent magic smoke
      digitalWrite(A3, LOW);
      digitalWrite(A4, HIGH);
    }
    ledcWrite(2, PWMDutyCycle(0));
    ledcWrite(0, PWMDutyCycle(dutyCycle));  // PWM Pulse Channel & Percentage Duration

    if (direction == 'S') {
      digitalWrite(A3, LOW);
      digitalWrite(A4, LOW);
      ledcWrite(0, PWMDutyCycle(0));
    }
  }

  if (targetMotor == 'T') {
    // Set ENA2 high, ENA1 low; break before make to prevent magic smoke
    digitalWrite(A3, LOW);
    digitalWrite(A4, LOW);

    if (direction == 'F') {
      // SET IN1/IN3 high, IN2/IN4 low; break before make to prevent magic smoke
      digitalWrite(MOSI, LOW);
      digitalWrite(SCK, HIGH);
    }

    if (direction == 'B') {
      // SET IN1/IN3 high, IN2/IN4 low; break before make to prevent magic smoke
      digitalWrite(SCK, LOW);
      digitalWrite(MOSI, HIGH);
    }

    ledcWrite(0, PWMDutyCycle(0));
    ledcWrite(2, PWMDutyCycle(dutyCycle));

    if (direction == 'S') {
      digitalWrite(MOSI, LOW);
      digitalWrite(SCK, LOW);
      ledcWrite(2, PWMDutyCycle(0));
    }
  }
}
/* /Target Motor & Direction Control Function */


/* Azimuth Motor Control & Feedback Cycle (Magnetometer-Based) */
void ATControl_MagnoRotate(double targetAngle, Adafruit_MMC5603 mag, double dutyCycle) {  // NEED TO ADDRESS EDGE CASES AROUND 0 DEG TARGET ANGLE

  double currentAngle = Magno_CompassSample(mag);
  double angleBounds[] = { targetAngle + 5, targetAngle - 5 };
  bool isTargetReached = false;

  if (angleBounds[0] >= 360) {
    angleBounds[0] -= 360;
  }

  if (angleBounds[1] < 0) {
    angleBounds[1] += 360;
  }

  while (!isTargetReached) {
    // Serial.print("Max Angle:");
    // Serial.println(angleBounds[0]);
    // Serial.print("Min Angle:");
    // Serial.println(angleBounds[1]);

    currentAngle = Magno_CompassSample(mag);

    if (currentAngle <= angleBounds[0]) {
      if (currentAngle >= angleBounds[1]) {
        ATControl_PinConfigure('a', 's', dutyCycle);
        isTargetReached = true;
      }
    }

    if (currentAngle > angleBounds[0]) {
      ATControl_PinConfigure('a', 'b', dutyCycle);
    }
    
    if (currentAngle < angleBounds[1]) {
      ATControl_PinConfigure('a', 'f', dutyCycle);
    }

    delay(10);
  }
  Serial.println("REACHED TARGET ANGLE");
}
/* /Azimuth Motor Control & Feedback Cycle (Magnetometer-Based) */

/* Azimuth Motor Control & Feedback Cycle (Magnetometer-Based) */
void ATControl_AccelRotate(double targetAngle, Adafruit_MMA8451 mma2, Adafruit_MMA8451 mma3, Adafruit_MMA8451 mma4, double dutyCycle) {  // NEED TO ADDRESS EDGE CASES AROUND 0 DEG TARGET ANGLE

  double currentAngle = Accel_ArrayTiltAngle(mma2, mma3, mma4);
  double angleBounds[] = { targetAngle + 0.5, targetAngle - 0.5 };
  bool isTargetReached = false;

  if (angleBounds[0] >= 360) {
      angleBounds[0] -= 360;
    }

    if (angleBounds[1] < 0) {
      angleBounds[1] += 360;
      
    }

  Serial.println("Beginning feedback tilt");

  while (!isTargetReached) {

    currentAngle = Accel_ArrayTiltAngle(mma2, mma3, mma4);


    if (currentAngle <= angleBounds[0]) {
      if (currentAngle >= angleBounds[1]) {
        ATControl_PinConfigure('t', 's', dutyCycle);
        isTargetReached = true;
      }
    }

    if (currentAngle > angleBounds[0]) {
      Serial.println("Tilt back");
      ATControl_PinConfigure('t', 'b', dutyCycle);
    } 

    if (currentAngle < angleBounds[1]) {
      Serial.println("Tilt back");
      ATControl_PinConfigure('t', 'f', dutyCycle);
    }

    delay(10);
  }
}
/* /Azimuth Motor Control & Feedback Cycle (Magnetometer-Based) */
