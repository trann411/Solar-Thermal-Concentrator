/* ---------- *
Object file containing settings, variables, and functions used in conjunction with the concentrator's onboard tilt, azimuth motor controller(s)
* ---------- */

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

  float currentAngle;
  float angleToInput;

  Serial.println("Beginning feedback tilt");

  /* Stage Zero [Check if Valid Target] */
  if (targetAngle > 90) {  // Force min and max angles
    targetAngle = 90;
  }
  if (targetAngle < 0) {
    targetAngle = 0;
  }

  // Populate start angle
  float startAngle = Accel_ArrayTiltAngle(mma2, mma3, mma4);  // Seed angleToReturn for bandpass filter

  // Start Kalmanfilter
  Accel_StartKalmanFilter(mma2, mma3, mma4);

  // Calculate angle bounds
  double stageOneAngleBounds[] = { targetAngle + 10, targetAngle - 10 };
  double stageTwoAngleBounds[] = { targetAngle + 1, targetAngle - 1 };
  double stageThreeAngleBounds[] = { targetAngle + 0.1, targetAngle - 0.1 };

  bool isStageOneTargetReached = false;
  bool isStageTwoTargetReached = false;
  bool isStageThreeTargetReached = false;

  double stageOneDutyCycle = 70;
  double stageTwoDutyCycle = 52;
  double stageThreeDutyCycle = 35;

  int stageOneCounter = 0;  // Track number of "good" values read in from kalman filter
  int stageTwoCounter = 0;  // before permitting control loop to end.
  int stageThreeCounter = 0;

  float stageOneLastAngle = 0;
  float stageTwoLastAngle = 0;
  float stageThreeLastAngle = 0;

  float startTime = millis();
  float currentTime;
  float timeElapsed;

  /* /Stage Zero [Check if Valid Target] */

  /* Stage One [Fast Rotate] */
  while (isStageOneTargetReached == false) {
    // Calculate current angle

    currentTime = millis();
    timeElapsed = (currentTime - startTime) / 1000.0000;


    for (int i = 0; i < 5; ++i) {  //Take multiple samples to avoid skew by bad value
      angleToInput = Accel_ArrayTiltAngle(mma2, mma3, mma4);
      currentAngle = Accel_KalmanFilter(angleToInput, timeElapsed);

      // Debug Output
      Serial.print(currentTime);
      Serial.print(",");
      Serial.print(angleToInput);
      Serial.print(",");
      Serial.println(currentAngle);
    }

    // Save for next iteration of loop
    startTime = currentTime;

    // If within target bounds, stop & move to next stage
    if (currentAngle <= stageOneAngleBounds[0]) {
      if (currentAngle >= stageOneAngleBounds[1]) {
        ATControl_PinConfigure('t', 's', stageOneDutyCycle);
        stageOneCounter += 1;
        if (stageOneCounter >= 100) {  // If 100 good values read in from kalmanfilter, allow to pass on to next stage
          isStageOneTargetReached = true;
        }
      }
    }

    // If greater angle than target bound max, tilt forwards
    if (currentAngle > stageOneAngleBounds[0]) {
      //Serial.println("Tilt forwards");
      ATControl_PinConfigure('t', 'f', stageOneDutyCycle);
      stageOneCounter = 0;  // Reset good samples counter
    }

    // If lesser angle than target bound min, tilt backwards
    if (currentAngle < stageOneAngleBounds[1]) {
      //Serial.println("Tilt back");
      ATControl_PinConfigure('t', 'b', stageOneDutyCycle);
      stageOneCounter = 0;  // Reset good samples counter
    }
  }

  delay(1000);  // Wait for everything to settle
  /* /Stage One [Fast Rotate] */

  /* Stage Slow [Slow Rotate] */
  while (isStageTwoTargetReached == false) {
    // Calculate current angle

    currentTime = millis();
    timeElapsed = (currentTime - startTime) / 1000.0000;

    for (int i = 0; i < 5; ++i) {  //Take multiple samples to avoid skew by bad value
      angleToInput = Accel_ArrayTiltAngle(mma2, mma3, mma4);
      currentAngle = Accel_KalmanFilter(angleToInput, timeElapsed);

      // Debug Output
      Serial.print(currentTime);
      Serial.print(",");
      Serial.print(angleToInput);
      Serial.print(",");
      Serial.println(currentAngle);
    }

    // Save for next iteration of loop
    startTime = currentTime;


    // If within target bounds, stop & move to next stage
    if (currentAngle <= stageTwoAngleBounds[0]) {
      if (currentAngle >= stageTwoAngleBounds[1]) {
        ATControl_PinConfigure('t', 's', stageTwoDutyCycle);
        stageTwoCounter += 1;
        if (stageTwoCounter >= 100) {  // If 100 good values read in from kalmanfilter, allow to pass on to next stage
          isStageTwoTargetReached = true;
        }
      }
    }

    // If greater angle than target bound max, tilt forwards
    if (currentAngle > stageTwoAngleBounds[0]) {
      //Serial.println("Tilt forwards");
      ATControl_PinConfigure('t', 'f', stageTwoDutyCycle);
      stageTwoCounter = 0;  // Reset good samples counter
    }

    // If lesser angle than target bound min, tilt backwards
    if (currentAngle < stageTwoAngleBounds[1]) {
      //Serial.println("Tilt back");
      ATControl_PinConfigure('t', 'b', stageTwoDutyCycle);
      stageTwoCounter = 0;  // Reset good samples counter
    }
  }

  delay(1000);  // Wait for everything to settle
  /* /Stage Two [Slow Rotate] */

  /* Stage Three [Feedback Correction] */
  // To prevent unstable system from just jittering continously,
  double watchdogStartTime = millis();  // Limit feedback calibration to 60 seconds

  while (isStageThreeTargetReached == false) {
    // Calculate current angle

    currentTime = millis();
    timeElapsed = (currentTime - startTime) / 1000.0000;

    for (int i = 0; i < 5; ++i) {  //Take multiple samples to avoid skew by bad value
      angleToInput = Accel_ArrayTiltAngle(mma2, mma3, mma4);
      currentAngle = Accel_KalmanFilter(angleToInput, timeElapsed);

      // Debug Output
      Serial.print(currentTime);
      Serial.print(",");
      Serial.print(angleToInput);
      Serial.print(",");
      Serial.println(currentAngle);
    }

    // Save for next iteration of loop
    startTime = currentTime;

    // If within target bounds, stop & move to next stage
    if (currentAngle <= stageThreeAngleBounds[0]) {
      if (currentAngle >= stageTwoAngleBounds[1]) {
        ATControl_PinConfigure('t', 's', stageThreeDutyCycle);
        stageThreeCounter += 1;
        if (stageThreeCounter >= 100) {  // If 100 good values read in from kalmanfilter, allow to pass on to next stage
          isStageThreeTargetReached = true;
        }
      }
    }

    // If greater angle than target bound max, tilt forwards
    if (currentAngle > stageThreeAngleBounds[0]) {
      //Serial.println("Tilt forwards");
      ATControl_PinConfigure('t', 'f', stageThreeDutyCycle);
      stageThreeCounter = 0;  // Reset good samples counter
    }

    // If lesser angle than target bound min, tilt backwards
    if (currentAngle < stageThreeAngleBounds[1]) {
      //Serial.println("Tilt back");
      ATControl_PinConfigure('t', 'b', stageThreeDutyCycle);
      stageThreeCounter = 0;  // Reset good samples counter
    }

    if ((currentTime - watchdogStartTime) > 60000) {
      Serial.println("Watchdog timer triggered - halting rotation due to feedback loop failing to lock.");
      isStageThreeTargetReached = true;
      delay(5000);
    }
  }

  delay(1000);  // Wait for everything to settle
  /* /Stage Three [Feedback Correction] */


  // Deprecated
  /*  
  while (!isTargetReached) {
    // Get angle & put through kalman filter
    lastAngle = angleToInput;
    angleToInput = Accel_ArrayTiltAngle(mma2, mma3, mma4);
    currentTime = millis();
    timeElapsed = (currentTime - startTime) / 1000.0000;
    startTime = currentTime;
    angleDiff = abs(lastAngle - angleToInput);

    // Bandpass filter to throw out random bad measurements
    if (angleDiff <= 2) {
      currentAngle = Accel_KalmanFilter(angleToInput, timeElapsed);
    } 


    Serial.print(currentTime);
    Serial.print(",");
    Serial.println(currentAngle);

    // Decide which way to rotate or to stop rotating
    if (doRotate == false) {
      waitCounter += 1;
    }

    if (waitCounter >= 1000) {
      doRotate = true;
      waitCounter = 0;
    }

    if (currentAngle <= angleBounds[0]) {
      if (currentAngle >= angleBounds[1]) {
        ATControl_PinConfigure('t', 's', dutyCycle);
        doRotate = false;

        ensureAngleReached += 1;  // Let Array settle and ensure that is at right angle (and not +/- 0.2 degrees of it
        delay(10);
      }
    }

    if (doRotate == true) {
      if (currentAngle > angleBounds[0]) {
        //Serial.println("Tilt forwards");
        ATControl_PinConfigure('t', 'f', dutyCycle);
        if (ensureAngleReached > 0){
          ensureAngleReached -= 1;  // Reset sequence which ensures angle is within target
        }
      }

      if (currentAngle < angleBounds[1]) {
        //Serial.println("Tilt back");
        ATControl_PinConfigure('t', 'b', dutyCycle);
        if (ensureAngleReached > 0){
          ensureAngleReached -= 1;  // Reset sequence which ensures angle is within target
        }
      }

      if (ensureAngleReached >= 100) {  // Only stop loop once certain array is within desired bounds
        isTargetReached = true;
      }
    }
  }
  */
}
/* /Azimuth Motor Control & Feedback Cycle (Magnetometer-Based) */
