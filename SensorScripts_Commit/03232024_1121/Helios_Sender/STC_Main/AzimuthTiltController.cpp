/* ---------- *
Object file containing settings, variables, and functions used in conjunction with the concentrator's onboard tilt, azimuth motor controller(s)
* ---------- */

#include "Arduino.h"
#include "Helper.h"
#include "Magno.h"
#include "Accel.h"
#include "RotaryEncode.h"
#include "Pyro.h"

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

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


// NOTE: Many functions pertaining to azimuth rotate have magno in the name but do not utilize the magnetometer.
//       This is due to the magnetometer not giving accurate enough results and it being swapped out for a
//       pyranometer-based sweep (in order to determine intial position).

/* Azimuth Motor Control & Feedback Cycle (RotaryEncoder-Based) */
double ATControl_MagnoRotate(double targetAngle, double presentAngle) {  // NEED TO ADDRESS EDGE CASES AROUND 0 DEG TARGET ANGLE
  if((targetAngle > 180) && (targetAngle <= 360)){
    targetAngle = targetAngle - 360;
  }
  
  double dutyCycle = 20;
  int numberOfSamples = 10;  // Numbers of samples for rotary encoder to take; ensure that (90 / PPR) * numberSamples <= 0.1 to keep within accuracy targets.

  double angleDelta = 0;
  double angleBounds[] = { targetAngle + 0.1, targetAngle - 0.1 };
  bool isTargetReached = false;
  float magnoFeedbackTime = millis();
  
  /*
  if (angleBounds[0] >= 360) {
    angleBounds[0] -= 360;
  }

  if (angleBounds[1] < 0) {
    angleBounds[1] += 360;
  }
  */

  while (isTargetReached == false) {
    // Serial.print("Max Angle:");
    // Serial.println(angleBounds[0]);
    // Serial.print("Min Angle:");
    // Serial.println(angleBounds[1]);

    if (presentAngle <= angleBounds[0]) {
      if (presentAngle >= angleBounds[1]) {
        ATControl_PinConfigure('a', 's', dutyCycle);
        isTargetReached = true;
      }
    }

    if (presentAngle > angleBounds[0]) {
      ATControl_PinConfigure('a', 'f', dutyCycle);
      angleDelta = RotaryEncode_Tracking(numberOfSamples);
      ATControl_PinConfigure('a', 's', dutyCycle);
    }

    if (presentAngle < angleBounds[1]) {
      ATControl_PinConfigure('a', 'b', dutyCycle);
      angleDelta = RotaryEncode_Tracking(numberOfSamples);
      ATControl_PinConfigure('a', 's', dutyCycle);
    }

    presentAngle += angleDelta;

    magnoFeedbackTime = millis();

    /* Serial Debug Output */
    Serial.print(magnoFeedbackTime);
    Serial.print(",");
    Serial.print(angleDelta);
    Serial.print(",");
    Serial.print(targetAngle);
    Serial.print(",");
    Serial.println(presentAngle);
    //delay(2000);
    /* /Serial Debug Output */
  }
  return (presentAngle);
}
/* /Azimuth Motor Control & Feedback Cycle (Magnetometer-Based) */

/* Tilt Motor Control & Feedback Cycle (Magnetometer-Based) */
void ATControl_AccelRotate(double targetAngle, Adafruit_MMA8451 mma2, Adafruit_MMA8451 mma3, Adafruit_MMA8451 mma4, int desiredStage) {  // NEED TO ADDRESS EDGE CASES AROUND 0 DEG TARGET ANGLE

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

  // Flags for when to move on to next stage
  bool isStageOneTargetReached = false;
  bool isStageTwoTargetReached = false;
  bool isStageThreeTargetReached = false;

  // Duty cycles per stage for tilt actuator
  double stageOneDutyCycle = 95;
  double stageTwoDutyCycle = 55;
  double stageThreeDutyCycle = 20;

  int stageOneCounter = 0;  // Track number of "good" values read in from kalman filter
  int stageTwoCounter = 0;  // before permitting control loop to end.
  int stageThreeCounter = 0;

  float lastAngleToInput = 0;
  float angleDiff = 0;

  // Time tracking for Kalman filter
  float currentTime;
  float timeElapsed;

  float stageOneWatchdogStart;
  float stageTwoWatchdogStart;
  float stageThreeWatchdogStart;

  float startTime = millis();

  /* /Stage Zero [Check if Valid Target] */

  if (desiredStage == 1) {
    /* Stage One [Fast Rotate] */
    stageOneWatchdogStart = millis();

    while (isStageOneTargetReached == false) {
      // Record time for kalman filter
      currentTime = millis();

      timeElapsed = (currentTime - startTime) / 1000.0000;


      //for (int i = 0; i < 5; ++i) {  //Take multiple samples to avoid skew by bad value
      angleToInput = Accel_ArrayTiltAngle(mma2, mma3, mma4);
      currentAngle = Accel_KalmanFilter(angleToInput, timeElapsed);
      //}

      // Debug Output
      Serial.print(currentTime);
      Serial.print(",");
      Serial.print("1");
      Serial.print(",");
      Serial.print(targetAngle);
      Serial.print(",");
      Serial.print(angleToInput);
      Serial.print(",");
      Serial.println(currentAngle);

      // Save for next iteration of loop
      startTime = currentTime;

      // If within target bounds, stop & move to next stage
      if (currentAngle <= stageOneAngleBounds[0]) {
        if (currentAngle >= stageOneAngleBounds[1]) {
          ATControl_PinConfigure('t', 's', stageOneDutyCycle);
          stageOneCounter += 1;
          if (stageOneCounter >= 10) {  // If 100 good values read in from kalmanfilter, allow to pass on to next stage
            isStageOneTargetReached = true;
          }
        }
      }

      // If greater angle than target bound max, tilt forwards
      if (currentAngle > stageOneAngleBounds[0]) {
        //Serial.println("Tilt forwards");
        ATControl_PinConfigure('t', 'f', stageOneDutyCycle);
        delay(5000);  // 250 is sweet spot
        ATControl_PinConfigure('t', 's', stageThreeDutyCycle);
        delay(500);
        stageOneCounter = 0;  // Reset good samples counter
      }

      // If lesser angle than target bound min, tilt backwards
      if (currentAngle < stageOneAngleBounds[1]) {
        //Serial.println("Tilt back");
        ATControl_PinConfigure('t', 'b', stageOneDutyCycle);
        delay(5000);  // 250 is sweet spot
        ATControl_PinConfigure('t', 's', stageThreeDutyCycle);
        delay(500);
        stageOneCounter = 0;  // Reset good samples counter
      }

      /*
      if ((currentTime - stageOneWatchdogStart) > 20000) {
        ATControl_PinConfigure('t', 's', stageOneDutyCycle);
        isStageOneTargetReached = true;
        Serial.println("WARNING: Stage One Watchdog Timer Triggered; Moving On to Prevent Bluetooth Dropout.");
      }
      */
    }
  }
  /* /Stage One [Fast Rotate] */

  /* Stage Slow [Slow Rotate] */
  if (desiredStage == 2) {

    stageTwoWatchdogStart = millis();

    while (isStageTwoTargetReached == false) {
      // Calculate current angle

      //currentTime = millis();

      //timeElapsed = (currentTime - startTime) / 1000.0000;

      //for (int i = 0; i < 5; ++i) {  //Take multiple samples to avoid skew by bad value
      //angleToInput = Accel_ArrayTiltAngle(mma2, mma3, mma4);
      //currentAngle = Accel_KalmanFilter(angleToInput, timeElapsed);

      for (int i = 0; i < 5; ++i) {  //Take multiple samples to avoid skew by bad value
        currentTime = millis();
        timeElapsed = (currentTime - startTime) / 1000.0000;

        // Save for next iteration of loop
        startTime = currentTime;

        angleToInput = Accel_ArrayTiltAngle(mma2, mma3, mma4);
        angleDiff = abs(angleToInput - lastAngleToInput);
        //if (angleToInput == 0) {
        // Don't throw in odd zero values
        // } else if (angleDiff > 10) {
        // Don't throw in weird outlier jumps
        //} else {
        currentAngle = Accel_KalmanFilter(angleToInput, timeElapsed);
        //}
        lastAngleToInput = angleToInput;
      }
      //}
      // Debug Output
      Serial.print(currentTime);
      Serial.print(",");
      Serial.print("2");
      Serial.print(",");
      Serial.print(targetAngle);
      Serial.print(",");
      Serial.print(angleToInput);
      Serial.print(",");
      Serial.println(currentAngle);

      // Save for next iteration of loop
      startTime = currentTime;

      // If within target bounds, stop & move to next stage
      if (currentAngle <= stageTwoAngleBounds[0]) {
        if (currentAngle >= stageTwoAngleBounds[1]) {
          ATControl_PinConfigure('t', 's', stageTwoDutyCycle);
          stageTwoCounter += 1;
          if (stageTwoCounter >= 30) {  // If 100 good values read in from kalmanfilter, allow to pass on to next stage
            isStageTwoTargetReached = true;
            lastAngleToInput = currentAngle;
          }
        }
      }

      // If greater angle than target bound max, tilt forwards
      if (currentAngle > stageTwoAngleBounds[0]) {
        //Serial.println("Tilt forwards");
        ATControl_PinConfigure('t', 'f', stageTwoDutyCycle);
        delay(750);
        ATControl_PinConfigure('t', 's', stageTwoDutyCycle);
        delay(500);
        stageTwoCounter = 0;  // Reset good samples counter
      }

      // If lesser angle than target bound min, tilt backwards
      if (currentAngle < stageTwoAngleBounds[1]) {
        //Serial.println("Tilt back");
        ATControl_PinConfigure('t', 'b', stageTwoDutyCycle);
        delay(750);
        ATControl_PinConfigure('t', 's', stageTwoDutyCycle);
        delay(500);
        stageTwoCounter = 0;  // Reset good samples counter
      }

      /*
      if ((currentTime - stageTwoWatchdogStart) > 20000) {
        ATControl_PinConfigure('t', 's', stageTwoDutyCycle);
        isStageTwoTargetReached = true;
        Serial.println("WARNING: Stage Two Watchdog Timer Triggered; Moving On to Prevent Bluetooth Dropout.");
      }
      */
    }
  }
  /* /Stage Two [Slow Rotate] */

  /* Stage Three [Feedback Correction] */
  // To prevent unstable system from just jittering continously,
  if (desiredStage == 3) {

    stageThreeWatchdogStart = millis();

    while (isStageThreeTargetReached == false) {
      // Calculate current angle
      currentTime = millis();

      timeElapsed = (currentTime - startTime) / 1000.0000;

      // Save for next iteration of loop
      startTime = currentTime;

      for (int i = 0; i < 50; ++i) {  //Take multiple samples to avoid skew by bad value
        currentTime = millis();
        timeElapsed = (currentTime - startTime) / 1000.0000;

        // Save for next iteration of loop
        startTime = currentTime;

        angleToInput = Accel_ArrayTiltAngle(mma2, mma3, mma4);
        angleDiff = abs(angleToInput - lastAngleToInput);
        // if (angleToInput == 0) {
        // Don't throw in odd zero values
        // } else if (angleDiff > 1) {
        // Don't throw in weird outlier jumps
        //} else {
        currentAngle = Accel_KalmanFilter(angleToInput, timeElapsed);
        //}
        lastAngleToInput = angleToInput;

        Serial.print(currentTime);
        Serial.print(",");
        Serial.print(timeElapsed);
        Serial.print(",");
        Serial.print("3");
        Serial.print(",");
        Serial.print(targetAngle);
        Serial.print(",");
        Serial.print(angleToInput);
        Serial.print(",");
        Serial.print(lastAngleToInput);
        Serial.print(",");
        Serial.println(currentAngle);
      }

      /* Debug Output
      Serial.print(currentTime);
      Serial.print(",");
      Serial.print(timeElapsed);
      Serial.print(",");
      Serial.print("3");
      Serial.print(",");
      Serial.print(targetAngle);
      Serial.print(",");
      Serial.print(angleToInput);
      Serial.print(",");
      Serial.print(lastAngleToInput);
      Serial.print(",");
      Serial.println(currentAngle);
      */
      // If within target bounds, stop & move to next stage
      if (currentAngle <= stageThreeAngleBounds[0]) {
        if (currentAngle >= stageTwoAngleBounds[1]) {
          ATControl_PinConfigure('t', 's', stageThreeDutyCycle);
          stageThreeCounter += 1;
          if (stageThreeCounter >= 5) {  // If 100 good values read in from kalmanfilter, allow to pass on to next stage
            isStageThreeTargetReached = true;
          }
        }
      }

      // If greater angle than target bound max, tilt forwards
      if (currentAngle > stageThreeAngleBounds[0]) {
        //Serial.println("Tilt forwards");
        ATControl_PinConfigure('t', 'f', stageThreeDutyCycle);
        delay(300);  // 250 is sweet spot
        ATControl_PinConfigure('t', 's', stageThreeDutyCycle);
        stageThreeCounter = 0;  // Reset good samples counter
        // Delay & wait to stabilize
        delay(500);
      }

      // If lesser angle than target bound min, tilt backwards
      if (currentAngle < stageThreeAngleBounds[1]) {
        //Serial.println("Tilt back");
        ATControl_PinConfigure('t', 'b', stageThreeDutyCycle);
        delay(300);
        ATControl_PinConfigure('t', 's', stageThreeDutyCycle);
        stageThreeCounter = 0;  // Reset good samples counter
        // Delay & wait to stabilize
        delay(500);
      }

      /*
      if ((currentTime - stageThreeWatchdogStart) > 20000) {
        ATControl_PinConfigure('t', 's', stageThreeDutyCycle);
        isStageThreeTargetReached = true;
        Serial.println("WARNING: Stage Three Watchdog Timer Triggered; Moving On to Prevent Bluetooth Dropout.");
      }
      */
    }
  }

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
  delay(3000);
}
/* /Tilt Motor Control & Feedback Cycle (Magnetometer-Based) */

/* Sweep Azimuth for Starting Angle based on Pyranometer Input & Astronomical Algorithm Value*/
double ATControl_PyroSweep(int _pyroPin, double presentAngle) {
  // Take in pin for pyronometer and global azimuth position variable.

  // 0. Set initial angle to 0 degrees & iterate until hit 180 degree
  //presentAngle = ATControl_MagnoRotate(0, presentAngle);

  // 1. Take in Pyronometer Value
  int numberOfPyroSamples = 30;
  double pyroValue = Pyro_ReadTemp(_pyroPin);
  double angleToIncrementBy = 1;
  double angleToTravelTo = 0;
  int numberOfLoopIterations = round(10 / angleToIncrementBy);

  double pyroSamples[numberOfLoopIterations];
  double azimuthAngles[numberOfLoopIterations];
  double maxPyroValue = 0;     // Maximum pyranometer value recorded...
  double matchingAzimuth = 0;  ///...and the corresponding angle at which it was recorded


  for (int i = 0; i < numberOfLoopIterations; ++i) {
    for (int j = 0; j < numberOfPyroSamples; ++j) {
      //Serial.println(pyroValue);
      pyroValue += Pyro_ReadTemp(_pyroPin);
      pyroValue = pyroValue / 2;
    }
    // 2. Take in current azimuth angle
    // 3. Rotate by desired arc
    //pyroSamples[i] = pyroValue;
    //azimuthAngles[i] = presentAngle;

    //Serial.print(pyroSamples[i]);
    //Serial.println(azimuthAngles[i]);
    angleToTravelTo = presentAngle - angleToIncrementBy;

    pyroSamples[i] = pyroValue;
    azimuthAngles[i] = presentAngle;

    Serial.print(pyroValue);
    Serial.print(",");
    Serial.print(presentAngle);
    Serial.print(",");
    Serial.println(angleToTravelTo);
    delay(10);

    presentAngle = ATControl_MagnoRotate(angleToTravelTo, presentAngle);
    //Serial.println("Reached Target");
    delay(10);
    // 4. Repeat of x data points across 180 degree range
  }

  // Find Maximum

  for (int k = 0; k < numberOfLoopIterations; ++k) {
    Serial.println("Sweeping through recorded values");
    if (pyroSamples[k] > maxPyroValue) {
      maxPyroValue = pyroSamples[k];
      matchingAzimuth = azimuthAngles[k];
    }

    Serial.print("Pyro: ");
    Serial.print(pyroSamples[k]);
    Serial.print(", Azimuth:");
    Serial.println(azimuthAngles[k]);
  }

  Serial.print("Max Pyro: ");
  Serial.print(maxPyroValue);
  Serial.print(", Matching Azimuth:");
  Serial.println(matchingAzimuth);

  // Travel to Maximum
  presentAngle = ATControl_MagnoRotate(matchingAzimuth, presentAngle);
  Serial.print("Reached angle of: ");
  Serial.println(presentAngle);

  return (presentAngle);
}
/* /Sweep Azimuth for Starting Angle based on Pyranometer Input & Astronomical Algorithm Value*/
