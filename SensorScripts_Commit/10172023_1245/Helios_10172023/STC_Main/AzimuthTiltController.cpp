#include "Arduino.h"
#include "Helper.h"
#include "Magno.h"

//
double angle = 0;

void ATControl_PWMPulse(int channel, int dutyCycle) {
  ledcWrite(channel, dutyCycle);

  /*
  for (int dutyCycle = 0; dutyCycle <= 255; dutyCycle++) {
    // increase LED brightness via PWM
    ledcWrite(channel, dutyCycle);
    delay(5);
  }

  for (int dutyCycle = 255; dutyCycle >= 0; dutyCycle--) {
    // decrease the LED brightness via PWM
    ledcWrite(channel, dutyCycle);
    delay(5);
  }
  */
}

void ATControl_PinConfigure(char targetMotor, char direction) {
  targetMotor = toupper(targetMotor);
  direction = toupper(direction);

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
    ATControl_PWMPulse(2, PWMDutyCycle(0)); 
    ATControl_PWMPulse(0, PWMDutyCycle(50));  // PWM Pulse Channel & Percentage Duration

    if (direction == 'S') {
      digitalWrite(A3, LOW);
      digitalWrite(A4, LOW);
      ATControl_PWMPulse(0, PWMDutyCycle(0));
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

    ATControl_PWMPulse(0, PWMDutyCycle(0));
    ATControl_PWMPulse(2, PWMDutyCycle(50));

    if (direction == 'S') {
      digitalWrite(MOSI, LOW);
      ATControl_PWMPulse(2, PWMDutyCycle(0));
    }
  }
}

void ATControl_Rotate(double targetAngle, Adafruit_MMC5603 mag) { // NEED TO ADDRESS EDGE CASES AROUND 0 DEG TARGET ANGLE

  /* Get Intial Starting Position (Relative to Compass) */
  double currentAngle = 0;
  for (int i = 0; i < 10; i++){ // Take average of 10 values
    currentAngle += Magno_CompassSample(mag);
    delay(10);
  }
  currentAngle /= 10;

  Serial.print("currentAngle:");
  Serial.println(currentAngle);
  /* /Get Intial Starting Position (Relative to Compass) */

  /* Calcuate Midpoint for Determing Which Way to Rotate */
  double midpoint = currentAngle + 180;
  if (midpoint >= 360){ // Correct if over 360 degrees
    midpoint -= 360;
  }

  Serial.print("midpoint:");
  Serial.println(midpoint);
  /* /Calcuate Midpoint for Determing Which Way to Rotate */

  /* Set Motor Direction (CW || CCW) */
  ATControl_PinConfigure('a', 's'); // Ensure motor starts in off state

  Serial.println(targetAngle);
  while ( (currentAngle > 95) || (currentAngle < 85) ){
    if (currentAngle >= midpoint){
      ATControl_PinConfigure('a', 'f');
    }

    if (currentAngle < midpoint){
      if(currentAngle < targetAngle){
        ATControl_PinConfigure('a', 'f');
      }
      if(currentAngle > targetAngle){
        ATControl_PinConfigure('a', 'b');
      }
    }
    currentAngle = Magno_CompassSample(mag);
    delay(10);
  }
  ATControl_PinConfigure('a', 's'); // Ensure motor starts in off state
  
  Serial.println("reached target angle");
  delay(1000);

  /* /Set Motor Direction (CW || CCW) */
}