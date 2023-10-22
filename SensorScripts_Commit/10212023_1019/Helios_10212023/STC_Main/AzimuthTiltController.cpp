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
    ATControl_PWMPulse(0, PWMDutyCycle(40));  // PWM Pulse Channel & Percentage Duration

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
    ATControl_PWMPulse(2, PWMDutyCycle(40));

    if (direction == 'S') {
      digitalWrite(MOSI, LOW);
      ATControl_PWMPulse(2, PWMDutyCycle(0));
    }
  }
}

void ATControl_Rotate(double targetAngle, Adafruit_MMC5603 mag) { // NEED TO ADDRESS EDGE CASES AROUND 0 DEG TARGET ANGLE

  double currentAngle = Magno_CompassSample(mag);
  double angleBounds[] = { targetAngle + 5, targetAngle - 5 };

  if(angleBounds[0] >= 360){
    angleBounds[0] -= 360;
  }

  if (angleBounds[1] < 0) {
    angleBounds[1] += 360;
  }

  while (currentAngle != targetAngle) {
    //Serial.print("Max Angle:");
    //Serial.println(angleBounds[0]);
    //Serial.print("Min Angle:");
   // Serial.println(angleBounds[1]);
    
    currentAngle = Magno_CompassSample(mag);

    if (currentAngle <= angleBounds[0]) {
      if (currentAngle >= angleBounds[1]) {
        ATControl_PinConfigure('a', 's');
        currentAngle = targetAngle;
        break;
      }
    }

    if (currentAngle > angleBounds[0]) {
      ATControl_PinConfigure('a', 'f');
    }
    if (currentAngle < angleBounds[1]) {
      ATControl_PinConfigure('a', 'b');
    }
    
    delay(10);
  }
}