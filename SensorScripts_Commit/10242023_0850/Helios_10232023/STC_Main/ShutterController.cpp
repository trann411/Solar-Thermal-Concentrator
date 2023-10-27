#include "Arduino.h"
#include "Helper.h"

//

void SHControl_PWMPulse(int channel, int dutyCycle) {
  ledcWrite(channel, dutyCycle);

  /* // Since naked eye can't see PWM pulses, cycle brightness to indicate functionality.
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

void SHControl_PinConfigure(char targetMotor, char direction) {
  targetMotor = toupper(targetMotor);
  direction = toupper(direction);

  if (targetMotor == 'L') {
    // Set EN3 high, EN4 low; break before make to prevent magic smoke
    digitalWrite(12, LOW);
    digitalWrite(13, LOW);

    if (direction == 'F') {
      // SET IN1/IN3 high, IN2/IN4 low; break before make to prevent magic smoke
      digitalWrite(10, LOW);
      digitalWrite(9, HIGH);
    }

    if (direction == 'B') {
      // SET IN1/IN3 high, IN2/IN4 low; break before make to prevent magic smoke
      digitalWrite(9, LOW);
      digitalWrite(10, HIGH);
    }
    SHControl_PWMPulse(6, PWMDutyCycle(0)); 
    SHControl_PWMPulse(4, PWMDutyCycle(40));  // PWM Pulse Channel & Percentage Duration

    if (direction == 'S') {
      digitalWrite(9, LOW);
      digitalWrite(10, LOW);
      SHControl_PWMPulse(4, PWMDutyCycle(0));
    }
  }

  if (targetMotor == 'R') {
    // Set ENA2 high, ENA1 low; break before make to prevent magic smoke
    digitalWrite(9, LOW);
    digitalWrite(10, LOW);

    if (direction == 'F') {
      // SET IN1/IN3 high, IN2/IN4 low; break before make to prevent magic smoke
      digitalWrite(13, LOW);
      digitalWrite(12, HIGH);
    }

    if (direction == 'B') {
      // SET IN1/IN3 high, IN2/IN4 low; break before make to prevent magic smoke
      digitalWrite(12, LOW);
      digitalWrite(13, HIGH);
    }

    SHControl_PWMPulse(4, PWMDutyCycle(0));
    SHControl_PWMPulse(6, PWMDutyCycle(40));

    if (direction == 'S') {
      digitalWrite(12, LOW);
      digitalWrite(13, LOW);
      SHControl_PWMPulse(6, PWMDutyCycle(0));
    }
  }
}