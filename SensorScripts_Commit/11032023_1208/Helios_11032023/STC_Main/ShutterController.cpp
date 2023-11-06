#include "Arduino.h"
#include "Helper.h"

/* Target Actuator & Direction Control Function */
void SHControl_PinConfigure(char targetMotor, char direction, double dutyCycle) {
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
    ledcWrite(6, PWMDutyCycle(0));
    ledcWrite(4, PWMDutyCycle(dutyCycle));  // PWM Pulse Channel & Percentage Duration

    if (direction == 'S') {
      digitalWrite(9, LOW);
      digitalWrite(10, LOW);
      ledcWrite(4, PWMDutyCycle(0));
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

    ledcWrite(4, PWMDutyCycle(0));
    ledcWrite(6, PWMDutyCycle(dutyCycle));

    if (direction == 'S') {
      digitalWrite(12, LOW);
      digitalWrite(13, LOW);
      ledcWrite(6, PWMDutyCycle(0));
    }
  }
}
/* /Target Actuator & Direction Control Function */
