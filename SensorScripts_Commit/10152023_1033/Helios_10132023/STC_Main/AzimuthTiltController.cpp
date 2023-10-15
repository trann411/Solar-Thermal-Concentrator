#include "Arduino.h"

//
double angle = 0;

void ATControl_PWMPulse(int _pin, int dutyCycle){
  //ledcSetup(_pin, 5000, 8);
}

void ATControl_PinConfigure(char targetMotor, char direction){
  targetMotor = toupper(targetMotor);
  direction = toupper(direction);

  if(targetMotor == 'A'){
    // Set ENA1 high, ENA2 low; break before make to prevent magic smoke
    //digitalWrite(A5, LOW);
    digitalWrite(SCK, LOW);
    digitalWrite(MOSI, LOW);
    //digitalWrite(A2, HIGH)

    if(direction == 'F'){
      // SET IN1/IN3 high, IN2/IN4 low; break before make to prevent magic smoke
      digitalWrite(A4, LOW);
      digitalWrite(A3, HIGH);
    }

    if(direction == 'B'){
      // SET IN1/IN3 high, IN2/IN4 low; break before make to prevent magic smoke
      digitalWrite(A3, LOW);
      digitalWrite(A4, HIGH);
    }

    for(int dutyCycle = 0; dutyCycle <= 255; dutyCycle++){   
    // changing the LED brightness with PWM
    ledcWrite(0, dutyCycle);
    delay(5);
    }

    for(int dutyCycle = 255; dutyCycle >= 0; dutyCycle--){
    // changing the LED brightness with PWM
    ledcWrite(0, dutyCycle);   
    delay(5);
    }

  }

  if(targetMotor == 'T'){
    // Set ENA2 high, ENA1 low; break before make to prevent magic smoke
      //digitalWrite(A2, LOW);
      //digitalWrite(A5, HIGH);
      digitalWrite(A3, LOW);
      digitalWrite(A4, LOW);

    if(direction == 'F'){
    // SET IN1/IN3 high, IN2/IN4 low; break before make to prevent magic smoke
      digitalWrite(MOSI, LOW);
      digitalWrite(SCK, HIGH);
    }

    if(direction == 'B'){
      // SET IN1/IN3 high, IN2/IN4 low; break before make to prevent magic smoke
      digitalWrite(SCK, LOW);
      digitalWrite(MOSI, HIGH);
    }

    /*
    for(int dutyCycle = 0; dutyCycle <= 255; dutyCycle++){   
    // changing the LED brightness with PWM
    ledcWrite(2, dutyCycle);
    delay(5);
    }

    for(int dutyCycle = 255; dutyCycle >= 0; dutyCycle--){
    // changing the LED brightness with PWM
    ledcWrite(2, dutyCycle);   
    delay(5);
    }
    */
  }
}

void ATControl_Rotate(char targetMotor, char direction, double targetAngle){
  targetMotor = toupper(targetMotor);
  direction = toupper(targetMotor);

  double tolerance = (5 / 100);
  double angleBounds[] = {targetAngle * (1 + tolerance), targetAngle * (1 - tolerance)};

  ATControl_PinConfigure(targetMotor, direction);

  //angle = read in angle
  // while angle != target angle, rotate; if angle == targetAngle (Within bounds), stop rotate

}