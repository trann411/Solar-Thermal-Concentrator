/* ---------- *
Object file containing settings, variables, and functions used in conjunction with the concentrator's onboard accelerometer(s)
* ---------- */

#include "Arduino.h"
#include "Kalman.h"
#include <Wire.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>

/* MUX Pin Select Function */
#define TCAADDR 0x70

void tcaselect(uint8_t i) {
  if (i > 7) return;

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}
/* /MUX Pin Select Function */

/* Kalman Filter Object */
Kalman filter;
float testRate = 0;
float testDt = 0;
float testInAngle = 0;
float testOutAngle = 0;
/* /Kalman Filter Object */

/* Calculate Tilt Angle of Single Accelerometer (Z-axis WRT Gravity) */
double Accel_TiltAngle(Adafruit_MMA8451 mma, double MUX_Channel) {
  /* Ensure Proper MUX_Channel Selected */
  if ((MUX_Channel < 2) || (MUX_Channel > 4)) {
    Serial.println("Accel_TiltAngle: ERROR -> Improper MUX_Channel Selected; Switching to 2...");
    tcaselect(2);
  } else {
    tcaselect(MUX_Channel);
  }
  /* /Ensure Proper MUX_Channel Selected */

  mma.read();  // Read the 'raw' data in 14-bit counts

  sensors_event_t event;  //Get a new sensor event
  mma.getEvent(&event);

  /* Display the results (acceleration is measured in m/s^2)  */
  double x = event.acceleration.x;
  double y = event.acceleration.y;
  double z = event.acceleration.z;
  double angle = y;

  //Serial.print("X: \t"); Serial.print(x); Serial.print("\t");
  //Serial.print("Y: \t"); Serial.print(y); Serial.print("\t");
  //Serial.print("Z: \t"); Serial.print(z); Serial.print("\t");
  //Serial.println("m/s^2 ");

  //Serial.print("Angle (wrt gravity): ");
  //Serial.print(angle);
  //Serial.println(" degrees");
  /* /Display the results (acceleration is measured in m/s^2)  */

  return (angle);
}
/* /Calculate Tilt Angle of Single Accelerometer (Z-axis WRT Gravity) */

/* Calculate Tilt Angle from Three-Module Array of Accelerometers */
float Accel_ArrayTiltAngle(Adafruit_MMA8451 mma2, Adafruit_MMA8451 mma3, Adafruit_MMA8451 mma4) {
  double g = 9.80665;  // m/s^2; Gravity @ Sea Level

  double Aout = Accel_TiltAngle(mma2, 2);
  double Bout = Accel_TiltAngle(mma3, 3);
  double Cout = Accel_TiltAngle(mma4, 4);

  double thetaAB = 120 * (PI / 180);  // Assume constant angle between three accelerometers; convert to radians
  double thetaBC = thetaAB;

  double absAout = abs(Aout);  // Declare as variables before conditional statements for legibility
  double absBout = abs(Bout);
  double absCout = abs(Cout);
  double halfg = g / 2;
  double neghalfg = -1 * halfg;

  float angle = 0;                    // Value to return
  double mechanicalMountComp = -1.5;  // Calibration factor (degrees) for angle at which array is mounted on concentrator


  //Serial.print("Aout: ");
  //Serial.println(Aout / g);
  //Serial.print("Bout: ");
  //Serial.println(Bout/ g);
  //Serial.print("Cout: ");
  //Serial.println(Cout / g);

  /* Logic for Angle; Based on Formulas from Weng et. al */
  /* For Use on Concentrator, only travels through Cases II - IV*/
  if ((0 < Aout) && (Aout < halfg) && (0 < Bout) && (Cout < 0)) {  //Case I: 0<Aout<0.5g, 0<Bout, Cout<0
    //Serial.println("Tilt Angle Case: I");
    angle = asin(Aout / g);  // θ = arcsin(Aout/g)
  }

  if ((0 < Aout) && (absBout < halfg) && (Cout < 0)) {  // Case II: 0<Aout, |Bout|<0.5g, Cout<0
    //Serial.println("Tilt Angle Case: II");
    angle = (-1 * asin(Bout / g)) + PI - thetaAB;  // θ = -arcsin(Bout/g) + π – θAB
    angle = angle + (6 * (PI / 180));              //Offset
  }

  if ((0 < Aout) && (Bout < 0) && (absCout < halfg)) {  // Case III: 0<Aout, Bout<0, |Cout|<0.5g
    //Serial.println("Tilt Angle Case: III");
    angle = asin(Cout / g) + (2 * PI) + thetaAB - thetaBC;  // θ = arcsin(Cout/g) + 2π – θAB - θBC
    angle = PI - (angle - (4 * PI / 3));                    //Offsete
  }

  if ((absAout < halfg) && (Bout < 0) && (0 < Cout)) {  // Case IV: |Aout|<0.5g, Bout<0, 0<Cout
    //Serial.println("Tilt Angle Case: IV");
    angle = (-1 * asin(Aout / g)) + PI;  // θ= -arcsin(Aout/g)+ π
    angle = -1 * (angle - PI);           //Offset
  }

  if ((Aout < 0) && (absBout < halfg) && (0 < Cout)) {  // Case V: Aout<0, |Bout|<0.5g, 0<Cout
    //Serial.println("Tilt Angle Case: V");
    angle = asin(Bout / g) + (2 * PI) - thetaAB;  // θ= arcsin(Bout/g)+2π-θAB
  }

  if ((Aout < 0) && (0 < Bout) && (absCout < halfg)) {  // Case VI: Aout<0, 0<Bout, |Cout|<0.5g
    //Serial.println("Tilt Angle Case: VI");
    angle = (-1 * asin(Cout / g)) + (3 * PI) - thetaAB - thetaBC;  // θ= -arcsin(Cout/g)+3π-θAB-θBC
  }

  if ((neghalfg < Aout) && (Aout < 0) && (0 < Bout) && (Cout < 0)) {  // Case VII: -0.5g<Aout<0, 0<Bout, Cout<0
    //Serial.println("Tilt Angle Case: VII");
    angle = asin(Aout / g) + (2 * PI);  // θ= sin-1(Aout/g)+2π
  }
  /* /Logic for Angle; Based on Formulas from Weng et. al */

  //angle = 180 - angle;
  angle *= (180 / PI);
  angle += mechanicalMountComp;
  //Serial.print("Angle: ");
  //Serial.println(angle);
  //delay(100);
  return (angle);
}
/* /Calculate Tilt Angle from Three-Module Array of Accelerometers */


/* Post-Processing to Derive More Accurate Reading from Noisy Angle Data */
void Accel_StartKalmanFilter(Adafruit_MMA8451 mma2, Adafruit_MMA8451 mma3, Adafruit_MMA8451 mma4) {
  float startAngle = Accel_ArrayTiltAngle(mma2, mma3, mma4);
  filter.setAngle(startAngle);
}

float Accel_KalmanFilter(float tiltArrayAngle, float timeBetweenSamples) {
  testRate = 0;                                                                  // Gyroscopic Rate, not utilized
  testOutAngle = filter.getAngle(tiltArrayAngle, testRate, timeBetweenSamples);  // Angle from Accel Array
  //Serial.print("Kalman Filter Output Angle:");
  //Serial.println(testOutAngle);
  //Serial.print("Kalman Filter getRate():");
  //Serial.println(testRate);
  return testOutAngle;
}
/* /Post-Processing to Derive More Accurate Reading from Noisy Angle Data */
