#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>

/* Calculate Tilt Angle of Single Accelerometer (Z-axis WRT Gravity) */
double Accel_TiltAngle(Adafruit_MMA8451 mma) {
  mma.read(); // Read the 'raw' data in 14-bit counts

  sensors_event_t event; //Get a new sensor event 
  mma.getEvent(&event);

  /* Display the results (acceleration is measured in m/s^2)  */
  double x = event.acceleration.x;
  double y = event.acceleration.y;
  double z = event.acceleration.z;
  double angle = atan( (sqrt( (x*x) + (y*y))) / z ) * (180 / PI);

  Serial.print("X: \t"); Serial.print(x); Serial.print("\t");
  Serial.print("Y: \t"); Serial.print(y); Serial.print("\t");
  Serial.print("Z: \t"); Serial.print(z); Serial.print("\t");
  Serial.println("m/s^2 ");
  
  Serial.print("Angle (wrt gravity): ");
  Serial.print(angle);
  Serial.println(" degrees");
  /* /Display the results (acceleration is measured in m/s^2)  */
  
  return(angle);
}
/* /Calculate Tilt Angle of Single Accelerometer (Z-axis WRT Gravity) */


/* Calculate Tilt Angle from Three-Module Array of Accelerometers */
double Accel_ArrayTiltAngle(Adafruit_MMA8451 mma) {
  double Aout = 0; // Sample Accel A (acceleration of z)
  double Bout = 0; // Sample Accel B (acceleration of z)
  double Cout = 0; // Sample Accel C (acceleration of z)

  double thetaAB = 120 * (PI/180); // Assume constant angle between three accelerometers; convert to radians
  double thetaBC = thetaAB;

  double g = 9.80665; // m/s^2; Gravity @ Sea Level

  double absAout = abs(Aout); // Declare as variables before conditional statements for legibility
  double absBout = abs(Bout);
  double absCout = abs(Cout);
  double halfg = g / 2;
  double neghalfg = -1 * halfg;

  double angle = 0; // Value to return

  /* Logic for Angle; Based on Formulas from Weng et. al */
  if ( (0 < Aout) && (Aout < halfg) && (0 < Bout) && (Cout < 0) ){ //Case I: 0<Aout<0.5g, 0<Bout, Cout<0
    angle = asin(Aout / g); // θ = arcsin(Aout/g)
  }

  if ( (0 < Aout) && (absBout < halfg) && (Cout < 0) ){ // Case II: 0<Aout, |Bout|<0.5g, Cout<0
    angle = (-1 * asin(Bout / g)) + PI - thetaAB; // θ = -arcsin(Bout/g) + π – θAB
  }

  if ( (0 < Aout) && (Bout < 0) && (absCout < 0) ){ // Case III: 0<Aout, Bout<0, |Cout|<0.5g
    angle = asin(Cout / g) + (2*PI) + thetaAB - thetaBC; // θ = arcsin(Cout/g) + 2π – θAB - θBC
  }

  if ( (absAout < halfg) && (Bout < 0) && (0 < Cout) ) { // Case IV: |Aout|<0.5g, Bout<0, 0<Cout
    angle = (-1 * asin(Aout / g)) + PI; // θ= -arcsin(Aout/g)+ π
  }
  
  if ( (Aout < 0) && (absBout < halfg) && (0 < Cout) ){ // Case V: Aout<0, |Bout|<0.5g, 0<Cout 
    angle = asin(Bout / g) + (2*PI) - thetaAB; // θ= arcsin(Bout/g)+2π-θAB
  }

  if( (Aout < 0) && (0 < Bout) && (absCout < halfg) ){ // Case VI: Aout<0, 0<Bout, |Cout|<0.5g
    angle = (-1 * asin(Cout / g)) + (3*PI) - thetaAB - thetaBC; // θ= -arcsin(Cout/g)+3π-θAB-θBC
  }

  if( (neghalfg < Aout) && (Aout < 0) && (0 < Bout) && (Cout < 0) ){ // Case VII: -0.5g<Aout<0, 0<Bout, Cout<0 
    angle = asin(Aout / g) + (2*PI); // θ= sin-1(Aout/g)+2π
  }
  /* /Logic for Angle; Based on Formulas from Weng et. al */

  angle *= 180/PI; // Convert Back to Degrees

  return(angle);
}
/* /Calculate Tilt Angle from Three-Module Array of Accelerometers */


/* Post-Processing to Derive More Accurate Reading from Noisy Angle Data */
double Accel_KalmannFilter(double tiltArrayAngle){
  // :)
}
/* /Post-Processing to Derive More Accurate Reading from Noisy Angle Data */
