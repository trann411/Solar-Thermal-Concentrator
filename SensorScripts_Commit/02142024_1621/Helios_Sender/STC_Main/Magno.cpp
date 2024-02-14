#include "Arduino.h"
#include "Helper.h"
#include <Adafruit_MMC56x3.h>

/* Calculate Compass Angle From & WRT Magnetometer */
double Magno_CompassSample(Adafruit_MMC5603 mag) {

  /* Calculate & Apply Hard Iron offsets */
  float MagMinX, MagMaxX;
  float MagMinY, MagMaxY;
  float MagMinZ, MagMaxZ;

  MagMinX = MagMaxX = MagMinY = MagMaxY = MagMinZ = MagMaxZ = 0;

  sensors_event_t event;

  for (int i = 0; i < 100; ++i) {
    mag.getEvent(&event);
    if (event.magnetic.x < MagMinX) MagMinX = event.magnetic.x;
    if (event.magnetic.x > MagMaxX) MagMaxX = event.magnetic.x;

    if (event.magnetic.y < MagMinY) MagMinY = event.magnetic.y;
    if (event.magnetic.y > MagMaxY) MagMaxY = event.magnetic.y;

    if (event.magnetic.z < MagMinZ) MagMinZ = event.magnetic.z;
    if (event.magnetic.z > MagMaxZ) MagMaxZ = event.magnetic.z;
  }

  double hardIronX = (MagMaxX + MagMinX) / 2;
  double hardIronY = (MagMaxY + MagMinY) / 2;

  Serial.print("Hard Iron Offsets: X = ");
  Serial.print(hardIronX);
  Serial.print(", Y = ");
  Serial.println(hardIronY);

  /* /Calculate Hard Iron offsets */

  delay(100);

  /* Read Current Position */
  mag.getEvent(&event);

  double Mag_X = event.magnetic.x;
  double Mag_Y = event.magnetic.y;
  Serial.print("Mag_X: ");
  Serial.print(Mag_X);
  Serial.print(", Mag_Y: ");
  Serial.println(Mag_Y);

  Mag_X -= hardIronX;
  Mag_Y -= hardIronY;
  Serial.print("Mag_X_Offset: ");
  Serial.print(Mag_X);
  Serial.print(", Mag_Y_Offset: ");
  Serial.println(Mag_Y);

  /* /Read Current Position */

  float heading = (atan2(Mag_Y, Mag_X)) * (180 / PI) - 90;  // Calculate Heading

  // Normalize to 0-360
  if (heading < 0) {
    heading = 360 + heading;
  }

  Serial.print("Compass Heading: ");
  //Serial.println(Mag_X);
  //Serial.println(Mag_Y);
  //Serial.print("Magno: ");
  Serial.println(heading);

  return (heading);
}
/* /Calculate Compass Angle From & WRT Magnetometer */
