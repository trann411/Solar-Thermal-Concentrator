/*
 * MIT License
 * Copyright (c) 2021 Anton Sundqvist
 */
#include "Magnetometer_Autocal.h"
#include <Adafruit_MMC56x3.h>

#define DECPOINTS 5
MAGAC model= MAGAC();

Adafruit_MMC5603 mag = Adafruit_MMC5603(12345);

float corrData[3]; //Variable to hold corrected data

void setup() {
  Serial.begin(115200);
  model.begin();
  
  //If you have a pre-trained model, you can initialize the parameters after calling model.begin().
  //model.setOffset()
  //model.setBias()
}

void loop() {
  //#CodeNeeded
  sensors_event_t magEvent;

  mag.getEvent(&magEvent);

  //Read most recent values from hardware into a three-element array.
  float rawData[]={magEvent.magnetic.x, magEvent.magnetic.y, magEvent.magnetic.z};  //This is just some dummy data

  model.update(rawData);
  model.read(corrData);
  
  Serial.print(rawData[0],DECPOINTS);Serial.print("\t");
  Serial.print(rawData[1],DECPOINTS);Serial.print("\t");
  Serial.print(rawData[2],DECPOINTS);Serial.print("\t");
  Serial.print(corrData[0],DECPOINTS);Serial.print("\t");
  Serial.print(corrData[1],DECPOINTS);Serial.print("\t");
  Serial.print(corrData[2],DECPOINTS);Serial.println("");
  
  delay(10);
}
