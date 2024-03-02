/* Include Internal Scripts */
#include "Accel.h"
#include "AzimuthTiltController.h"
#include "Debug.h"
#include "Helper.h"
#include "Magno.h"
#include "Pyro.h"
#include "RotaryEncode.h"
#include "ShutterController.h"
#include "Thermo.h"
#include "Kalman.h"
/* /Include Internal Scripts */

/* Include External Libraries */
#include <stdlib.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <string.h>
/* /Include External Libraries */

/* Include Bluetooth Libraries */
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
/* /Include Bluetooth Libraries */


/* Sensor & Bluetooth Object Declaration */

/* Magnetometer Variables */
Adafruit_MMC5603 mag = Adafruit_MMC5603(12345);
/* /Magnetometer Variables */

/* Accelerometer Variables */
Adafruit_MMA8451 mma2 = Adafruit_MMA8451(2);
Adafruit_MMA8451 mma3 = Adafruit_MMA8451(3);
Adafruit_MMA8451 mma4 = Adafruit_MMA8451(4);
/* /Accelerometer Variables */


/* Bluetooth Variables & Command Interface */
// https://randomnerdtutorials.com/esp32-ble-server-client/
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

#define returnData
float temp;
float dataToReturn;
bool deviceConnected = false;
bool oldDeviceConnected = false;
String input = "null";
BLEServer *pServer = NULL;
char *pch = NULL;
String pch0;  //Command
String pch1;  // Percentage

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    deviceConnected = true;
  };

  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
  }
};

class MyCallbacks : public BLECharacteristicCallbacks {  // Interface Read/Write Function to Return Data
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string inputCache = pCharacteristic->getValue();  // String conversion that I don't understand, but it no longer errors out,
    String input = inputCache.c_str();

    deviceConnected = true;
    String delimiter = " ";
    int delimiterIndex = input.indexOf(delimiter);

    /* Take Input, split into command and percentage (if applicable) */
    if (delimiterIndex != -1) {
      pch0 = input.substring(0, delimiterIndex);
      pch1 = input.substring(delimiterIndex + 1, input.length());
    } else {
      pch0 = input;
      pch1 = "na";
    }

    if (pch1 == "na") {
      if (pch0 == "thermo") {
        double temp = Thermo_ReadTemp(A0);
        static char temperatureCTemp[6];
        dtostrf(temp, 6, 2, temperatureCTemp);
        pCharacteristic->setValue(temperatureCTemp);
      }

      else if (pch0 == "pyro") {
        // Insert pyronometer data query
      }

      else if (pch0 == "magno") {  // Query Tilt Angle
        double heading = Magno_CompassSample(mag);
        static char compassHeading[6];
        dtostrf(heading, 6, 2, compassHeading);
        pCharacteristic->setValue(compassHeading);
      }

      if (pch0 == "accelarray") {  // Query Azimuth Angle
        double angle = Accel_ArrayTiltAngle(mma2, mma3, mma4);
        static char accelAngle[6];
        dtostrf(angle, 6, 2, accelAngle);
        pCharacteristic->setValue(accelAngle);
      }

      else {
        pCharacteristic->setValue("ERROR: pch0 Not Found; Input Command Not Recognized.");
      }
    }

    else {
      if (pch0 == "azimuthfeedbackrotate") {  // Take in Azimuth Angle
        pCharacteristic->setValue(pch);
      }

      if (pch0 == "tiltfeedbackrotate") {  // Take in Azimuth Angle
        double targetAngle = pch1.toDouble();
        pCharacteristic->setValue("Rotating Tilt");
        delay(100);
        ATControl_AccelRotate(targetAngle, mma2, mma3, mma4, 20);
      }

      if (pch0 == "kalmanangle") {
        float angleToInput;
        float currentTime;
        float timeElapsed;
        float startTime = millis();
        float numberOfSamples = pch1.toDouble();
        float angleToReturn = Accel_ArrayTiltAngle(mma2, mma3, mma4);  // Seed angleToReturn for bandpass filter
        Accel_StartKalmanFilter(mma2, mma3, mma4);

        for (int i = 0; i < numberOfSamples; ++i) {
          angleToInput = Accel_ArrayTiltAngle(mma2, mma3, mma4);
          currentTime = millis();
          timeElapsed = (currentTime - startTime) / 1000.0000;
          startTime = currentTime;

          // Bandpass filter to throw out measurements greater than/less than +/-25 degrees.
          if (abs(angleToInput - angleToReturn) > 25) {
            angleToInput = angleToReturn;
          }

          angleToReturn = Accel_KalmanFilter(angleToInput, timeElapsed);
        }
        pCharacteristic->setValue(angleToReturn);
      }

      else {
        pCharacteristic->setValue("ERROR: pch1 Found; Input Command Not Recognized.");
      }
    }
  };
};
/* /Bluetooth Variables & Command Interface */
/* /Sensor & Bluetooth Object Declaration */

/* Miscellaneous Global Variables */
double angle = 0;
String command;
float startTime = millis();
float currentTime = 0;
float timeElapsed = 0;
/* /Miscellaneous Global Variables */

/* Code to Run Once on Initialization */
void setup() {
  Serial.begin(115200);  // Start Serial interface
  delay(500);

  SetPinModes();  // Intialize GPIO Pins

  /* Initialize Magnetometer */
  if (!mag.begin(MMC56X3_DEFAULT_ADDRESS, &Wire)) {  // I2C mode
    /* There was a problem detecting the MMC5603 ... check your connections */
    Serial.println("Ooops, no MMC5603 detected ... Check your wiring!");
  }
  /* /Initialize Magnetometer */


  /* Initialize Accelerometers */
  tcaselect(2);  // Accelerometer A
  if (!mma2.begin()) {
    while (1) {
      Serial.println("MMA2 Couldnt start");
      delay(1000);
    }
  }
  Serial.println("MMA8451 (2) found!");
  mma2.setRange(MMA8451_RANGE_2_G);

  tcaselect(3);  // Accelerometer B
  if (!mma3.begin()) {
    while (1) {
      Serial.println("MMA3 Couldnt start");
      delay(1000);
    }
  }
  Serial.println("MMA8451 (3) found!");
  mma3.setRange(MMA8451_RANGE_2_G);

  tcaselect(4);  // Accelerometer C
  if (!mma4.begin()) {
    while (1) {
      Serial.println("MMA4 Couldnt start");
      delay(1000);
    }
  }
  Serial.println("MMA8451 (4) found!");
  mma4.setRange(MMA8451_RANGE_2_G);
  /* /Initialize Accelerometers */

  /* Bluetooth Server Intialization */
  BLEDevice::init("Helios_ESP32");  // Create Server
  pServer = BLEDevice::createServer();

  BLEService *pService = pServer->createService(SERVICE_UUID);  // Assign Service ID

  BLECharacteristic *pCharacteristic = pService->createCharacteristic(  // Create Read/Write Interface Characteristic
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);

  pCharacteristic->setCallbacks(new MyCallbacks());  // ???

  pCharacteristic->setValue("Hello World");  // Set Interface Default Read Value
  pService->start();                         // Start the BLE Service


  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");
  /* /Bluetooth Server Intialization */
}
/* /Code to Run Once on Initialization */

/* Code to Run Continuously */
void loop() {

  /* Bluetooth Connectivity */
  delay(3);  // Bluetooth stack goes into congestion, wait to alleviate
  if (deviceConnected) {
  }

  if (!deviceConnected && oldDeviceConnected) {
    delay(500);                   // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising();  // restart advertising
    Serial.println("start advertising");
    oldDeviceConnected = deviceConnected;
  }

  // connecting
  if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected;
  }
  /* /Bluetooth Connectivity */

  /* Serial Command Interface */
  if (Serial.available()) {
    command = Serial.readStringUntil('\n');  // Sample Serial Input

    if (command.equals("halt")) {
      Serial.println("Halting all motion operations");
      ATControl_PinConfigure('A', 'S', 0);
      ATControl_PinConfigure('T', 'S', 0);
      SHControl_PinConfigure('R', 'S', 0);
      SHControl_PinConfigure('L', 'S', 0);
    }

    else if (command.equals("azimuthcw")) {
      Serial.println("Engaging Azimuth (CW)");
      ATControl_PinConfigure('A', 'B', 35);
    }

    else if (command.equals("azimuthccw")) {
      Serial.println("Engaging Azimuth (CCW)");
      ATControl_PinConfigure('A', 'F', 35);
    }

    else if (command.equals("tiltforwards")) {
      Serial.println("Engaging Tilt (Forwards)");
      ATControl_PinConfigure('T', 'F', 35);
      while (!command.equals("halt")) {
        Serial.print("accelarray: ");
        angle = Accel_ArrayTiltAngle(mma2, mma3, mma4);
        Serial.println(angle);
        if (Serial.available()) {
          command = Serial.readStringUntil('\n');
        }
      }
    }

    else if (command.equals("tiltbackwards")) {
      Serial.println("Engaging Tilt (Backwards)");
      ATControl_PinConfigure('T', 'B', 35);
      while (!command.equals("halt")) {
        Serial.print("accelarray: ");
        angle = Accel_ArrayTiltAngle(mma2, mma3, mma4);
        Serial.println(angle);
        if (Serial.available()) {
          command = Serial.readStringUntil('\n');
        }
      }
    }

    else if (command.equals("rightshutteropen")) {
      Serial.println("Engaging Right Shutter (Open)");
      SHControl_PinConfigure('R', 'F', 35);
    }

    else if (command.equals("rightshutterclose")) {
      Serial.println("Engaging Right Shutter (Close)");
      SHControl_PinConfigure('R', 'B', 35);
    }

    else if (command.equals("leftshutteropen")) {
      Serial.println("Engaging Left Shutter (Open)");
      SHControl_PinConfigure('L', 'F', 35);
    }

    else if (command.equals("leftshutterclose")) {
      Serial.println("Engaging Left Shutter (Close)");
      SHControl_PinConfigure('L', 'B', 35);
    }

    else if (command.equals("thermo")) {
      while (!command.equals("halt")) {
        Serial.print("Temp: ");
        Serial.println(Thermo_ReadTemp(A0));
        if (Serial.available()) {
          command = Serial.readStringUntil('\n');
        }
      }
    }

    else if (command.equals("magno")) {
      while (!command.equals("halt")) {
        //Serial.print("Magno: ");
        Magno_CompassSample(mag);

        if (Serial.available()) {
          command = Serial.readStringUntil('\n');
        }
      }
    }

    else if (command.equals("accela")) {
      while (!command.equals("halt")) {
        Serial.print("accela: ");
        angle = Accel_TiltAngle(mma2, 2);
        Serial.println(angle);
        if (Serial.available()) {
          command = Serial.readStringUntil('\n');
        }
      }
    }

    else if (command.equals("accelb")) {
      while (!command.equals("halt")) {
        Serial.print("accelb: ");
        angle = Accel_TiltAngle(mma3, 3);
        Serial.println(angle);
        if (Serial.available()) {
          command = Serial.readStringUntil('\n');
        }
      }
    }

    else if (command.equals("accelc")) {
      while (!command.equals("halt")) {
        Serial.print("accelc: ");
        angle = Accel_TiltAngle(mma4, 4);
        Serial.println(angle);
        if (Serial.available()) {
          command = Serial.readStringUntil('\n');
        }
      }
    }

    else if (command.equals("accelarray")) {
      while (!command.equals("halt")) {
        Serial.print("accelarray: ");
        angle = Accel_ArrayTiltAngle(mma2, mma3, mma4);
        Serial.println(angle);
        if (Serial.available()) {
          command = Serial.readStringUntil('\n');
        }
      }
    }

    else if (command.equals("magnofeedbackrotate")) {
      bool finished = false;
      while (!finished) {
        Serial.println("Starting Magnotometer Feedback Loop; enter desired angle:");
        if (Serial.available()) {
          command = Serial.readStringUntil('\n');  // Sample Serial Input
          double desiredAngle = command.toDouble();
          if (!command.equals("")) {
            ATControl_MagnoRotate(desiredAngle, mag, 20);
            finished = true;
          }
        }
      }
    }

    else if (command.equals("accelfeedbacktilt")) {
      bool finished = false;
      while (!finished) {
        Serial.println("Starting Accelerometer Feedback Loop; enter desired angle:");
        if (Serial.available()) {
          command = Serial.readStringUntil('\n');  // Sample Serial Input
          double desiredAngle = command.toDouble();
          if (!command.equals("")) {
            ATControl_AccelRotate(desiredAngle, mma2, mma3, mma4, 25);
            finished = true;
          }
        }
      }
    }

    else if (command.equals("accelkalman")) {
      float angleToInput;
      float angleToReturn = Accel_ArrayTiltAngle(mma2, mma3, mma4);  // Seed angleToReturn for bandpass filter
      Accel_StartKalmanFilter(mma2, mma3, mma4);

      while (!command.equals("halt")) {
        angleToInput = Accel_ArrayTiltAngle(mma2, mma3, mma4);
        currentTime = millis();
        timeElapsed = (currentTime - startTime) / 1000.0000;
        startTime = currentTime;

        // Bandpass filter to throw out measurements greater than/less than +/-25 degrees.
        if (abs(angleToInput - angleToReturn) > 25) {
          angleToInput = angleToReturn;
        }

        angleToReturn = Accel_KalmanFilter(angleToInput, timeElapsed);

        Serial.print(currentTime);
        Serial.print(",");
        Serial.print(angleToInput);
        Serial.print(",");
        Serial.println(angleToReturn);

        if (Serial.available()) {
          command = Serial.readStringUntil('\n');
        }
      }
    }

    else if (command.equals("timeelapsed")) {
      currentTime = millis();
      timeElapsed = (currentTime - startTime) / 1000.0000;
      startTime = currentTime;
      Serial.println(timeElapsed);
    }

    else if (command.equals("magnokalman")) {
      float angleToInput;
      float angleToReturn = Magno_CompassSample(mag);  // Seed angleToReturn for bandpass filter
      Magno_StartKalmanFilter(mag);

      while (!command.equals("halt")) {
        angleToInput = Magno_CompassSample(mag);
        currentTime = millis();
        timeElapsed = (currentTime - startTime) / 1000.0000;
        startTime = currentTime;


        angleToReturn = Magno_KalmanFilter(angleToInput, timeElapsed);

        Serial.print(currentTime);
        Serial.print(",");
        Serial.print(angleToInput);
        Serial.print(",");
        Serial.println(angleToReturn);

        if (Serial.available()) {
          command = Serial.readStringUntil('\n');
        }
      }
    }

    else if (command.equals("rotarytracking")){
      bool rotaryfinished = false;
      while (!rotaryfinished) {
        Serial.println("Starting Azimuth Feedback Loop; enter desired angle:");
        if (Serial.available()) {
          command = Serial.readStringUntil('\n');  // Sample Serial Input
          double desiredRotaryAngle = command.toDouble();
          if (!command.equals("")) {
            RotaryEncode_Tracking(0,  desiredRotaryAngle);
            rotaryfinished = true;
          }
        }
      }
    }

    else if (!command.equals("")) {
      Serial.println("ERROR: Command not recognized.");
    }

    /* Serial Command Interface */
  }
}
/* /Code to Run continuously */