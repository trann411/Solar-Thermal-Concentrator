/* Internal Scripts */
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
/* /Internal Scripts */

/* External Libraries */
#include <stdlib.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
/* /External Libraries */

/* Bluetooth Libraries */
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
/* /Bluetooth Libraries */

/* Magnetometer Variables */
Adafruit_MMC5603 mag = Adafruit_MMC5603(12345);
/* /Magnetometer Variables */

/* Accelerometer Variables */
Adafruit_MMA8451 mma2 = Adafruit_MMA8451(2);
Adafruit_MMA8451 mma3 = Adafruit_MMA8451(3);
Adafruit_MMA8451 mma4 = Adafruit_MMA8451(4);
/* /Accelerometer Variables */

/* Bluetooth Variables */
// https://randomnerdtutorials.com/esp32-ble-server-client/
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

#define returnData
float temp;
float dataToReturn;
bool deviceConnected = false;
bool oldDeviceConnected = false;
std::string value = "null";
BLEServer* pServer = NULL;

BLECharacteristic bmeDataToReturnCharacteristics("cba1d466-344c-4be3-ab3f-189f80dd7518", BLECharacteristic::PROPERTY_NOTIFY);  // Create Continous Return Data Stream
BLEDescriptor bmeDataToReturnDescriptor(BLEUUID((uint16_t)0x2902));

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
    value = pCharacteristic->getValue();
    int savedAngle;
    deviceConnected = true;

    if (value == "thermo") {
      double temp = Thermo_ReadTemp(A0);
      static char temperatureCTemp[6];
      dtostrf(temp, 6, 2, temperatureCTemp);
      pCharacteristic->setValue(temperatureCTemp);
    }

    else if (value == "pyro") {
      // Insert pyronometer data query
    }

    else if (value == "magno") { // Query Azimuth Angle
      double angle = Accel_ArrayTiltAngle(mma2, mma3, mma4);
      static char accelAngle[6];
      dtostrf(angle, 6, 2, accelAngle);
      pCharacteristic->setValue(accelAngle);
    }

    else if (value == "accelarray") { // Query Tilt Angle
      double heading = Magno_CompassSample(mag);
      static char compassHeading[6];
      dtostrf(heading, 6, 2, compassHeading);
      pCharacteristic->setValue(compassHeading);
    }

    else if (value == "azimuthfeedbackrotate") { // Take in Azimuth Angle

    }

    else if (value == "accelfeedbacktilt") { // Take in Tilt Angle

    }

    else{
      pCharacteristic->setValue("Command not recognized.");
    }
  };

  /*
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  };

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
  */
};
/* /Bluetooth Variables */

/* Miscellaneous */
double angle = 0;
String command;
/* /Miscellaneous */

/* Kalman Function Testing *
Kalman testing;
float testRate = testing.getRate();
float testDt = 0.0001;
float testInAngle = 90;
float testOutAngle = testing.getAngle(testInAngle, testRate, testDt);

/* /Kalman Function Testing */

/* Code to Run on Initialization */
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

  pService->addCharacteristic(&bmeDataToReturnCharacteristics);
  bmeDataToReturnDescriptor.setValue("Return Data from User Query");
  bmeDataToReturnCharacteristics.addDescriptor(&bmeDataToReturnDescriptor);

  pCharacteristic->setCallbacks(new MyCallbacks());  // ???

  pCharacteristic->setValue("Hello World");  // Set Interface Default Read Value
  pService->start();                         // Start the BLE Service


  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");
  //BLEAdvertising *pAdvertising = pServer->getAdvertising(); // Advertise Presence of BLE Service
  //pAdvertising->start();
  /* /Bluetooth Server Intialization */
}
/* /Code to Run on Initialization */

/* Code to Run continuously */
void loop() {
  /* Bluetooth Command Interface */
  delay(3); // Bluetooth stack goes into congestion, wait to alleviate

  if (deviceConnected) {
    if (value == "thermo") {
      temp = Thermo_ReadTemp(A0);
      static char temperatureCTemp[6];
      dtostrf(temp, 6, 2, temperatureCTemp);
      //Set temperature Characteristic value and notify connected client
      bmeDataToReturnCharacteristics.setValue(temperatureCTemp);
      bmeDataToReturnCharacteristics.notify();
      //Serial.print("Temperature Celsius: ");
      //Serial.print(temp);
      //Serial.println(" ºC");
    }

    if (value == "magno"){
      float angle = Accel_ArrayTiltAngle(mma2, mma3, mma4);
      static char accelAngle[6];
      dtostrf(angle, 6, 2, accelAngle);
      bmeDataToReturnCharacteristics.setValue(angle);
      bmeDataToReturnCharacteristics.notify();
    }

    if (value == "accelarray"){
      float angle = Accel_ArrayTiltAngle(mma2, mma3, mma4);
      static char accelAngle[6];
      dtostrf(angle, 6, 2, accelAngle);
      bmeDataToReturnCharacteristics.setValue(angle);
      bmeDataToReturnCharacteristics.notify();
    }
  }

  if (!deviceConnected && oldDeviceConnected) {
        delay(500); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        Serial.println("start advertising");
        oldDeviceConnected = deviceConnected;
    }
    // connecting
    if (deviceConnected && !oldDeviceConnected) {
        // do stuff here on connecting
        oldDeviceConnected = deviceConnected;
    }
  /* /Bluetooth Command Interface */

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
    }

    else if (command.equals("tiltbackwards")) {
      Serial.println("Engaging Tilt (Backwards)");
      ATControl_PinConfigure('T', 'B', 35);
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
        Serial.print("Magno: ");
        Serial.println(Magno_CompassSample(mag));
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

    else if (command.equals("kalmanfilter")){
      float angleToInput;
      float angleToReturn;
      Accel_StartKalmanFilter(mma2, mma3, mma4);

      while (!command.equals("halt")) {
        angleToInput = Accel_ArrayTiltAngle(mma2, mma3, mma4);
        // To-Do: Implement Bandpass filter to throw out measurements greater than/less than +/-5 degrees.
        angleToReturn = Accel_KalmanFilter(angleToInput);
        if (Serial.available()) {
          command = Serial.readStringUntil('\n');
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