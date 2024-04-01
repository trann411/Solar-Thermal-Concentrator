

/* Include Internal Scripts */
#include "Accel.h"                  // Accelerometer Functions
#include "AzimuthTiltController.h"  // Azimuth/Tilt Motor Control Functions
#include "Debug.h"                  // Debug Commands
#include "Helper.h"                 // Common-Use Functions
#include "Magno.h"                  // Magnetometer Functions
#include "Pyro.h"                   // Pyranometer Functions
#include "RotaryEncode.h"           // Rotary Encoder Functions
#include "ShutterController.h"      // Shutter Motor Controller Functions
#include "Thermo.h"                 // Thermocouple Functions
#include "Kalman.h"                 // Kalman Filter Functions
/* /Include Internal Scripts */

/* Include External Libraries */
#include <stdlib.h>
#include <Adafruit_MMA8451.h>  // Adafruit Accelerometer Scripts
#include <Adafruit_Sensor.h>   // Adafruit Miscellaneous Sensor Scripts
#include <Wire.h>              // I2C Scripts
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

/* Miscellaneous Global Variables */
float temp;
float dataToReturn;
bool deviceConnected = false;
bool oldDeviceConnected = false;
String input = "null";
BLEServer *pServer = NULL;
char *pch = NULL;
String pch0;  // Text Command
String pch1;  // Numeric Value (if applicable)
char bluetoothReturnData[6] = "NULL";
double presentAngle = 90;  // Internal Azimuth Position
double angle = 0;
String command;
float startTime = millis();
float currentTime = 0;
float timeElapsed = 0;
/* Miscellaneous Global Variables */

/* Bluetooth-ESP32 doAction Flags */
bool doThermo = false;
bool doPyro = false;
bool doMagno = false;
bool doAccelArray = false;
bool doDelayTest = false;
bool doTiltForwards = false;
bool doTiltBackwards = false;
bool doAzimuthCW = false;
bool doAzimuthCCW = false;
bool doHalt = false;
bool doTiltStage1 = false;
bool doTiltStage2 = false;
bool doTiltStage3 = false;
bool doKalmanAngle = false;
bool doAzimuthStage = false;
bool doLevel = false;
bool doPyroSweep = false;
/* /Bluetooth-ESP32 doAction Flags */


/* Bluetooth Callbacks */
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer, esp_ble_gatts_cb_param_t *param) {
    //pServer->updateConnParams( param->connect.remote_bda, 0x06, 0x12, 0, 200000 ); // Prevent timeout disconnection if requested command takes awhile to execute (i.e. tilt rotate)
    //BLEDevice::startAdvertising();
    /*
    esp
    memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
    conn_params.latency = 10;
    conn_params.max_int = 0x20;
    con
    */
    deviceConnected = true;
  };

  void onDisconnect(BLEServer *pServer, esp_ble_gatts_cb_param_t *param) {
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
        doThermo = true;
      }

      if (pch0 == "pyro") {
        doPyro = true;
      }

      if (pch0 == "magno") {  // Query Azimuth Angle
        doMagno = true;
      }

      if (pch0 == "accelarray") {  // Query Tilt Angle
        doAccelArray = true;
      }

      if (pch0 == "delaytest") {
        doDelayTest = true;
      }

      if (pch0 == "tiltforwards") {
        doTiltForwards = true;
      }

      if (pch0 == "tiltbackwards") {
        doTiltBackwards = true;
      }

      if (pch0 == "azimuthcw") {
        doAzimuthCW = true;
      }

      if (pch0 == "azimuthccw") {
        doAzimuthCCW = true;
      }

      if (pch0 == "halt") {
        doHalt = true;
      }

      if (pch0 == "level") {
        doLevel = true;
      }

    }

    else {
      if (pch0 == "azimuthstage") {  // Take in Azimuth Angle
        doAzimuthStage = true;
      }

      if (pch0 == "tiltstage1") {  // Take in Azimuth Angle
        doTiltStage1 = true;
      }

      if (pch0 == "tiltstage2") {  // Take in Azimuth Angle
        doTiltStage2 = true;
      }

      if (pch0 == "tiltstage3") {  // Take in Azimuth Angle
        doTiltStage3 = true;
      }

      if (pch0 == "kalmanangle") {  // Take in Azimuth Angle
        doKalmanAngle = true;
      }

      if (pch0 == "pyrosweep") {
        doPyroSweep = true;
      }


      //Reset Command Flags at end of loops so not stuck requesting same thing
      //pch0 = "";
      //pch1 = "";
    };
  };
  void onRead(BLECharacteristic *pCharacteristic) {
    pCharacteristic->setValue(bluetoothReturnData);
  };
};
/* /Bluetooth Callbacks */

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
    //esp_ble_gatts_cb_param_t* param;
    //pServer->updateConnParams(param->connect.remote_bda, 100, 100, 0, 1000);
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

  /* Bluetooth Command Interface */
  if (doThermo) {
    double temp = Thermo_ReadTemp(A0);
    dtostrf(temp, 6, 2, bluetoothReturnData);
    //pCharacteristic->setValue(temperatureCTemp);
    doThermo = false;  // Set flag back to low or else will be locked into this functionality
  }

  if (doPyro) {
    // Insert pyronometer data query
    Serial.println("Doing Pyro");
    doPyro = false;
  }

  if (doMagno) {  // Query Azimuth Angle
    //double heading = Magno_CompassSample(mag);
    static char compassHeading[6];
    dtostrf(presentAngle, 6, 2, bluetoothReturnData);
    //pCharacteristic->setValue(compassHeading);
    doMagno = false;
  }

  if (doAccelArray) {  // Query Tilt Angle
    double angle = Accel_ArrayTiltAngle(mma2, mma3, mma4);
    static char accelAngle[6];
    dtostrf(angle, 6, 2, bluetoothReturnData);
    //pCharacteristic->setValue(accelAngle);
    doAccelArray = false;
  }

  if (doDelayTest) {
    //pCharacteristic->setValue("Begin delay test");
    delay(20000);
    // pCharacteristic->setValue("Pass delay test");
    doDelayTest = false;
  }

  if (doTiltForwards) {
    ATControl_PinConfigure('T', 'F', 95);
    doTiltForwards = false;
  }

  if (doTiltBackwards) {
    ATControl_PinConfigure('T', 'B', 95);
    doTiltBackwards = false;
  }

  if (doAzimuthCW) {
    ATControl_PinConfigure('A', 'B', 80);
    //pCharacteristic->setValue("Engaged Azimuth CW Motion");
    doAzimuthCW = false;
  }

  if (doAzimuthCCW) {
    ATControl_PinConfigure('A', 'F', 80);
    //pCharacteristic->setValue("Engaged Azimuth CCW Motion");
    doAzimuthCCW = false;
  }

  if (doHalt) {
    ATControl_PinConfigure('A', 'S', 80);
    ATControl_PinConfigure('T', 'S', 80);
    // pCharacteristic->setValue("Halted all motion");
    doHalt = false;
  }

  if (doLevel) {
    ATControl_AccelRotate(90, mma2, mma3, mma4, 1);  // Return to nominal concentrator position
    presentAngle = ATControl_MagnoRotate(90, presentAngle);
    doLevel = false;
  }

  if (doAzimuthStage) {  // Take in Azimuth Angle
    double targetAngle = pch1.toDouble();
    presentAngle = ATControl_MagnoRotate(targetAngle, presentAngle);
    dtostrf(presentAngle, 6, 2, bluetoothReturnData);
    doAzimuthStage = false;
  }

  if (doTiltStage1) {  // Take in Azimuth Angle
    double targetAngle = pch1.toDouble();
    ATControl_AccelRotate(targetAngle, mma2, mma3, mma4, 1);
    doTiltStage1 = false;
  }

  if (doTiltStage2) {  // Take in Azimuth Angle
    double targetAngle = pch1.toDouble();
    ATControl_AccelRotate(targetAngle, mma2, mma3, mma4, 2);
    doTiltStage2 = false;
  }

  if (doTiltStage3) {  // Take in Azimuth Angle
    double targetAngle = pch1.toDouble();
    ATControl_AccelRotate(targetAngle, mma2, mma3, mma4, 3);
    doTiltStage3 = false;
  }

  if (doKalmanAngle) {  // UPDATE WITH MORE RECENT ALGORITHM
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

      //Bandpass filter to throw out measurements greater than/less than +/-25 degrees.
      if (abs(angleToInput - angleToReturn) > 25) {
        angleToInput = angleToReturn;
      }

      angleToReturn = Accel_KalmanFilter(angleToInput, timeElapsed);
    }
    static char accelAngle[6];
    dtostrf(angleToReturn, 6, 2, bluetoothReturnData);
    doKalmanAngle = false;
    // pCharacteristic->setValue(accelAngle);
  }

  if (doPyroSweep) {
    ATControl_PyroSweep(A0, presentAngle);
    presentAngle = pch1.toDouble();
    doPyroSweep = false;
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
      ATControl_PinConfigure('A', 'B', 80);
    }

    else if (command.equals("azimuthccw")) {
      Serial.println("Engaging Azimuth (CCW)");
      ATControl_PinConfigure('A', 'F', 80);
    }

    else if (command.equals("tiltforwards")) {
      Serial.println("Engaging Tilt (Forwards)");
      ATControl_PinConfigure('T', 'F', 95);
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
      ATControl_PinConfigure('T', 'B', 95);
      while (!command.equals("halt")) {
        Serial.print("accelarray: ");
        angle = Accel_ArrayTiltAngle(mma2, mma3, mma4);
        Serial.println(angle);
        if (Serial.available()) {
          command = Serial.readStringUntil('\n');
        }
      }
    }

    else if (command.equals("tiltstage1")) {
      bool finished = false;
      while (!finished) {
        Serial.println("Starting Tilt Stage 1; enter desired angle:");
        if (Serial.available()) {
          command = Serial.readStringUntil('\n');  // Sample Serial Input
          double desiredAngle = command.toDouble();
          if (!command.equals("")) {
            ATControl_AccelRotate(desiredAngle, mma2, mma3, mma4, 1);
            finished = true;
          }
        }
      }
    }

    else if (command.equals("tiltstage2")) {
      bool finished = false;
      while (!finished) {
        Serial.println("Starting Tilt Stage 2; enter desired angle:");
        if (Serial.available()) {
          command = Serial.readStringUntil('\n');  // Sample Serial Input
          double desiredAngle = command.toDouble();
          if (!command.equals("")) {
            ATControl_AccelRotate(desiredAngle, mma2, mma3, mma4, 2);
            finished = true;
          }
        }
      }
    }

    else if (command.equals("tiltstage3")) {
      bool finished = false;
      while (!finished) {
        Serial.println("Starting Tilt Stage 3; enter desired angle:");
        if (Serial.available()) {
          command = Serial.readStringUntil('\n');  // Sample Serial Input
          double desiredAngle = command.toDouble();
          if (!command.equals("")) {
            ATControl_AccelRotate(desiredAngle, mma2, mma3, mma4, 3);
            finished = true;
          }
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
      Serial.println(presentAngle);
    }

    else if (command.equals("setMagno")) {
      bool finished = false;
      Serial.println("Enter current azimuth angle:");

      while (!finished) {
        if (Serial.available()) {
          command = Serial.readStringUntil('\n');  // Sample Serial Input
          double desiredAngle = command.toDouble();
          if (!command.equals("")) {
            presentAngle = desiredAngle;
            finished = true;
          }
        }
      }
    }

    else if (command.equals("pyro")) {
      while (!command.equals("halt")) {
        double pyroReading = Pyro_ReadTemp(A1);
        Serial.println(pyroReading);
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
            presentAngle = ATControl_MagnoRotate(desiredAngle, presentAngle);
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
            //ATControl_AccelRotate(desiredAngle, mma2, mma3, mma4, 25);
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

    else if (command.equals("rotarytracking")) {
      bool rotaryfinished = false;
      while (!rotaryfinished) {
        Serial.println("Starting Azimuth Feedback Loop; enter desired angle:");
        if (Serial.available()) {
          command = Serial.readStringUntil('\n');  // Sample Serial Input
          double desiredRotaryAngle = command.toDouble();
          if (!command.equals("")) {
            RotaryEncode_Tracking(100);
            rotaryfinished = true;
          }
        }
      }
    }

    else if (command.equals("rotarychar")) {
      ATControl_PinConfigure('A', 'F', 25);
      //RotaryEncode_Tracking(90);
    }

    else if (command.equals("pyrosweep")) {
      presentAngle = ATControl_PyroSweep(A0, presentAngle);
    }

    else if (command.equals("pyro")) {
      while (!command.equals("halt")) {
        double pyroReading = Pyro_ReadTemp(A1);
        Serial.println(pyroReading);
        if (Serial.available()) {
          command = Serial.readStringUntil('\n');
        }
      }
    }

    else if (command.equals("level")) {
      ATControl_AccelRotate(82.5, mma2, mma3, mma4, 1);  // Return to nominal concentrator position
      ATControl_AccelRotate(82.5, mma2, mma3, mma4, 2);  // Return to nominal concentrator position
      ATControl_AccelRotate(82.5, mma2, mma3, mma4, 3);  // Return to nominal concentrator position

      presentAngle = ATControl_MagnoRotate(90, presentAngle);
    }

    else if (command.equals("highspeedchar")) {
      double highSpeedStartTime = millis();
      ATControl_PinConfigure('T', 'F', 95);
      delay(10000);
      ATControl_PinConfigure('T', 'S', 0);
      double highSpeedResult = millis() - highSpeedStartTime;
      Serial.print("High Speed Travel Time");
      Serial.println(highSpeedResult);
    }

    else if (command.equals("midspeedchar")) {
      double midSpeedStartTime = millis();
      ATControl_PinConfigure('T', 'F', 55);
      delay(10000);
      ATControl_PinConfigure('T', 'S', 0);
      double midSpeedResult = millis() - midSpeedStartTime;
      Serial.print("Mid Speed Travel Time");
      Serial.println(midSpeedResult);
    }

    else if (command.equals("lowspeedchar")) {
      double lowSpeedStartTime = millis();
      ATControl_PinConfigure('T', 'F', 20);
      delay(10000);
      ATControl_PinConfigure('T', 'S', 0);
      double lowSpeedResult = millis() - lowSpeedStartTime;
      Serial.print("Low Speed Travel Time");
      Serial.println(lowSpeedResult);
    }

    else if (!command.equals("")) {
      Serial.println("ERROR: Command not recognized.");
    }

    /* Serial Command Interface */
  }
}
/* /Code to Run continuously */