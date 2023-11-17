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
/* /Internal Scripts */

/* External Libraries */
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
#define temperatureCelsius
#define bleServerName "Helios_ESP32"
float temp;
unsigned long lastTime = 0;
unsigned long timerDelay = 30000;
bool deviceConnected = false;
#define SERVICE_UUID "91bad492-b950-4226-aa2b-4ede9fa42f59"

// Temperature Characteristic & Descriptor
BLECharacteristic bmeTemperatureCelsiusCharacteristics("cba1d466-344c-4be3-ab3f-189f80dd7518", BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor bmeTemperatureCelsiusDescriptor(BLEUUID((uint16_t)0x2902));

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    deviceConnected = true;
  };
  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
  }
};
/* /Bluetooth Variables */

/* Miscellaneous */
double angle = 0;
String command;
/* Miscellaneous */

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
  // Create the BLE Device
  BLEDevice::init(bleServerName);

  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *bmeService = pServer->createService(SERVICE_UUID);

  // Create BLE Characteristics and Create a BLE Descriptor
  // Temperature
  bmeService->addCharacteristic(&bmeTemperatureCelsiusCharacteristics);
  bmeTemperatureCelsiusDescriptor.setValue("BME temperature Celsius");
  bmeTemperatureCelsiusCharacteristics.addDescriptor(&bmeTemperatureCelsiusDescriptor);

  // Start the service
  bmeService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");
  /* /Bluetooth Server Intialization */
}
/* /Code to Run on Initialization */

/* Code to Run continuously */
void loop() {
  /* Bluetooth Command Interface */
  if (deviceConnected) {
    if ((millis() - lastTime) > timerDelay) {
      temp = Thermo_ReadTemp(A0);
      static char temperatureCTemp[6];
      dtostrf(temp, 6, 2, temperatureCTemp);
      //Set temperature Characteristic value and notify connected client
      bmeTemperatureCelsiusCharacteristics.setValue(temperatureCTemp);
      bmeTemperatureCelsiusCharacteristics.notify();
      Serial.print("Temperature Celsius: ");
      Serial.print(temp);
      Serial.print(" ºC");
      lastTime = millis();
    }
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
      ATControl_PinConfigure('A', 'F', 35);
    }

    else if (command.equals("azimuthccw")) {
      Serial.println("Engaging Azimuth (CCW)");
      ATControl_PinConfigure('A', 'B', 35);
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

    else if (command.equals("accelmatrix")) {
      while (!command.equals("halt")) {
        angle = Accel_ArrayTiltAngle(mma2, mma3, mma4);
        Serial.print("Tilt Angle: ");
        Serial.println(angle);
      }
    }
  }
  /* Serial Command Interface */
}
/* /Code to Run continuously */