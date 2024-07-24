/*********
  Rui Santos
  Complete instructions at https://RandomNerdTutorials.com/esp32-ble-server-client/
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*********/

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>



//BLE server name
#define bleServerName "MPU6050_ESP32"

Adafruit_MPU6050 mpu;


float roll;
float pitch;
float yaw;

// Timer variables
unsigned long lastTime = 0;
unsigned long timerDelay = 30000;

bool deviceConnected = false;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/
#define SERVICE_UUID "91bad492-b950-4226-aa2b-4ede9fa42f59"

// Temperature Characteristic and Descriptor

BLECharacteristic mpuRollCharacteristics("cba1d466-344c-4be3-ab3f-189f80dd7518", BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor mpuRollDescriptor(BLEUUID((uint16_t)0x2902));


// Humidity Characteristic and Descriptor
BLECharacteristic mpuPitchCharacteristics("ca73b3ba-39f6-4ab3-91ae-186dc9577d99", BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor mpuPitchDescriptor(BLEUUID((uint16_t)0x2903));

//Setup callbacks onConnect and onDisconnect
class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  };
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};

void init_mpu(){
  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  //setupt motion detection
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(1);
  mpu.setMotionDetectionDuration(20);
  mpu.setInterruptPinLatch(true);	// Keep it latched.  Will turn off when reinitialized.
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);

  Serial.println("");
  delay(100);
}

void setup() {
  // Start serial communication 
  Serial.begin(115200);

  // Init BME Sensor
  init_mpu();

  // Create the BLE Device
  BLEDevice::init(bleServerName);

  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *bmeService = pServer->createService(SERVICE_UUID);

  // Create BLE Characteristics and Create a BLE Descriptor
  // Temperature
  
    bmeService->addCharacteristic(&mpuRollCharacteristics);
    mpuRollDescriptor.setValue("MPU ROLL");
    mpuRollCharacteristics.addDescriptor(&mpuRollDescriptor);
 

  // Humidity
  bmeService->addCharacteristic(&mpuPitchCharacteristics);
  mpuPitchDescriptor.setValue("MPU Pitch");
  mpuPitchCharacteristics.addDescriptor(new BLE2902());
  
  // Start the service
  bmeService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");
}

void loop() {
  if (deviceConnected) {

      mpu.getMotionInterruptStatus();
        
      /* Get new sensor events with the readings */
       sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);

      roll = atan2(a.acceleration.y, a.acceleration.z) * 180 / PI *(-1);
      pitch = atan2(a.acceleration.x*(-1), sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180 / PI;
      yaw = atan2(g.gyro.y, g.gyro.z) * 180 / PI;

        static char rollTemp[6];
        dtostrf(roll, 6, 2, rollTemp);
        //Set temperature Characteristic value and notify connected client
        mpuRollCharacteristics.setValue(rollTemp);
        mpuRollCharacteristics.notify();
        Serial.print("Roll : ");
        Serial.print(roll);
     

        static char pitchTemp[6];
        dtostrf(pitch, 6, 2, pitchTemp);
        //Set temperature Characteristic value and notify connected client
        mpuPitchCharacteristics.setValue(pitchTemp);
        mpuPitchCharacteristics.notify();
        Serial.print("Pitch : ");
        Serial.println(pitch);
        
     
      // //Notify humidity reading from BME
      // static char yawTemp[6];
      // dtostrf(yaw, 6, 2, yawTemp);
      // //Set humidity Characteristic value and notify connected client
      // mpuPitchCharacteristics.setValue(yawTemp);
      // mpuPitchCharacteristics.notify();   
      // Serial.print(" - Humidity: ");
      // Serial.print(hum);
      // Serial.println(" %");
      

      delay(10);
    
  }
}