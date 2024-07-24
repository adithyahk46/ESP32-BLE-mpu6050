/*********
  Rui Santos
  Complete instructions at https://RandomNerdTutorials.com/esp32-ble-server-client/
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*********/
#include <Arduino.h>
#include "BLEDevice.h"
#include <Wire.h>


//Default mpuRoll is in Celsius
float roll;
float pitch;
float yaw;

//BLE Server name (the other ESP32 name running the server sketch)
#define bleServerName "MPU6050_ESP32"

/* UUID's of the service, characteristic that we want to read*/
// BLE Service
static BLEUUID bmeServiceUUID("91bad492-b950-4226-aa2b-4ede9fa42f59");


//mpuRoll  Characteristic
static BLEUUID mpuRollCharacteristicUUID("cba1d466-344c-4be3-ab3f-189f80dd7518");

// mpuPitch Characteristic
static BLEUUID mpuPitchCharacteristicUUID("ca73b3ba-39f6-4ab3-91ae-186dc9577d99");

//mpuYaw Characteristic
//static BLEUUID mpuYawCharacteristicUUID("f78ebbff-c8b7-4107-93de-889a6a06d408");


//Flags stating if should begin connecting and if the connection is up
static bool doConnect = false;
static bool connected = false;

//Address of the peripheral device. Address will be found during scanning...
static BLEAddress *pServerAddress;
 
//Characteristic that we want to read
static BLERemoteCharacteristic* mpuRollCharacteristic;
static BLERemoteCharacteristic* mpuPitchCharacteristic;

//Activate notify
const uint8_t notificationOn[] = {0x1, 0x0};
const uint8_t notificationOff[] = {0x0, 0x0};


//Variables to store mpuRoll and mpuPitch
char* mpuRollChar;
char* mpuPitchChar;

//Flags to check whether new mpuRoll and mpuPitch readings are available
bool newRoll = false;
bool newPitch = false;


//When the BLE Server sends a new mpuRoll reading with the notify property
static void mpuRollNotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, 
                                        uint8_t* pData, size_t length, bool isNotify) {
  //store mpuRoll value
  mpuRollChar = (char*)pData;
  newRoll = true;
}

//When the BLE Server sends a new mpuPitch reading with the notify property
static void mpuPitchNotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, 
                                    uint8_t* pData, size_t length, bool isNotify) {
  //store mpuPitch value
  mpuPitchChar = (char*)pData;
  newPitch = true;
  //Serial.print(newPitch);
}



//Connect to the BLE Server that has the name, Service, and Characteristics
bool connectToServer(BLEAddress pAddress) {
   BLEClient* pClient = BLEDevice::createClient();
 
  // Connect to the remove BLE Server.
  pClient->connect(pAddress);
  Serial.println(" - Connected to server");
 
  // Obtain a reference to the service we are after in the remote BLE server.
  BLERemoteService* pRemoteService = pClient->getService(bmeServiceUUID);
  if (pRemoteService == nullptr) {
    Serial.print("Failed to find our service UUID: ");
    Serial.println(bmeServiceUUID.toString().c_str());
    return (false);
  }
 
  // Obtain a reference to the characteristics in the service of the remote BLE server.
  mpuRollCharacteristic = pRemoteService->getCharacteristic(mpuRollCharacteristicUUID);
  mpuPitchCharacteristic = pRemoteService->getCharacteristic(mpuPitchCharacteristicUUID);

  if (mpuRollCharacteristic == nullptr || mpuPitchCharacteristic == nullptr) {
    Serial.print("Failed to find our characteristic UUID");
    return false;
  }
  Serial.println(" - Found our characteristics");
 
  //Assign callback functions for the Characteristics
  mpuRollCharacteristic->registerForNotify(mpuRollNotifyCallback);
  mpuPitchCharacteristic->registerForNotify(mpuPitchNotifyCallback);
  return true;
}

//Callback function that gets called, when another device's advertisement has been received
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    if (advertisedDevice.getName() == bleServerName) { //Check if the name of the advertiser matches
      advertisedDevice.getScan()->stop(); //Scan can be stopped, we found what we are looking for
      pServerAddress = new BLEAddress(advertisedDevice.getAddress()); //Address of advertiser is the one we need
      doConnect = true; //Set indicator, stating that we are ready to connect
      Serial.println("Device found. Connecting!");
    }
  }
};
 


void setup() {
 
  //Start serial communication
  Serial.begin(115200);
  Serial.println("Starting Arduino BLE Client application...");

  //Init BLE device
  BLEDevice::init("");
 
  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 30 seconds.
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->start(30);
}

void loop() {
  // If the flag "doConnect" is true then we have scanned for and found the desired
  // BLE Server with which we wish to connect.  Now we connect to it.  Once we are
  // connected we set the connected flag to be true.
  if (doConnect == true) {
    if (connectToServer(*pServerAddress)) {
      Serial.println("We are now connected to the BLE Server.");
      //Activate the Notify property of each Characteristic
      mpuRollCharacteristic->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*)notificationOn, 2, true);
      mpuPitchCharacteristic->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*)notificationOn, 2, true);
      connected = true;
    } else {
      Serial.println("We have failed to connect to the server; Restart your device to scan for nearby BLE server again.");
    }
    doConnect = false;
  }
  //if new mpuRoll readings are available, print in the OLED
  if (newRoll && newPitch){
    newRoll = false;
    newPitch = false;
    Serial.print(mpuRollChar);
    Serial.print("\t");
    Serial.println(mpuPitchChar);
  }
  delay(10); // Delay a second between loops.
}