/*
  Callback LED

  This example creates a BLE peripheral with service that contains a
  characteristic to control an LED. The callback features of the
  library are used.

  The circuit:
  - Arduino MKR WiFi 1010, Arduino Uno WiFi Rev2 board, Arduino Nano 33 IoT,
    Arduino Nano 33 BLE, or Arduino Nano 33 BLE Sense board.

  You can use a generic BLE central app, like LightBlue (iOS and Android) or
  nRF Connect (Android), to interact with the services and characteristics
  created in this sketch.

  This example code is in the public domain.
*/

#include <ArduinoBLE.h>
#include <SPI.h>
#include <SD.h>
#include "EEPROM.h"

// Give a default if the variant does not define one.
#ifndef A0
#define A0 0
#endif

#define    MESSAGE_LEN    19
int i=0;
File myFile;
//preferences_t prefs;

#define EEPROM_VALID_IDX  (0)
#define EEPROM_VALID_CODE (0xAB)
uint32_t position=0;

BLEService ledService("19B10000-E8F2-537E-4F6C-D104768A1214"); // create service

BLEStringCharacteristic switchCharacteristic  ("FB349B5F-8000-0080-0010-000000212000", BLERead | BLEWrite, MESSAGE_LEN);

const int ledPin = LED_BUILTIN; // pin to use for the LED

void setup() {
  Serial.begin(9600);
  while (!Serial);
  
  pinMode(ledPin, OUTPUT); // use the LED pin as an output

  // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");

    while (1);
  }

  // set the local name peripheral advertises
  BLE.setLocalName("LEDCallback");
  // set the UUID for the service this peripheral advertises
  BLE.setAdvertisedService(ledService);

  // add the characteristic to the service
  ledService.addCharacteristic(switchCharacteristic);

  // add service
  BLE.addService(ledService);

  // assign event handlers for connected, disconnected to peripheral
  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

  // assign event handlers for characteristic
  switchCharacteristic.setEventHandler(BLEWritten, switchCharacteristicWritten);
  // set an initial value for the characteristic

  switchCharacteristic.setValue("h");

  // start advertising
  BLE.advertise();

  
  Serial.println(("Bluetooth device active, waiting for connections..."));

  eeprom_setup();
  init_file();
  //myFileRead();

  
}

void loop() {
  // poll for BLE events
  BLE.poll();
}

void blePeripheralConnectHandler(BLEDevice central) {
  // central connected event handler
  Serial.print("Connected event, central: ");
  Serial.println(central.address());
}

void blePeripheralDisconnectHandler(BLEDevice central) {
  // central disconnected event handler

    write_file();
    myFile.close();
  Serial.print("Disconnected event, central: ");
  Serial.println(central.address());
  
}

void switchCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  // central wrote new value to characteristic, update LED

 
    Serial.print("Pointer position: ");
    String val = switchCharacteristic.value();
    //Serial.println(val);
    val.remove(0,1);
    
    EEPROM.put(position,val);
    Serial.println(position);
    //position++;
    position+=val.length();
  
 // myFile.print((unsigned long)switchCharacteristic.value(),HEX);
   
   //myFile.print(val);
   
//  static bool _write_en = false;
// 
//  if(val.equals( "START"))
//    _write_en = true;
  
//  if(true)
//  {

  
 // Serial.println("Writting to file firm");
 
    //myFileWrite(val);
    i++;
  
//    if(val.equals( "STOP"))
//    {
//      myFile.close();
//      _write_en = false;
//      i==0;
//     
//    }
//  }
  
  if (true) {
    //Serial.println("LED on");
    digitalWrite(ledPin, HIGH);
    delay(1000);
    //Serial.println("LED off");
    digitalWrite(ledPin, LOW);
    delay(1000);
  }
  
}

////////////////////////////////////////////////////////////////////////////
void init_file()
{
   Serial.print("Initializing SD card...");

  if (!SD.begin(28)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  myFile = SD.open("firm.bin", FILE_WRITE);
//   myFile = SD.open("firm.bin", FILE_READ);
//  // if the file opened okay, write to it:
  if (myFile) {
    Serial.println("Opend file firm.bin");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening firm.bin");
  }
}

void myFileRead()
{  
   myFile = SD.open("FIRM.bin");
    if (myFile) {
    Serial.println("firm.bin:");

    // read from the file until there's nothing else in it:
    while (myFile.available()) {
      Serial.write(myFile.read());
    }
   
    // close the file:
    myFile.close();
  } 
  else {
    // if the file didn't open, print an error:
    Serial.println("error opening FIRM.bin");
  }
}

//void myFileWrite(String data)
//{
//  myFile.println(data);  
//}
void write_file()
{
  String readata;
  for(int i=0;i<position;i++)
  {
    EEPROM.get(position, readata);
    //Serial.println(readata);
     myFile.print(readata);
  }
}
void eeprom_setup()
{

  Serial.println("EEPROM Examples");

  randomSeed(analogRead(A0));

  EEPROM.init();

  // You may choose to enable more or less EEPROM -
  // Default length is 1024 bytes (if setLength is not called)
  EEPROM.setLength(3 * 1024); // this would make the length 1080 bytes. Max is 4096.
  // Note: larger sizes will increase RAM usage and execution time
  Serial.print("Size of EEPROM is : ");
  Serial.println(EEPROM.length());
  long startTime;
  long endTime;
  int randomLocation;

  //Test erase time
  startTime = millis();
  EEPROM.erase();
  endTime = millis();
  Serial.printf("Time to erase all EEPROM: %dms\n", endTime - startTime);

  //String write test
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  
  Serial.println();
  Serial.println("String test");
  

  char myString[19] = "How are you today?";
  //randomLocation = random(0, EEPROM.length() - sizeof(myString));
  EEPROM.put(0, myString);

  char readMy[19];
  EEPROM.read(0, (uint8_t *)readMy, sizeof(readMy));
  Serial.printf("Location %d string should read 'How are you today?': ", randomLocation);
  Serial.println(readMy);
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  


}
