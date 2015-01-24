//*******************************************************************************
//*
//* Arduino based RF24Network sensor node
//* Based on example sketches for RF24Network by J. Coliz <maniacbug@ymail.com> / TMRh20
//*
//*******************************************************************************
//* Processor:  Arduino Pro Mini 3.3V/ 8 or 16MHz
//* Author      Date       Comment
//*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//* Kubik       14.1.2015 First version, just basic code for HW tests
//* Kubik       20.1.2015 Added and polished the sensor support
//* Kubik       22.1.2015 Added battery support
//*******************************************************************************

//*******************************************************************************
//*                            HW details                                       *
//*******************************************************************************
// Standard NRF24L01+ module (eBay)
// One DS1820 style sensor
//
// Used pins:
//  NRF24L01+   SPI + 9, 10
//  DS1820		8

//*******************************************************************************
//*                           Includes and defines                              *
//******************************************************************************* 

#define VERSION 0
#define DEBUG_OW_TEMP 1
#define DEBUG_RF24 1
#define DEBUG 1

#include <RF24Network.h>
#include <RF24.h>
#include <SPI.h>
#include <OneWire.h>
#include <LowPower.h>
#include <Rf24PacketDefine.h>
#include <EEPROM.h>

#define THIS_NODE_DEFAULT_ADDRESS 05      // Address of our node in Octal format - will be used if EEPROM isn't valid
#define THIS_NODE_DEFAULT_ID "TempNode  " // Default node ID, used when EEPROM isn't valid
#define PIN_BATTERY_ANALOG A0
#define PIN_BATTERY_MEASURE 15
#define BATTERY_K 480L                    // Battery conversion constant
#define BATTERY_TYPE F_BATTERY_NONE       // Type of the battery this node uses - NONE means net powered 

#define TIME_COUNTER_TEMP 2
#define TIME_COUNTER_ID 2

#define F_DEFAULTS 0x0000

typedef struct {
  uint32_t Crc;
  uint16_t Header;
  char Id[NODE_ID_SIZE];
  uint16_t NodeAddress;
  uint16_t BatteryCoefficient;
  uint16_t Flags;
} Eeprom_t;

//*******************************************************************************
//*                               Static variables                              *
//*******************************************************************************
 
//
// RF24 variables
//

RF24 Radio (9, 10);              // nRF24L01(+) radio attached using standard pins 9,10
RF24Network Network (Radio);    // Network uses that radio

PayloadTemperature_t PayloadTemperature;
PayloadId_t PayloadId;

//
// OneWire variables
//

OneWire Sensor (8);            // on pin 7 (a 4.7K resistor is necessary)
byte SensorAddress[8];
byte SensorType; 

//
// EEPROM variables
//

Eeprom_t Eeprom;

//
// Other variables
//

uint16_t BattLevelAverage;
uint16_t Flags = 0;

//*******************************************************************************
//*                          Debug and assert support                           *
//*******************************************************************************

#if DEBUG
  #define Debug(...) {Serial.print(__VA_ARGS__); delay (50);}
  #define Debugln(...) {Serial.println(__VA_ARGS__); delay (50);}
#else
  #define Debug(...)
  #define Debugln(...)
#endif 


void Assert (void ) {
  
  Debugln ("*** Assert! ***");
  delay (100);
  // BUGBUG - how to report missing sensor?
  while (1) {
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
  }
}  

//*******************************************************************************
//*                                  CRC32 support                              *
//*******************************************************************************

#include <avr/pgmspace.h>

static PROGMEM prog_uint32_t CrcTable[16] = {
  0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
  0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
  0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
  0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
};

uint32_t CrcUpdate (uint32_t Crc, uint8_t Data) {
  byte TblIdx;
  TblIdx = Crc ^ (Data >> (0 * 4));
  Crc = pgm_read_dword_near(CrcTable + (TblIdx & 0x0f)) ^ (Crc >> 4);
  TblIdx = Crc ^ (Data >> (1 * 4));
  Crc = pgm_read_dword_near(CrcTable + (TblIdx & 0x0f)) ^ (Crc >> 4);
  return Crc;
}

uint32_t CrcBuffer (byte *Buffer, byte Size) {

  uint32_t Crc = ~0L;
  while (Size > 0) {
    Crc = CrcUpdate (Crc, *Buffer++);
    Size--;
  }
  Crc = ~Crc;
  return Crc;
}

//*******************************************************************************
//*                                  EEPROM support                             *
//*******************************************************************************

//
// Read EEPROM from address 0 to Eeprom. 
// Check the CRC, compare it with stored CRC.
// If it does not match, load Eeprom with defaults.
//

bool EepromRead (void) {
  byte i;
  byte *EepromPtr;
  uint32_t Crc;

  EepromPtr = (byte *) &Eeprom;
  for (i = 0; i < sizeof (Eeprom_t); i++) {
    *EepromPtr++ = EEPROM.read (i);
  }
  
  EepromPtr = (byte *) &Eeprom;
  Crc = CrcBuffer (EepromPtr + 4, sizeof (Eeprom_t) - 4);    // Do not include stored CRC into CRC calculation
  if (Crc != Eeprom.Crc) {
    Eeprom.Flags = F_DEFAULTS;
    Eeprom.Header = 'ID';
    Eeprom.NodeAddress = THIS_NODE_DEFAULT_ADDRESS;
    Eeprom.BatteryCoefficient = BATTERY_K;
    memcpy (Eeprom.Id, THIS_NODE_DEFAULT_ID, sizeof (Eeprom.Id));
    return false;
  }
  return true;
}
  
//
// Write the content of Eeprom structure to EEPROM.
// Compute CRC and write it into the structure as well.
//

void EepromWrite (void) {
  byte i;
  byte *EepromPtr;
  uint32_t Crc;

  EepromPtr = (byte *) &Eeprom;
  Crc = CrcBuffer (EepromPtr + 4, sizeof (Eeprom_t) - 4);    // Do not include stored CRC into CRC calculation
  Eeprom.Crc = Crc;

  EepromPtr = (byte *) &Eeprom;
  for (i = 0; i < sizeof (Eeprom_t); i++) {
    EEPROM.write (i, *EepromPtr++);
  }
}
 

//*******************************************************************************
//*                               DS1820 support                                *
//*******************************************************************************

#if DEBUG_OW_TEMP
  #define DebugOwTemp(...) {Serial.print(__VA_ARGS__); delay (50);}
  #define DebugOwTempln(...) {Serial.println(__VA_ARGS__); delay (50);}
#else
  #define DebugOwTemp(...)
  #define DebugOwTempln(...)
#endif 

uint16_t SensorRead (void) {
  byte SensorData[12];
  byte i;
  int16_t Raw;
  byte Cfg;
  
  Sensor.reset ();
  Sensor.select (SensorAddress);
  Sensor.write (0x44, 1);                          // start conversion, with parasite power on at the end
  
  LowPower.powerDown (SLEEP_1S, ADC_OFF, BOD_OFF);     
  
  Sensor.reset ();
  Sensor.select (SensorAddress);    
  Sensor.write (0xBE);                             // Read Scratchpad

  for ( i = 0; i < 9; i++) {                       // we need 9 bytes
    SensorData[i] = Sensor.read();
  }

  Raw = (SensorData[1] << 8) | SensorData[0];
  if (SensorType) {
    Raw <<= 3;                                     // 9 bit resolution default
    if (SensorData [7] == 0x10) {
      Raw = (Raw & 0xFFF0) + 12 - SensorData [6];  // count remain gives full 12 bit resolution
    }
  } else {
    Cfg = (SensorData [4] & 0x60);
    if (Cfg == 0x00) {
      Raw <<= 3;                                  // 9 bit resolution, 93.75 ms
    } else if (Cfg == 0x20) {
      Raw <<= 2;                                 // 10 bit res, 187.5 ms
    } else if (Cfg == 0x40) {
      Raw <<= 1;                                 // 11 bit res, 375 ms
    }
    
    //
    // default is 12 bit resolution, 750 ms conversion time
    //
  }

  DebugOwTemp (F("Raw = "));
  DebugOwTempln (Raw);
  return (Raw * 5 + 4) / 8; 
}

void DS1820Init (void) {
  byte i;
  
  if (!Sensor.search (SensorAddress)) {
    DebugOwTempln (F("No sensor found, gone sleeping"));
    Assert ();
  } 
  
  DebugOwTemp ("ROM =");
  for (i = 0; i < 8; i++) {
    DebugOwTemp (" ");
    DebugOwTemp (SensorAddress [i], HEX);
  }
  
  DebugOwTempln ();
  if (OneWire::crc8 (SensorAddress, 7) != SensorAddress [7]) {
    DebugOwTempln(F("CRC is not valid!"));
    Assert ();
  }
  
  // the first ROM byte indicates which chip
  switch (SensorAddress [0]) {
    case 0x10:
      DebugOwTempln (F("  Chip = DS18S20"));
      SensorType = 1;
      Flags = (Flags & ~F_SENSOR_MASK) | F_SENSOR_DS1820;
      break;
    case 0x28:
      DebugOwTempln (F("  Chip = DS18B20"));
      SensorType = 0;
      Flags = (Flags & ~F_SENSOR_MASK) | F_SENSOR_DS1820;
      break;
    case 0x22:
      DebugOwTempln (F("  Chip = DS1822"));
      SensorType = 0;
      Flags = (Flags & ~F_SENSOR_MASK) | F_SENSOR_DS1822;
      break;
    default:
      DebugOwTempln (F("Device is not a DS18x20 family device."));
      Assert ();
  } 
}

//*******************************************************************************
//*                                 RF24 support                                *
//*******************************************************************************

#if DEBUG_RF24
  #define DebugRf24(...) {Serial.print(__VA_ARGS__); delay (50);}
  #define DebugRf24ln(...) {Serial.println(__VA_ARGS__); delay (50);}
#else
  #define DebugRf24(...)
  #define DebugRf24ln(...)
#endif 

#include "EepromEditor.h"
  
//*******************************************************************************
//*                            Arduino setup method                             *
//******************************************************************************* 

void setup () {
  byte i;
  
  Serial.begin (57600);
  Serial.println (F("[RF24Node]"));

  //
  // Read EEPROM to get the node address, ID and other stuff
  //
  
  if (EepromRead ()) {
    Debug (F("EEPROM read OK, address "));
    Debugln (Eeprom.NodeAddress);
  } else {
    Debugln (F("EEPROM read failed, default address 05 assigned"));
  }    

  //
  // Sensor init, printf and VT100 init
  //
  
  DS1820Init ();
  PrintfInit ();
  Vt100Init (); 
  
  //
  // RF24Network init
  //
  
  SPI.begin ();
  Radio.begin ();
  Network.begin (RF24_CHANNEL, Eeprom.NodeAddress);

  //
  // Prepare battery measurement feature
  //
  
  analogReference (INTERNAL);
  pinMode (PIN_BATTERY_ANALOG, INPUT);
  pinMode (PIN_BATTERY_MEASURE, OUTPUT);
  digitalWrite (PIN_BATTERY_MEASURE, LOW);
  delay (1000);
  BattLevelAverage = analogRead (PIN_BATTERY_ANALOG);
  BattLevelAverage = analogRead (PIN_BATTERY_ANALOG);
  if (IsChar ()) {
    EepromEditor ();
  }
}

//*******************************************************************************
//*                              Main program loop                              *
//******************************************************************************* 

void loop () {
  uint16_t Temperature; 
  bool Ok;
  uint32_t tmp;
  uint16_t BattLevel;
  uint8_t TempCounter;
  static uint8_t IdCounter = 0;
  
  //
  // Enable the pin as output - that grounds the voltage divider
  //

  pinMode (PIN_BATTERY_MEASURE, OUTPUT);
  
  //
  // Check the network regularly - in fact we don't expect any incoming packets, but just in case...
  //
  
  Network.update ();                 
  
  //
  // Measure the battery level. Use running average of last 16 values
  //
  
  BattLevelAverage = ((BattLevelAverage * 15) + analogRead (PIN_BATTERY_ANALOG)) >> 4;
  pinMode (PIN_BATTERY_MEASURE, INPUT);
  Debug ("Battery level average ");
  Debugln (BattLevelAverage);
  
  //
  // We'll be sending the battery voltage in tens of mV. 
  // To calculate the voltage, first measure the resistors, one is around 1MOhm, the other around 0.33MOhm.
  // Calculate constant K, where K = R2 / (R1 + R2). K should be around 4.
  // In other words, Vraw = K * Va, where Va is the voltage measured on pin A0.
  // Then, calculate the value 100 * Vr * K, where
  //  - 100 means we want tens of milivolts, and there's 100 tens of milivolts in one volt
  //  - Vr is Arduino reference voltage, typically 1.1V
  //  - K is constant above
  // Resulting number will be something around 450. 
  // To get the RAW voltage, multiply the reading from the ADC by that number, and divide by 1024 (resolution of ADC)
  // I additionally deduct 200 as I report voltage over 2V to make the result fit to one byte (2V - 4.55V)
  //
  
  tmp = ((uint32_t) BattLevelAverage * Eeprom.BatteryCoefficient + 512) >> 10;
  if (tmp < 200) {
    BattLevel = 0;
    Eeprom.Flags != F_BATTERY_DEAD;
  } else if (tmp > (200 + 255)) {
    BattLevel = 255;
  } else {
    BattLevel = (uint8_t) (tmp - 200);
  }
  
  Debug ("Battery level ");
  Debugln (BattLevel);
    
  //
  // Now measure the temperature
  //

  Temperature = SensorRead ();
  Debug ("Temperature ");
  Debugln (Temperature);
    
  PayloadTemperature.BattLevel = BattLevel;
  PayloadTemperature.Temperature[0] = Temperature;
  PayloadTemperature.Temperature[1] = 0xFFFF;
  
  RF24NetworkHeader Header (0, RF24_TYPE_TEMP);   
  
  Ok = Network.write (Header, &PayloadTemperature, sizeof(PayloadTemperature));
  if (Ok) {
    DebugRf24ln (F("Temperature sending ok."));
  } else {
    DebugRf24ln (F("Temperature sending failed."));
  }
  
  //
  // Send sensor ID when it's time to do - that is every TIME_COUNTER_TEMP * TIME_COUNTER_ID * 8 seconds
  //

  if (IdCounter == 0) {
    IdCounter = TIME_COUNTER_ID;
    
    PayloadId.BattLevel = BattLevel;
    memcpy (PayloadId.Id, Eeprom.Id, 8);
    PayloadId.Version = VERSION;
    PayloadId.Flags = Eeprom.Flags | BATTERY_TYPE;
    
    RF24NetworkHeader HeaderId (0, RF24_TYPE_ID);   
    
    Ok = Network.write (HeaderId, &PayloadId, sizeof(PayloadId));
    if (Ok) {
      DebugRf24ln(F("ID sending ok."));
    } else {
      DebugRf24ln(F("ID sending failed."));
    }
  } else {
    IdCounter--;
  }

  //
  // Put the node to sleep for TIME_COUNTER_TEMP * 8 seconds
  //
  
  for (TempCounter = 0; TempCounter < TIME_COUNTER_TEMP; TempCounter++) {
    LowPower.powerDown (SLEEP_8S, ADC_OFF, BOD_OFF);
  }
}

