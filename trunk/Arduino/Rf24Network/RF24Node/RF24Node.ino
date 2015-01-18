//*******************************************************************************
//*
//* Arduino based RF24Network sensor node
//* Based on example sketches for RF24Network by J. Coliz <maniacbug@ymail.com> / TMRh20
//*
//*******************************************************************************
//* Processor:  Arduino Pro Mini 3.3V/ 8 or 16MHz
//* Author      Date       Comment
//*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//* Kubik       14.1.2013 First version, just basic code for HW tests
//*******************************************************************************

//*******************************************************************************
//*                            HW details                                       *
//*******************************************************************************
// Standard NRF24L01+ module (eBay)
//
// Used pins:
//  NRF24L01+       SPI + 9, 10

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
#define THIS_NODE_DEFAULT_ID "TempNode"   // Default node ID, used when EEPROM isn't valid

#define F_DEFAULTS 0x8000

typedef struct {
  uint32_t Crc;
  uint16_t Header;
  char Id[8];
  uint16_t NodeAddress;
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
#define SENSOR_POWER_PIN 7

//
// EEPROM variables
//

Eeprom_t EepromContent;

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
// Read EEPROM from address 0 to EepromContent. 
// Check the CRC, compare it with stored CRC.
// If it does not match, load EepromContent with defaults.
//

bool EepromRead (void) {
  byte i;
  byte *EepromPtr;
  uint32_t Crc;

  EepromPtr = (byte *) &EepromContent;
  for (i = 0; i < sizeof (Eeprom_t); i++) {
    *EepromPtr++ = EEPROM.read (i);
  }
  
  EepromPtr = (byte *) &EepromContent;
  Crc = CrcBuffer (EepromPtr + 4, sizeof (Eeprom_t) - 4);    // Do not include stored CRC into CRC calculation
  if (Crc != EepromContent.Crc) {
    EepromContent.Flags = F_DEFAULTS;
    EepromContent.Header = 'ID';
    EepromContent.NodeAddress = THIS_NODE_DEFAULT_ADDRESS;
    memcpy (EepromContent.Id, THIS_NODE_DEFAULT_ID, sizeof (EepromContent.Id));
    return false;
  }
  return true;
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

int SensorRead (void) {
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

  return (Raw + 8) >> 4; 
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
  
//*******************************************************************************
//*                            Arduino setup method                             *
//******************************************************************************* 

void setup () {
  byte i;
  
#if DEBUG || DEBUG_OW_TEMP || DEBUG_RF24
  Serial.begin (57600);
  Serial.println (F("[RF24Node]"));
#endif

  //
  // OneWire init, search for sensor
  //
  
  pinMode (SENSOR_POWER_PIN, OUTPUT);
  digitalWrite (SENSOR_POWER_PIN, HIGH);
  delay (1000);
  
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
      break;
    case 0x28:
      DebugOwTempln (F("  Chip = DS18B20"));
      SensorType = 0;
      break;
    case 0x22:
      DebugOwTempln (F("  Chip = DS1822"));
      SensorType = 0;
      break;
    default:
      DebugOwTempln (F("Device is not a DS18x20 family device."));
      Assert ();
  } 

  //
  // Read EEPROM to get the node address, ID and other stuff
  //
  
  if (EepromRead ()) {
    Debug (F("EEPROM read OK, address "));
    Debugln (EepromContent.NodeAddress);
  } else {
    Debugln (F("EEPROM read failed, default address 05 assigned"));
  }    

  //
  // RF24Network init
  //
  
  SPI.begin ();
  Radio.begin ();
  Network.begin (RF24_CHANNEL, EepromContent.NodeAddress);
}

//*******************************************************************************
//*                              Main program loop                              *
//******************************************************************************* 

void loop () {
  int Temperature; 
  bool Ok;

  //
  // Things that have to be done every loop pass
  //
  
  Network.update ();                 // Check the network regularly
  
  Temperature = SensorRead ();
  PayloadTemperature.BattLevel = 0xFF;          // No batt level measurement
  PayloadTemperature.Temperature[0] = (int8_t) Temperature;
  PayloadTemperature.Temperature[1] = 0xFF;
  PayloadTemperature.Temperature[2] = 0xFF;
  PayloadTemperature.Temperature[3] = 0xFF;

  Network.update ();                 // Check the network regularly
  RF24NetworkHeader Header (0, RF24_TYPE_TEMP);   
  
  Network.update ();                 // Check the network regularly
  Ok = Network.write (Header, &PayloadTemperature, sizeof(PayloadTemperature));
  if (Ok) {
    DebugRf24ln (F("Temperature sending ok."));
  } else {
    DebugRf24ln (F("Temperature sending failed."));
  }
  
  LowPower.powerDown (SLEEP_8S, ADC_OFF, BOD_OFF);     

  //
  // Send sensor ID
  //

  Network.update ();                 // Check the network regularly

  PayloadId.BattLevel = 0xFF;          // No batt level measurement
  memcpy (PayloadId.Id, EepromContent.Id, 8);
  PayloadId.Version = VERSION;
  PayloadId.Flags = EepromContent.Flags;
  
  Network.update ();                 // Check the network regularly
  RF24NetworkHeader HeaderId (0, RF24_TYPE_ID);   
  
  Network.update ();                 // Check the network regularly
  Ok = Network.write (HeaderId, &PayloadId, sizeof(PayloadId));
  if (Ok) {
    DebugRf24ln(F("ID sending ok."));
  } else {
    DebugRf24ln(F("ID sending failed."));
  }
  
  LowPower.powerDown (SLEEP_8S, ADC_OFF, BOD_OFF);     


}

