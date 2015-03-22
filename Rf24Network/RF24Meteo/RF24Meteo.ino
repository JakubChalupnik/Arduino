//*******************************************************************************
//*
//* Arduino based low power meteo unit using nRF24L01+, DHT22 and BMP180
//* Using Adafruit libraries for sensors, based on Adafruit sample code
//* RF24Radio library by TMRh20: http://tmrh20.github.io/RF24/
//*
//*******************************************************************************
//* Processor:  Arduino Pro Mini 3.3V/8MHz
//* Author      Date       Comment
//*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//* Kubik       5.3.2015 First version, just basic code for HW tests
//*******************************************************************************

//*******************************************************************************
//*                            HW details                                       *
//*******************************************************************************
// Standard NRF24L01+ module (eBay)
// One BMP180 barometric pressure sensor
// One DHT22 humidity / temperature sensor
//
// Used pins:
//  NRF24L01+       SPI + 9, 10
//  BMP180  - I2C
//  DHT22   - pin 3 
//  Input voltage - A0
//  Ground of the voltage divider - pin 15

//*******************************************************************************
//*                           Includes and defines                              *
//******************************************************************************* 

#define VERSION 0
#define DEBUG_RF24 0
#define DEBUG 0

#include <RF24.h>
#include <SPI.h>
#include <LowPower.h>
#include <Rf24PacketDefine.h>
#include <EEPROM.h>
#include "DHT.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>

#define PIN_DHT 3                                 // Pin used to communicate with DTHxx
#define DHTTYPE DHT22                             // DHT 22  (AM2302)

#define PIN_BATTERY_ANALOG A0                     // Analog pin used to measure the battery voltage 
#define PIN_BATTERY_MEASURE 15                    // Pull this pin low to measure the bat. voltage (ground of the resistor divider)
#define DEFAULT_BATTERY_K 480L                    // Default battery conversion constant (adjust according to divider used)
#define DEFAULT_BATTERY_TYPE F_BATTERY_NONE       // Type of the battery this node uses - NONE means net powered 
#define DEFAULT_TIME_COUNTER_MEASURE 8
#define F_DEFAULTS DEFAULT_BATTERY_TYPE
#define DEFAULT_SENSOR_ID 'Mx'                    // Default sensor type is Meteo X (to remind me it needs setting up)

//*******************************************************************************
//*                           Data types                                        *
//******************************************************************************* 

typedef struct {
  uint32_t Crc;
  uint16_t Header;
  uint16_t SensorId;
  uint16_t BatteryCoefficient;
  uint8_t MeasurementPeriod;
  uint8_t Flags;
} Eeprom_t;

//*******************************************************************************
//*                               Static variables                              *
//*******************************************************************************
 
//
// RF24 variables
//

RF24 Rf24Radio (9, 10);
SensorPayload_t Payload;

//
// Sensor variables
// DHT uses third parameter. This needs to be set to 3 for 8MHz processor
//

DHT Dht (PIN_DHT, DHTTYPE, 3);
Adafruit_BMP085_Unified Bmp = Adafruit_BMP085_Unified (10085);

//
// EEPROM variables
//

Eeprom_t Eeprom;

//
// Other variables
//

uint16_t BattLevelAverage;
uint16_t Flags = DEFAULT_BATTERY_TYPE;

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
  delay (10);
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
    Eeprom.SensorId = DEFAULT_SENSOR_ID;
    Eeprom.BatteryCoefficient = DEFAULT_BATTERY_K;
    Eeprom.MeasurementPeriod = DEFAULT_TIME_COUNTER_MEASURE;
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
//*                          BMP085 / BMP180 support                            *
//******************************************************************************* 
uint16_t GetBarometricPressure (void) {
  sensors_event_t event;

  Bmp.getEvent (&event);
  return event.pressure; 
}

#if DEBUG
void displaySensorDetails(void) {
  sensor_t sensor;
  Bmp.getSensor (&sensor);
  Serial.println ("------------------------------------");
  Serial.print   ("Sensor:       "); Serial.println (sensor.name);
  Serial.print   ("Driver Ver:   "); Serial.println (sensor.version);
  Serial.print   ("Unique ID:    "); Serial.println (sensor.sensor_id);
  Serial.print   ("Max Value:    "); Serial.print (sensor.max_value); Serial.println (" hPa");
  Serial.print   ("Min Value:    "); Serial.print (sensor.min_value); Serial.println (" hPa");
  Serial.print   ("Resolution:   "); Serial.print (sensor.resolution); Serial.println (" hPa");  
  Serial.println ("------------------------------------");
  Serial.println ("");
  delay(500);
}
#endif // DEBUG

//*******************************************************************************
//*                            Arduino setup method                             *
//******************************************************************************* 

void setup(void) {

  Serial.begin (57600);
  Serial.println (F("[RF24Meteo]"));

  //
  // Read EEPROM to get the node address, ID and other stuff
  //
  
  if (EepromRead ()) {
    Debug (F("EEPROM read OK, ID "));
    Debug ((char) (Eeprom.SensorId >> 8));
    Debugln ((char) (Eeprom.SensorId & 0xFF));
  } else {
    Debugln (F("EEPROM read failed, default address Mx assigned"));
  }    

  //
  // Sensor init, printf and VT100 init
  //

  PrintfInit ();
  Vt100Init (); 
  Dht.begin ();
  if (!Bmp.begin()) {
    Serial.println ("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

#if DEBUG
  displaySensorDetails();
#endif // DEBUG

  //
  // Setup and configure rf radio
  // Open sensor pipe to master node, set 250kbps and the right packet length
  //

  Rf24Radio.begin();
  Rf24Radio.setPayloadSize (sizeof (SensorPayloadMeteo_t));
  Rf24Radio.setAutoAck (false);
  Rf24Radio.setPALevel (RF24_PA_HIGH);
  Rf24Radio.setDataRate (RF24_250KBPS);
  Rf24Radio.setChannel (RF24_RADIO_CHANNEL);
  Rf24Radio.openWritingPipe (RF24_SENSOR_PIPE);

#if DEBUG_RF24
  Rf24Radio.printDetails();
#endif

  Rf24Radio.powerDown ();

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

  //
  // If a key is pressed, go to EEPROM editor first
  //
  
  if (IsChar ()) {
    EepromEditor ();
  }

  Payload.PacketType = RF24_SENSOR_TYPE_METEO;
  Payload.SensorId = Eeprom.SensorId;
}

//*******************************************************************************
//*                              Main program loop                              *
//******************************************************************************* 

void loop (void) {
  bool Ok;
  uint32_t tmp;
  uint16_t BattLevel;
  uint8_t MeasureCounter;
  float Humidity;
  float Temperature;
  uint16_t Pressure;
  
  //
  // If measuring the battery voltage is supported (coef > 0), do the following:
  // Enable the pin as output - that grounds the voltage divider
  // Measure the battery level. Use running average of last 16 values
  // Disable the pin again to save some power
  //

  if (Eeprom.BatteryCoefficient > 0) {
    pinMode (PIN_BATTERY_MEASURE, OUTPUT);
    BattLevel = analogRead (PIN_BATTERY_ANALOG);
    BattLevelAverage = ((BattLevelAverage * 15) + BattLevel) >> 4;
//BUGBUG    pinMode (PIN_BATTERY_MEASURE, INPUT);
    Debug ("Battery level ");
    Debug (BattLevel);
    Debug (", average ");
    Debugln (BattLevelAverage);

    tmp = ((uint32_t) BattLevel * Eeprom.BatteryCoefficient + 512) >> 10;
    Debug ("Voltage ");
    Debugln (tmp);
  
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
      Eeprom.Flags |= F_BATTERY_DEAD;
    } else if (tmp > (200 + 255)) {
      BattLevel = 255;
    } else {
      BattLevel = (uint8_t) (tmp - 200);
    }

  } else {      // Voltage measurement not available
    BattLevel = 0;
    Eeprom.Flags |= F_NO_VOLTAGE;
  }
  
  //
  // Now measure the temperature
  //

  Humidity = Dht.readHumidity ();
  Temperature = Dht.readTemperature ();

  Debug ("DHT22 temperature ");
  Debug (Temperature);
  Debug (" Humidity ");
  Debug (Humidity);
  Debugln ("%");
  
  Pressure = GetBarometricPressure ();
  Debug ("Pressure ");
  Debug (Pressure);
  Debugln ("hPA");
   
  Payload.FlagsS = 0;  // BUGBUG
  Payload.BattLevel = BattLevel;
  Payload.Temperature = ((int) (Temperature + 0.05)) * 10;
  Payload.Humidity = (uint8_t) (Humidity + 0.5);
  Payload.Pressure = Pressure;
  
  Rf24Radio.powerUp ();
  Ok = Rf24Radio.write ((uint8_t *) &Payload, sizeof (Payload));
  Rf24Radio.powerDown ();
  if (Ok) {
    DebugRf24ln (F("Payload sent."));
  } else {
    DebugRf24ln (F("Payload sending failed."));
  }

  //
  // Put the node to sleep for TemperaturePeriod * 8 seconds
  //
  
  for (MeasureCounter = 0; MeasureCounter < Eeprom.MeasurementPeriod; MeasureCounter++) {
    LowPower.powerDown (SLEEP_8S, ADC_OFF, BOD_OFF);
  }
}
