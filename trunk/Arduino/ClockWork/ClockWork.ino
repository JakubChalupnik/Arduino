//*******************************************************************************
//*
//* Arduino based 7segment clock, "work" version
//*
//*******************************************************************************
//* Processor:  Arduino Pro Mini
//*
//* Author      Date       Comment
//*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//* Kubik       10.8.2013 First release, testing the HW
//* Kubik       10.8.2013 Added time and display support
//* Kubik       11.8.2013 Temperature support added
//*******************************************************************************

//*******************************************************************************
//*                            HW details                                       *
//*******************************************************************************
// Segment display is driven by seven MBI5170 chips linked into a chain. 
// All chips share clock, latch and output enable.
//

//*******************************************************************************
//*                           Includes and defines                              *
//*******************************************************************************

#include <Time.h>  
#include <Wire.h>  
#include <DS1307RTC.h>
#include <OneWire.h>

//
// Pins used to drive the segment chain.
//

const int ShiftPWM_latchPin= 17;
const int ShiftPWM_dataPin = 11;
const int ShiftPWM_clockPin = 13;
const int ShiftPWM_oePin = 16;

const bool ShiftPWM_invertOutputs = false;
const bool ShiftPWM_balanceLoad = true;

#include <ShiftPWM.h>   // include ShiftPWM.h after setting the pins!

//
// Here you set the number of brightness levels, the update frequency and the number of shift registers.
//

#define ShiftPwmMaxBrightness     63
#define ShiftPwmFrequency         75
#define ShiftPwmRegisters         7

//*******************************************************************************
//*                               Static variables                              *
//*******************************************************************************

OneWire Ds (2);
byte TempSensorAddr [8];
byte TempSensorType;
byte TempSensorData [12];
byte TempSensorPresent;
int Temperature;

//*******************************************************************************
//*                               Segment functions                             *
//*******************************************************************************

extern unsigned char SegHexTable [];

void SegmentSetDigit (byte Digit, byte Brightness, byte Offset) {
  byte i;
  
  for (i = 0; i < 8; i++) {
    if (Digit & (1 << (7 - i))) {
      ShiftPWM.SetOne (Offset, Brightness);
    } else {
      ShiftPWM.SetOne (Offset, 0);
    }
    Offset++;
  }
} 

#define SEGMENT_BLANK   SegHexTable [0x10]

void SegmentSetDisplay (byte Hour, byte Minute, int Temperature, byte Brightness) {
  byte Digit;
  
  //
  // First digit of hour could be empty for hour < 10
  //
  
  if (Hour >= 10) {
    Digit = SegHexTable [Hour / 10];
  } else {
    Digit = SEGMENT_BLANK;
  }
  
  SegmentSetDigit (Digit, Brightness, 24);
  
  //
  // Second digit of hour
  //
  
  Digit = SegHexTable [Hour % 10];
  SegmentSetDigit (Digit, Brightness, 16);
  
  //
  // First digit of minute
  //
  
  Digit = SegHexTable [Minute / 10];
  SegmentSetDigit (Digit, Brightness, 8);
  
  //
  // Second digit of minute
  //
  
  Digit = SegHexTable [Minute % 10];
  SegmentSetDigit (Digit, Brightness, 0);

  //
  // First digit of temperature. 
  // Process negative temperature correctly, that is display the positive value and enable the dash
  //

  if (Temperature < 0) {
    Temperature = -Temperature;
    Digit = SegHexTable [Temperature / 10] | 0x01;
  } else {
    Digit = SegHexTable [Temperature / 10];
  }

  SegmentSetDigit (Digit, Brightness / 2, 48);

  //
  // Second digit of temperature
  //
  
  Digit = SegHexTable [Temperature % 10];
  SegmentSetDigit (Digit, Brightness / 2, 40);

  //
  // Degree of Celsius sign
  //
  
  SegmentSetDigit (0xE0, Brightness / 2, 32);
}
  
//*******************************************************************************
//*                               DS1820 support                                *
//*******************************************************************************

typedef enum {
  S_IDLE, S_WAIT, S_READ, S_COMP
} state_t;

void Ds1307Init (void) {
  byte i;
  
  if (!Ds.search (TempSensorAddr)) {
    Serial.println ("No more addresses.");
    Ds.reset_search ();
    TempSensorPresent = 0;
    return;
  } else {
    TempSensorPresent = 1;
    Serial.print("ROM =");
    for (i = 0; i < 8; i++) {
      Serial.write (' ');
      Serial.print (TempSensorAddr [i], HEX);
    }
    Serial.println ();
  }

  if (OneWire::crc8(TempSensorAddr, 7) != TempSensorAddr [7]) {
    Serial.println ("CRC is not valid!");
    return;
  }
 
  // the first ROM byte indicates which chip
  switch (TempSensorAddr [0]) {
    case 0x10:
      Serial.println("  Chip = DS18S20");  // or old DS1820
      TempSensorType = 1;
      break;
    case 0x28:
      Serial.println("  Chip = DS18B20");
      TempSensorType = 0;
      break;
    case 0x22:
      Serial.println("  Chip = DS1822");
      TempSensorType = 0;
      break;
    default:
      Serial.println("Device is not a DS18x20 family device.");
      break;
  } 
}  

//
// Poll the temperature. This should be called repeatedly, but the timing is not very critical.
// Updates the global variable Temperature
//

void TempSensorPoll (void) {
  byte i;
  byte present = 0;
  unsigned int raw;
  signed int sraw;
  static state_t State = S_IDLE;
  static uint32_t WaitTime;

  switch (State) {
  case S_IDLE:
    Ds.reset ();
    Ds.select (TempSensorAddr);
    Ds.write (0x44, 1);         // start conversion, with parasite power on at the end
    WaitTime = millis () + 1000;
    State = S_WAIT;
    break;

  case S_WAIT:
    if (millis () > WaitTime) {
      State = S_READ;
    }
    break;

  case S_READ:
    present = Ds.reset();
    Ds.select (TempSensorAddr);    
    Ds.write (0xBE);         // Read Scratchpad
    State = S_COMP;
    break;

  case S_COMP:  
    for (i = 0; i < 9; i++) {           // we need 9 bytes
      TempSensorData [i] = Ds.read();
    }

    // convert the data to actual temperature

    raw = (TempSensorData [1] << 8) | TempSensorData [0];
    if (TempSensorType) {
      raw = raw << 3; // 9 bit resolution default
      if (TempSensorData [7] == 0x10) {
        // count remain gives full 12 bit resolution
        raw = (raw & 0xFFF0) + 12 - TempSensorData [6];
      }
    } else {
      byte cfg = (TempSensorData [4] & 0x60);
      if (cfg == 0x00) raw = raw << 3;  // 9 bit resolution, 93.75 ms
      else if (cfg == 0x20) raw = raw << 2; // 10 bit res, 187.5 ms
      else if (cfg == 0x40) raw = raw << 1; // 11 bit res, 375 ms
      // default is 12 bit resolution, 750 ms conversion time
    }

    Temperature = (raw + 8) >> 4;
    
//    sraw = raw;
//      if (sraw < 0) {
//        Serial.print("-");
//        sraw = -raw;
//      }
//    
//    if (1) {
//      Serial.print("Temp ");
//      Serial.print(raw, HEX);
//      Serial.print(" ");
//      sraw = raw;
//      if (sraw < 0) {
//        Serial.print("-");
//        sraw = -raw;
//      }
//  
//      Serial.print(sraw / 16);
//      Serial.print(".");
//      Serial.print(sraw & 0x000F);
//      Serial.print("     ");
//      Serial.println((sraw + 8) / 16);
//    }
//    
//    Temperature = raw;
    State = S_IDLE;
    break;
  }
}



//const int ShiftPWM_latchPin= 17;
//const int ShiftPWM_dataPin = 11;
//const int ShiftPWM_clockPin = 13;
//const int ShiftPWM_oePin = 16;



#define CLK 13
#define SDI 11
#define OE  16
#define LE  17

#define MatrixClkHigh()        { digitalWrite (CLK, HIGH); }
#define MatrixClkLow()         { digitalWrite (CLK, LOW); }
#define MatrixClkPulse()       { digitalWrite (CLK, HIGH); digitalWrite (CLK, LOW); }
#define MatrixStrobeHigh()     { digitalWrite (LE, HIGH); }
#define MatrixStrobeLow()      { digitalWrite (LE, LOW); }
#define MatrixStrobePulse()    { digitalWrite (LE, HIGH); digitalWrite (LE, LOW); }
#define MatrixDataHigh()       { digitalWrite (SDI, HIGH); }
#define MatrixDataLow()        { digitalWrite (SDI, LOW); }
#define MatrixOeHigh()         { digitalWrite (OE, HIGH); }
#define MatrixOeLow()          { digitalWrite (OE, LOW); } 

//
// Send one byte to the chain of shift registers. No latching at all.
//

void ShiftSendByte (byte b) {
  byte i;
  
  for (i = 0; i < 8; i++) {
    if (b & 0x01) {
      MatrixDataHigh ();
    } else {
      MatrixDataLow ();
    }
    MatrixClkPulse ();
    b = b >> 1;
  }
} 

//
// Send the current adjust code to MBI5170 chips
//

void  MatrixCurrentAdjust (byte CurrentAdjustCode) {
  byte i;
  
  //
  // Prepare CLK low
  //
  
  MatrixClkLow ();
  
  //
  // Follow the sequence in MBI5170 datasheet to enable CA mode
  // Step 1
  //
  
  MatrixStrobeLow ();
  MatrixOeHigh ();
  MatrixClkPulse ();
  
  //
  // Step 2
  //
  
  MatrixStrobeLow ();
  MatrixOeLow ();
  MatrixClkPulse ();

  //
  // Step 3
  //
  
  MatrixStrobeLow ();
  MatrixOeHigh ();
  MatrixClkPulse ();
  
  //
  // Step 4
  //
  
  MatrixStrobeHigh ();
  MatrixOeHigh ();
  MatrixClkPulse ();
  
  //
  // Step 5
  //
  
  MatrixStrobeLow ();
  MatrixOeHigh ();
  MatrixClkPulse ();
  
  //
  // Send CurrentAdjustCode to all four MBI5170 chips
  //

//  for (i = 0; i < 7; i++) {
//    ShiftSendByte (CurrentAdjustCode);  
//  }
  
  ShiftSendByte (0xFF);  
  ShiftSendByte (0xFF);  
  ShiftSendByte (0xFF);  
  ShiftSendByte (0xFF);  
//  ShiftSendByte (0x00);  
//  ShiftSendByte (0x00);  
//  ShiftSendByte (0x00);  
//  ShiftSendByte (0x00);  
  ShiftSendByte (0x00);  
  ShiftSendByte (0x00);  
  ShiftSendByte (0x00);  
  
  MatrixStrobePulse ();
  
  //
  // Switch back to normal mode 
  // Step 1
  //
  
  MatrixStrobeLow ();
  MatrixOeHigh ();
  MatrixClkPulse ();
  
  //
  // Step 2
  //
  
  MatrixStrobeLow ();
  MatrixOeLow ();
  MatrixClkPulse ();
  
  //
  // Step 3, 4, 5
  //
  
  MatrixStrobeLow ();
  MatrixOeHigh ();
  MatrixClkPulse ();
  MatrixClkPulse ();
  MatrixClkPulse ();
  
  //
  // Enable OE again
  //

  MatrixOeLow ();
}   


//*******************************************************************************
//*                            Arduino setup method                             *
//*******************************************************************************

void setup() {                
  Serial.begin(115200);

  //
  // Initialize the pins used for commmunicating with segment drivers.
  // Most of them are initialized in ShiftPWM already
  //
  
  digitalWrite (ShiftPWM_oePin, LOW);
  pinMode (ShiftPWM_oePin, OUTPUT);

  ShiftPWM.SetAmountOfRegisters (ShiftPwmRegisters);

  ShiftPWM.Start (ShiftPwmFrequency, ShiftPwmMaxBrightness);

  ShiftPWM.SetAll (0);

  setSyncProvider (RTC.get);

  Ds1307Init ();

//  MatrixCurrentAdjust (0x0);  // Just for test 
}

//*******************************************************************************
//*                              Main program loop                              *
//*******************************************************************************

void loop () {
  time_t Time;
  static time_t PreviousTime = 0;
  unsigned long Millis;
  static unsigned long LastMillis = 0;

  //
  // Any code that needs to be executed on every loop should go here
  //

  TempSensorPoll ();
  
  //
  // Every time the Millis changes, execute the code below.
  // In other words, all the code that follows is executed every millisecond or less frequently
  //

  Millis = millis();
  if (Millis != LastMillis) {
    LastMillis = Millis;
  } else {
    return;
  }

  //
  // Every time the time changes, display the new time on the screen
  // We're only displaying hours and minutes, but the code is called for every 
  //

  Time = now ();
  if (Time != PreviousTime) {
    PreviousTime = Time;
    
    SegmentSetDisplay (hour (), minute (), Temperature, 63);
    if (second () & 0x01) {
      ShiftPWM.SetOne (15, 63);
    } else {
      ShiftPWM.SetOne (15, 0);
    }
  }
}


