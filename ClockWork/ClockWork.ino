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
//#include <Bounce2.h>

#define DELAY_DEBOUNCE 100
#define DELAY_REPEAT_START 400
#define DELAY_REPEAT 250

#define SWITCH_SET 0x01
#define SWITCH_INC 0x02

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

//
// Switches on the back of the clock, used to set time
//

const byte SwitchLeft = 5;
const byte SwitchRight = 7;

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
    State = S_IDLE;
    break;
  }
}

//*******************************************************************************
//*                            Input switches support                           *
//*******************************************************************************

byte InputSwitches (void) {
  byte SwitchesState = 0;
  
  if (!digitalRead (SwitchLeft)) {
    SwitchesState |= SWITCH_SET;
  }

  if (!digitalRead (SwitchRight)) {
    SwitchesState |= SWITCH_INC;
  }
  
  return SwitchesState;
}

//*******************************************************************************
//*                            Arduino setup method                             *
//*******************************************************************************

void setup() {                

  Serial.begin (115200);
  Serial.println ("[ClockWork]");

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

  //
  // Initialize input switches
  //
  
  pinMode (SwitchLeft, INPUT);
  digitalWrite (SwitchLeft, HIGH);          // Activate internal pull-up
  
  pinMode (SwitchRight, INPUT);
  digitalWrite (SwitchRight, HIGH);          // Activate internal pull-up
}

//*******************************************************************************
//*                              Main program loop                              *
//*******************************************************************************

void loop () {
  time_t Time;
  time_t TimeInc;
  static time_t PreviousTime = 0;
  unsigned long Millis;
  static unsigned long LastMillis = 0;
  
  static byte id = 0x00;
  static word key_counter = 0;
  byte c;
  byte key = 0;

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

  c = InputSwitches ();
  if(c == 0) {                    // No key pressed
    id = 0;
    key_counter = 0;
  } else if(c != id) {            // New key differs from the previous one
    id = c;
    key_counter = 0;
  } else {                        // New key is the same as previous one
    key_counter++;
  }
  
  if(key_counter == DELAY_DEBOUNCE) {     // Debouncing complete - set key pressed
    key = id;
//    Serial.write (key + '0');
  } else if(key_counter == DELAY_REPEAT_START) {  // Repeated key
    key = id;
    key_counter -= DELAY_REPEAT;
//    Serial.write (key + '0');
  }

  if (key & SWITCH_SET) {
    key = 0;
    TimeInc = 3600;
  } else if (key & SWITCH_INC) {
    key = 0;
    TimeInc = 60;
  } else {
    TimeInc = 0;
  }
  
  if (TimeInc != 0) {
    Time = RTC.get ();
    Time += TimeInc;
    RTC.set (Time);
    setTime (Time);
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


