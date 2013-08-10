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

#include "Time.h"

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

#define ShiftPwmMaxBrightness     127
#define ShiftPwmFrequency         75
#define ShiftPwmRegisters         7

//*******************************************************************************
//*                               Static variables                              *
//*******************************************************************************

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

  SegmentSetDigit (Digit, Brightness, 48);

  //
  // Second digit of temperature
  //
  
  Digit = SegHexTable [Temperature % 10];
  SegmentSetDigit (Digit, Brightness, 40);

  //
  // Degree of Celsius sign
  //
  
  SegmentSetDigit (0xE0, Brightness, 32);
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

  setTime (0, 0, 0, 1, 1, 2013);    // Dummy time until the time gets synced 
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
    
    SegmentSetDisplay (hour (), minute (), -32, 63);
  }
}


