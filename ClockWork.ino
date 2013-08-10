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

#include "TimerOne.h"

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

//*******************************************************************************
//*                               Static variables                              *
//*******************************************************************************

//
// Here you set the number of brightness levels, the update frequency and the number of shift registers.
//

#define ShiftPwmMaxBrightness     127
#define ShiftPwmFrequency         75
#define ShiftPwmRegisters         7

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
}

//*******************************************************************************
//*                              Main program loop                              *
//*******************************************************************************

void loop () {
  ShiftPWM.OneByOneFast();
}


