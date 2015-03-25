//*******************************************************************************
//*
//* Arduino based GM counter using SBM-20 tube
//* Based on example sketches for Adafruit_GFX by Adafruit
//* Some ideas taken from the https://sites.google.com/site/diygeigercounter/home
//*
//*******************************************************************************
//* Processor:  Arduino Pro Mini 3.3V/ 16MHz
//* Author      Date       Comment
//*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//* Kubik       16.2.2015 First version, just basic code for HW tests
//*******************************************************************************

//*******************************************************************************
//*                            HW details                                       *
//*******************************************************************************
//
// Used pins:
//  Nokia display:
//   pin 3 - Serial clock out (SCLK)
//   pin 4 - Serial data out (DIN)
//   pin 5 - Data/Command select (D/C)
//   pin 6 - LCD chip select (CS)
//   pin 7 - LCD reset (RST)
//
//  Input from the GM counter - pin 2
//

//*******************************************************************************
//*                           Includes and defines                              *
//******************************************************************************* 

#define VERSION 0 
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>

#define COUNTER_INPUT 2
#define COUNTER_IRQ 0

//*******************************************************************************
//*                               Static variables                              *
//******************************************************************************* 

Adafruit_PCD8544 display = Adafruit_PCD8544 (3, 4, 5, 6, 7);
volatile uint32_t ClickCounter = 0;

//*******************************************************************************
//*                               Interrupt counter                             *
//******************************************************************************* 

//
// The function just increases the click counter. 
// It's called everytime the GM tube produces the impulse.
//

void ClickIrq () {
  ClickCounter++;
}

//*******************************************************************************
//*                            Graphic support                                  *
//*******************************************************************************  

void BarGraph (byte X, byte Height, byte value) {
	
	drawRect (X, 0, 84, Height, 1);
	fillRect (X, 0, (value * 84) / 255, Height, 1);
}

//*******************************************************************************
//*                            Arduino setup method                             *
//*******************************************************************************  

void setup (void) {

  //
  // Intitialize serial port, identify yourself
  //

  Serial.begin (57600);
  Serial.println ("[GmCounter]");

  //
  // Enable click pin pull up, attach interrupt to count the clicks
  //
  
  pinMode (COUNTER_INPUT, INPUT_PULLUP);
  attachInterrupt(COUNTER_IRQ, ClickIrq, RISING);

  //
  // Initialize display
  //
  
  display.begin ();
  display.setContrast (50);
  display.display (); 
  delay (500);
  display.clearDisplay ();

  display.setTextSize (2);
  display.setTextColor (BLACK);
  display.setCursor (30, 0);
  display.print ("GM");
  display.setCursor (0, 16);
  display.print ("counter");
  display.setTextSize (1);
  display.setCursor (0, 36);
  display.print ("(c) 2015 Kubik");
  display.display ();
  delay (1000);
}

//*******************************************************************************
//*                              Main program loop                              *
//*******************************************************************************  

void loop (void) {
  uint32_t Millis = millis ();
  static uint32_t CpsTime = 0, CpsCount = 0, CpsAverage = 0;
  static uint32_t Cp5sTime = 0, Cp5sCount = 0, Cp5sAverage = 0;
  
  if (Cp5sTime == 0) {
    
    //
    // Start measuring Cp5s. Measure it every 5s and use running average of 30s.
    //
  
    Cp5sTime = Millis;
    Cp5sCount = ClickCounter;
  } else if ((Cp5sTime + 5000) < Millis) {

    //
    // 5 seconds passed, show the Cp5s and reset Cp5sTime to start again
    //
    
//    Cp5sAverage = ClickCounter - Cp5sCount;
    Cp5sAverage = ((Cp5sAverage * 5) + (ClickCounter - Cp5sCount) * 12) / 6;
    Cp5sTime = 0;
  }
  
  if (CpsTime == 0) {
    
    //
    // Start measuring CPS
    //
  
    CpsTime = Millis;
    CpsCount = ClickCounter;
  } else if ((CpsTime + 1000) >= Millis) {
    
    //
    // counting, no need to do anything
    //
    
  } else {
    
    //
    // One second passed, show the CPS and reset CpsTime to start again
    //
    
    CpsCount = ClickCounter - CpsCount;
    CpsAverage = ((CpsAverage * 3) + CpsCount) >> 2;
    display.clearDisplay ();
    display.setCursor (0, 0);
    display.print ("CPS ");
    display.print (CpsCount);
    display.print (" / ");
    display.println (CpsAverage);

    display.print ("CPM ");
    display.println (Cp5sAverage);

#define CPS_PER_MR_H 29
    display.print ("Dose ");
    display.print (Cp5sAverage * 1000UL / (CPS_PER_MR_H * 60));
    display.println ("uR/h");

    display.print ("Counter ");
    display.println (ClickCounter);
    display.display();
    CpsTime = 0;
  }
}

