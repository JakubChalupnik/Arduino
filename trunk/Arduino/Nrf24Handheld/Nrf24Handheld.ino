//*******************************************************************************
//*
//* Arduino based handheld with Nokia screen and NRF24L01+ unit
//* Based on example sketch for Adafruit Monochrome Nokia 5110 LCD Displays
//* Based on "scanner" from Rf24 library by J. Coliz <maniacbug@ymail.com>
//*
//*******************************************************************************
//* Processor:  Arduino Pro Mini 3.3V/16MHz
//* Author      Date       Comment
//*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//* Kubik       28.11.2013 First version, just basic code for HW tests
//*******************************************************************************

//*******************************************************************************
//*                            HW details                                       *
//*******************************************************************************
// Nokia LCD (eBay, Adafruit library)
// Note that the Adafruit library was patched to rotate the display, the setRotation sadly does not work.
// Standard NRF24L01+ module (eBay)
//
// Used pins:
//  Nokia LCD	    2, 3, 4, 5, 6 + ??? (b/l)
//  NRF24L01+       SPI + 9, 10

//*******************************************************************************
//*                           Includes and defines                              *
//******************************************************************************* 

#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

//*******************************************************************************
//*                               Static variables                              *
//*******************************************************************************
 
//
// Nokia display variables
// pin 2 - Serial clock out (SCLK)
// pin 3 - Serial data out (DIN)
// pin 4 - Data/Command select (D/C)
// pin 5 - LCD chip select (CS)
// pin 6 - LCD reset (RST)
//

Adafruit_PCD8544 display = Adafruit_PCD8544(2, 3, 4, 5, 6);

//
// RF24 variables
//

RF24 radio(9,10);    // Set up nRF24L01 radio on SPI bus plus pins 9 & 10

//
// Channel info
//

const uint8_t num_channels = 84;
uint8_t values[num_channels];
uint8_t max_values[num_channels];

//*******************************************************************************
//*                            Arduino setup method                             *
//******************************************************************************* 

void setup () {

  Serial.begin (57600);
  Serial.println (F("[Nrf24Handheld]"));

  //
  // Setup LCD, print some basic info
  //
  
  display.begin ();
  display.setContrast (55);
  display.clearDisplay ();   // clears the screen and buffer
  display.setTextSize (1);
  display.setTextColor (BLACK);
  display.setCursor (0, 0);
  display.print (F("Nrf24 Handheld"));
  display.setCursor (0, 10);
  display.print (F("(c) 2013 Kubik"));
  display.display ();

  //
  // Setup and configure rf radio
  //

  radio.begin ();
  radio.setAutoAck (false);
  radio.startListening ();    // Get into standby mode
  radio.stopListening ();

  delay (1000);
  display.clearDisplay ();

  display.setTextSize (1);
  display.setTextColor (BLACK);

  memset (max_values, 0, sizeof (max_values));
}

//*******************************************************************************
//*                              Main program loop                              *
//******************************************************************************* 

void loop () {
  const int num_reps = 100;
  byte x;

  display.clearDisplay ();
  memset (values, 0, sizeof (values));

  //
  // Scan all channels num_reps times
  //
  
  int rep_counter = num_reps;
  while (rep_counter--) {
    int i = num_channels;
    while (i--) {
      radio.setChannel (i);        // Select this channel
      radio.startListening ();     // Listen on the channel for a while 
      delayMicroseconds (128);
      radio.stopListening ();

      // Did we get a carrier?
      if (radio.testCarrier ()) {
        ++values[i];
      }
    }
  }

  //
  // Display channel measurements, clamped to a single hex digit
  //
  
  for (x = 0; x < 84; x++) {
    if (values[x] > max_values[x]) {
      max_values[x] = values[x];
    }
    
    display.drawLine (x, 36, x, 36 - min (36, max_values[x] * 3), BLACK);
  }
  
  for (x = 0; x < 9; x++) {
    display.setCursor (x * 10, 40);
    display.write (x + '0');
    display.drawLine (x * 10, 37, x * 10, 38, BLACK);
  }
  
  display.display ();
}

