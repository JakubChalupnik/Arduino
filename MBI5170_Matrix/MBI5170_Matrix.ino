//*******************************************************************************
//*
//* Arduino based Mina Clock
//*
//*******************************************************************************
//* Processor:  Arduino Mega2560
//*
//* Author      Date       Comment
//*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//* Kubik       28.7.2013 First release, just testing the HW
//* Kubik       30.7.2013 Added screen code from 3matrix clock
//* Kubik        4.8.2013 New font, time support, MBI5170 current adjust
//*******************************************************************************

//*******************************************************************************
//*                            HW details                                       *
//*******************************************************************************
// Matrix display is driven by four MBI5170 chips linked into a chain. 
// Next in the chain is HC595 driving PNP transistors selecting rows.
// All chips share clock, latch and output enable.
//

//*******************************************************************************
//*                           Includes and defines                              *
//*******************************************************************************

#include "TimerOne.h"
#include <Time.h>

//
// Pins used to drive the matrix. Note that when you change these, you have to 
// change the next paragraph too - direct HW access is used for time critical 
// pin handling. The HW mapping differs between different flavours of Arduino!
//

#define CLK 10    // PB4
#define SDI  9    // PH6
#define OE  12    // PB6
#define LE  11    // PB5
#define LED 13

#define MatrixClkHigh()        { PORTB |= (1 << 4); }
#define MatrixClkLow()         { PORTB &= ~(1 << 4); }
#define MatrixClkPulse()       { PORTB |= (1 << 4); PORTB &= ~(1 << 4); }
#define MatrixStrobeHigh()     { PORTB |= (1 << 5); }
#define MatrixStrobeLow()      { PORTB &= ~(1 << 5); }
#define MatrixStrobePulse()    { PORTB |= (1 << 5); PORTB &= ~(1 << 5); }
#define MatrixDataHigh()       { PORTH |= (1 << 6); }
#define MatrixDataLow()        { PORTH &= ~(1 << 6); }
#define MatrixOeHigh()         { PORTB |= (1 << 6); }
#define MatrixOeLow()          { PORTB &= ~(1 << 6); }

//
// How columns of the LED matrix correspond to bits in the byte/dword stored in screen buffer
// e.g. column 4 corresponds to bit 7. 
//

#define C_MASK_4 0x80  
#define C_MASK_2 0x40  
#define C_MASK_1 0x20  
#define C_MASK_7 0x10  
#define C_MASK_6 0x08  
#define C_MASK_0 0x04  
#define C_MASK_5 0x02  
#define C_MASK_3 0x01  

//
// Screen related defines
//

#define SX 4      // Amount of 8x8 matrixes
#define SY 8      // Total number of rows
#define CONFIG_FADE_DELAY 6

//*******************************************************************************
//*                               Static variables                              *
//*******************************************************************************

byte Screen [2][SX * SY];
volatile byte Flags;

#define FLAGS_FADING            0x08    // Do not use - interrupt routine uses this
#define FLAGS_FADE_PAGE         0x04    // Set for fading between screens (e.g. "flags |= FLAGS_FADE_PAGE;" )
#define FLAGS_FLIP_PAGE         0x02    // Set for flipping screens
#define FLAGS_PAGE_DISPLAYED    0x01    // Do not use - interrupt routine uses that
                                        // to keep track about what screen buffer is displayed
#define ScreenPageDisplayed()   (Flags & FLAGS_PAGE_DISPLAYED)
#define ScreenPageInactive()    (~Flags & FLAGS_PAGE_DISPLAYED)


//*******************************************************************************
//*                              Matrix specific code                           *
//*******************************************************************************

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
// Convert columns from internal representation to how HW is wired
//

byte MbiConvertColumns (byte c) {
  byte Columns = 0;
  
  if (c & 0x80) Columns |= C_MASK_0;
  if (c & 0x40) Columns |= C_MASK_1;
  if (c & 0x20) Columns |= C_MASK_2;
  if (c & 0x10) Columns |= C_MASK_3;
  if (c & 0x08) Columns |= C_MASK_4;
  if (c & 0x04) Columns |= C_MASK_5;
  if (c & 0x02) Columns |= C_MASK_6;
  if (c & 0x01) Columns |= C_MASK_7;
  
  return Columns;
}

//
// Send the data to matrix shift registers.
// <Columns> go to MBI5170 chips, <Row> is converted from 0..7 to bitmask and goes to HC595
//

void MatrixSend (byte *Columns, byte Row) {
  byte i;
  
  ShiftSendByte (~(1 << (Row & 0x07)));
  
  for (i = 0; i < SX; i++) {
    ShiftSendByte (MbiConvertColumns (*Columns++));
  }
  MatrixStrobePulse ();
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

  for (i = 0; i < 4; i++) {
    ShiftSendByte (CurrentAdjustCode);  
  }
  
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
//*                              Interrupt handler                              *
//*******************************************************************************

//
// Interrupt handler is now part of the MatrixCode file
//
    
//*******************************************************************************
//*                            Arduino setup method                             *
//*******************************************************************************

void setup() {                
  
  //
  // Initialize the pins used for commmunicating with the matrix
  // We're using Arduino methods here, this is not time critical
  //
  
  digitalWrite (CLK, LOW);
  digitalWrite (LE, LOW);
  digitalWrite (SDI, LOW);
  digitalWrite (OE, HIGH);
  
  pinMode (CLK, OUTPUT);
  pinMode (SDI, OUTPUT);
  pinMode (OE, OUTPUT);
  pinMode (LE, OUTPUT);

  digitalWrite (OE, LOW);

  memset (Screen, 0, sizeof (Screen));

  MatrixCurrentAdjust (0x0);  // Just for test
  
  //
  // Initialize timer and attach the matrix interrupt to it.
  // The interrupt will be called every 1ms
  // 4ms already causes flickering.
  //
  
  Timer1.initialize (1000);
  Timer1.attachInterrupt (MatrixInterrupt);
  
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
    
    digitalWrite (LED, second () & 0x01 ? HIGH : LOW);

    //
    // PutChar writes to inactive screen page. Clear it first, fill in the data and flip pages
    //
    
    memset (Screen [ScreenPageInactive()], 0, sizeof (Screen [0]));
    if (hour () > 9) {
      PutChar (0, 0, hour () / 10);
    }
    PutChar (7, 0, (hour () % 10) + '0');
    PutChar (15, 0, ':');
    PutChar (19, 0, minute () / 10 + '0');
    PutChar (26, 0, (minute () % 10) + '0');
    Flags |= FLAGS_FADE_PAGE;
  }
}
