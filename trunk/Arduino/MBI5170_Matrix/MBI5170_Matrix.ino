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

//
// Pins used to drive the matrix. Note that when you change these, you have to 
// change the next paragraph too - direct HW access is used for time critical 
// pin handling. The HW mapping differs between different flavours of Arduino!
//

#define CLK 10    // PB4
#define SDI  9    // PH6
#define OE  12    // PB6
#define LE  11    // PB5

#define MatrixClkPulse()       { PORTB |= (1 << 4); PORTB &= ~(1 << 4); }
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

//*******************************************************************************
//*                               Static variables                              *
//*******************************************************************************

byte MatrixBuffer [2][SX * SY];
volatile byte Flags;

#define FLAGS_FADING            0x08    // Do not use - interrupt routine uses this
#define FLAGS_FADE_PAGE         0x04    // Set for fading between screens (e.g. "flags |= FLAGS_FADE_PAGE;" )
#define FLAGS_FLIP_PAGE         0x02    // Set for flipping screens
#define FLAGS_PAGE_DISPLAYED    0x01    // Do not use - interrupt routine uses that
                                        // to keep track about what screen buffer is displayed
#define ScreenPageDisplayed()   (Flags & FLAGS_PAGE_DISPLAYED)
#define ScreenPageInactive()    (~Flags & FLAGS_PAGE_DISPLAYED)

//
////*******************************************************************************
////*                              Fonts                                          *
////*******************************************************************************
//
////
//// Fonts by holger.klabunde@t-online.de - I found them in a package named
//// "T6963 based LCD(8x8).zip"
//// The fonts are in program memory space so they have to be accessed in a special way!
////
//
////const byte font_8x6[96 * 8] PROGMEM = {
////#include "FN6X8_reduced.h"
//const byte font_8x6[32 * 8] PROGMEM = {
//#include "FN6X8_reduced2.h"
//};
//
//#define FontByte(Index) pgm_read_byte (((PGM_P) font_8x6) + Index)
//

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
// TODO - this shall be done in display write routines, not in interrupt when sending the byte out
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

//*******************************************************************************
//*                              Screen routines                                *
//*******************************************************************************


//*******************************************************************************
//*                              Interrupt handler                              *
//*******************************************************************************
  
void MatrixInterrupt (void) {
  static byte Row = 0;

  MatrixSend (&(MatrixBuffer [0][Row * SX]), Row);
  Row = (Row + 1) & 0x07;
}
    
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

  //
  // Initialize timer and attach the matrix interrupt to it.
  // The interrupt will be called every 1ms
  // 4ms already causes flickering.
  //
  
  Timer1.initialize (1000);
  Timer1.attachInterrupt (MatrixInterrupt);
  
}

//*******************************************************************************
//*                              Main program loop                              *
//*******************************************************************************

void loop() {
  byte i;
  
  for (i = 0; i < SX * SY; i++) {
    MatrixBuffer [0][i] = 0xAA;
  }
  
  delay (100);
  
  for (i = 0; i < SX * SY; i++) {
    MatrixBuffer [0][i] = 0x55;
  }
  
  delay (100);
}
