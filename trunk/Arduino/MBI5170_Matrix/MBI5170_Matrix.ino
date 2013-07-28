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

//*******************************************************************************
//*                               Static variables                              *
//*******************************************************************************

unsigned long MatrixBuffer [8] = {
  0xAA000000L,
  0x55000000L,
  0xAA000000L,
  0x55000000L,
  0xAA000000L,
  0x55000000L,
  0xAA000000L,
  0x55000000L
};

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
// <Column> goes to MBI5170 chips, <Row> is converted from 0..7 to bitmask and goes to HC595
//

void MatrixSend (unsigned long Columns, byte Row) {
  ShiftSendByte (~(1 << (Row & 0x07)));
  ShiftSendByte (MbiConvertColumns ((Columns >> 24) & 0xFF));
  ShiftSendByte (MbiConvertColumns ((Columns >> 16) & 0xFF));
  ShiftSendByte (MbiConvertColumns ((Columns >> 8)  & 0xFF));
  ShiftSendByte (MbiConvertColumns ((Columns >> 0)  & 0xFF));
  MatrixStrobePulse ();
}

//*******************************************************************************
//*                              Interrupt handler                              *
//*******************************************************************************
  
void MatrixInterrupt (void) {
  static byte Row = 0;

  MatrixSend (MatrixBuffer [Row], Row);
  Row = (Row + 1) & 0x07;
}
    
//*******************************************************************************
//*                              Interrupt handler                              *
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
}
