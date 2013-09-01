//*******************************************************************************
//*
//* Arduino based VFD display
//*
//*******************************************************************************
//* Processor:  Arduino Uno
//* Author      Date       Comment
//*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//* Kubik        1.9.2013 First release, testing the HW
//*******************************************************************************

//*******************************************************************************
//*                            HW details                                       *
//*******************************************************************************
// Futaba FG165 K6 VFD display (16 digits, each consists of 14 segments, and comma+dot)
// 2 pieces of Maxim MAX69xx VFD drivers (20 outputs each) in serie
// CLK = 5, DIN = 7, LOAD = 4, BLANK = 6
//
// Note - code coming from another clock project, some remnants kept for later use
//

//*******************************************************************************
//*                           Includes and defines                              *
//******************************************************************************* 

#include <Time.h>
#include <JeeLib.h>
#include <PacketDefine.h>

//
// Pins (will be converted to direct access)
// 

const byte PinBlank = 6;
const byte PinClk = 5;
const byte PinDin = 7;
const byte PinLoad = 4;

//
// Digits and segments. 
// The whole display is controlled via one dword, each controlling one output.
// Due to HW design, some outputs activate segments, some activate grids.
// To light specific digit's segment, activate the DIGIT_x and SEGMENT_x outputs at the same time.
// The whole display is of course multiplexed.
//

#define DIGIT_1    0x40000000L
#define DIGIT_2    0x10000000L
#define DIGIT_3    0x04000000L
#define DIGIT_4    0x01000000L
#define DIGIT_5    0x00400000L
#define DIGIT_6    0x00080000L
#define DIGIT_7    0x00020000L
#define DIGIT_8    0x00008000L
#define DIGIT_9    0x00002000L
#define DIGIT_10   0x00000800L
#define DIGIT_11   0x00000200L
#define DIGIT_12   0x00000040L
#define DIGIT_13   0x00000010L
#define DIGIT_14   0x00000004L
#define DIGIT_15   0x00000001L
#define DIGIT_16   0x00000080L
#define SEGMENT_A  0x80000000L
#define SEGMENT_K  0x00040000L
#define SEGMENT_I  0x00000020L
#define SEGMENT_L  0x00100000L
#define SEGMENT_G1 0x08000000L
#define SEGMENT_G2 0x00000400L
#define SEGMENT_F  0x20000000L
#define SEGMENT_E  0x02000000L
#define SEGMENT_D  0x00010000L
#define SEGMENT_C  0x00001000L
#define SEGMENT_B  0x00000002L
#define SEGMENT_J  0x00000008L
#define SEGMENT_H  0x00000100L
#define SEGMENT_M  0x00800000L
#define SEGMENT_cm 0x00200000L
#define SEGMENT_dt 0x00004000L

#define SEGMENT_G (SEGMENT_G1 | SEGMENT_G2)

//*******************************************************************************
//*                   Static variables and PROGMEM constants                    *
//******************************************************************************* 

unsigned long int Screen [16];    // Contains the values consisting of segment bits and one digit bit

const unsigned long int PROGMEM SegmentTable [] = {
  0L,
  SEGMENT_A | SEGMENT_B | SEGMENT_C | SEGMENT_E | SEGMENT_F | SEGMENT_G,     // A
  SEGMENT_A | SEGMENT_B | SEGMENT_C | SEGMENT_D | SEGMENT_I | SEGMENT_L | SEGMENT_G2,     // B
  SEGMENT_A | SEGMENT_D | SEGMENT_E | SEGMENT_F,                       // C
  SEGMENT_A | SEGMENT_B | SEGMENT_C | SEGMENT_D | SEGMENT_I | SEGMENT_L,           // D
  SEGMENT_A | SEGMENT_D | SEGMENT_E | SEGMENT_F | SEGMENT_G1,           // E
  SEGMENT_A | SEGMENT_E | SEGMENT_F | SEGMENT_G1,                 // F
  SEGMENT_A | SEGMENT_C | SEGMENT_D | SEGMENT_E | SEGMENT_F | SEGMENT_G2,           // G
  SEGMENT_B | SEGMENT_C | SEGMENT_E | SEGMENT_F | SEGMENT_G,           // H
  SEGMENT_I | SEGMENT_L,                                   // I
  SEGMENT_B | SEGMENT_C | SEGMENT_D | SEGMENT_E,                       // J
  SEGMENT_E | SEGMENT_F | SEGMENT_G1 | SEGMENT_K | SEGMENT_J,                 // K
  SEGMENT_D | SEGMENT_E | SEGMENT_F,                             // L
  SEGMENT_B | SEGMENT_C | SEGMENT_E | SEGMENT_F | SEGMENT_H | SEGMENT_J,           // M
  SEGMENT_B | SEGMENT_C | SEGMENT_E | SEGMENT_F | SEGMENT_H | SEGMENT_K,           // N
  SEGMENT_A | SEGMENT_B | SEGMENT_C | SEGMENT_D | SEGMENT_E | SEGMENT_F,           // O
  SEGMENT_A | SEGMENT_B | SEGMENT_E | SEGMENT_F | SEGMENT_G,           // P
  SEGMENT_A | SEGMENT_B | SEGMENT_C | SEGMENT_D | SEGMENT_E | SEGMENT_F | SEGMENT_K,     // Q
  SEGMENT_A | SEGMENT_B | SEGMENT_E | SEGMENT_F | SEGMENT_G | SEGMENT_K,     // R
  SEGMENT_A | SEGMENT_C | SEGMENT_D | SEGMENT_F | SEGMENT_G,     // S
  SEGMENT_A | SEGMENT_I | SEGMENT_L,                             // T
  SEGMENT_B | SEGMENT_C | SEGMENT_D | SEGMENT_E | SEGMENT_F,                 // U
  SEGMENT_E | SEGMENT_F | SEGMENT_J | SEGMENT_M,                       // V
  SEGMENT_B | SEGMENT_C | SEGMENT_E | SEGMENT_F | SEGMENT_K | SEGMENT_M,           // W
  SEGMENT_H | SEGMENT_J | SEGMENT_M | SEGMENT_K,                       // X
  SEGMENT_H | SEGMENT_J | SEGMENT_L,                             // Y
  SEGMENT_A | SEGMENT_D | SEGMENT_J | SEGMENT_D,                       // Z
  // --- numerals
  SEGMENT_A | SEGMENT_B | SEGMENT_C | SEGMENT_D | SEGMENT_E | SEGMENT_F | SEGMENT_J | SEGMENT_M,
  SEGMENT_J | SEGMENT_B | SEGMENT_C,
  SEGMENT_A | SEGMENT_B | SEGMENT_D | SEGMENT_E | SEGMENT_G,
  SEGMENT_A | SEGMENT_B | SEGMENT_C | SEGMENT_D | SEGMENT_G,
  SEGMENT_B | SEGMENT_C | SEGMENT_F | SEGMENT_G,
  SEGMENT_A | SEGMENT_C | SEGMENT_D | SEGMENT_F | SEGMENT_G,
  SEGMENT_A | SEGMENT_C | SEGMENT_D | SEGMENT_E | SEGMENT_F | SEGMENT_G,
  SEGMENT_A | SEGMENT_B | SEGMENT_C,
  SEGMENT_A | SEGMENT_B | SEGMENT_C | SEGMENT_D | SEGMENT_E | SEGMENT_F | SEGMENT_G,
  SEGMENT_A | SEGMENT_B | SEGMENT_C | SEGMENT_D | SEGMENT_F | SEGMENT_G,
  // --- symbols
  SEGMENT_G, 
};

const unsigned long int PROGMEM DigitTable[] = {
  DIGIT_1,
  DIGIT_2,
  DIGIT_3,
  DIGIT_4,
  DIGIT_5,
  DIGIT_6,
  DIGIT_7,
  DIGIT_8,
  DIGIT_9,
  DIGIT_10,
  DIGIT_11,
  DIGIT_12,
  DIGIT_13,
  DIGIT_14,
  DIGIT_15,
  DIGIT_16
};

//*******************************************************************************
//*                            VFD functions                                    *
//******************************************************************************* 

//
// VfdSend sends the data to the VFD drivers. 
// This is the lowest level interface to VFD, just setting the corresponding anodes or grids to Vbb or Gnd
//

void VfdSend (unsigned long int DataIn) {
  int i;

  digitalWrite (PinLoad, LOW);
  digitalWrite (PinClk, LOW);

  for (i = 0; i < 32; i++) {
    if (DataIn & 0x00000001) {
      digitalWrite (PinDin, HIGH);
    } else {
      digitalWrite (PinDin, LOW);
    }
    digitalWrite (PinClk, HIGH);
    digitalWrite (PinClk, LOW);
    DataIn >>= 1;
  }

  digitalWrite (PinLoad, HIGH);
  digitalWrite (PinLoad, LOW);
}

//
// VfdRefresh is called every millisecond and displays just one digit. Being called repeatedly takes care about multiplexing.
//

void VfdRefresh (void) {
  static byte Digit = 0;
  
  VfdSend (Screen [Digit]);
  Digit = (Digit + 1) & 0x0F;
}   

//
// VfdCharConvert takes ASCII character and converts it into internal representation (index into SegmentTable)
//

byte VfdCharConvert (char c) {
  
  if ((c >= 'a') && (c <= 'z')) {  
    return c + 1 - 'a';
  }
  
  if ((c >= 'A') && (c <= 'Z')) {
    return c + 1 - 'A';
  }

  if ((c >= '0') && (c <= '9')) {
    return c + 27 - '0';
  }
  
  if (c == ':') {
    return 37;
  }
  
  return 0;
}

//
// VfdDisplay displays (puts into Screen buffer) the string, up to 16 characters
//

void VfdDisplay (char *String) {
  byte i = 0;

  memset (Screen, 0, sizeof (Screen));  
  while ((i < 16) && (*String != '\0')) {
    Screen [i] = pgm_read_dword_near (SegmentTable + VfdCharConvert (*String));
    Screen [i] |= pgm_read_dword_near (DigitTable + i);
    i++;
    String++;
  }
}

//
// VfdDot displays dot after specified digit
//

void VfdDot (byte d) {
  Screen [d] |= SEGMENT_dt;
}

//
// VfdComma displays comma after specified digit
//

void VfdComma (byte d) {
  Screen [d] |= SEGMENT_cm;
}

//*******************************************************************************
//*                            Arduino setup method                             *
//******************************************************************************* 

void setup() {

  TCCR1B = TCCR1B & 0b11111000 | 0x01; // Change the PWM frequency of pins 9 and 10 to ~31kHz - see http://playground.arduino.cc/Main/TimerPWMCheatsheet
  pinMode(PinClk, OUTPUT);
  pinMode(PinLoad, OUTPUT);
  pinMode(PinDin, OUTPUT);

  Serial.begin(57600);
  Serial.println("\n[VfdDisplay]\n");

  pinMode(PinBlank, OUTPUT);
  digitalWrite (PinBlank, LOW);

  setTime (0, 0, 0, 1, 1, 2013);
  
//  VfdDisplay ("ABCDEFGHIJKLMNOP");  
}

//*******************************************************************************
//*                              Main program loop                              *
//*******************************************************************************  

void loop() {
  unsigned long Millis;
  static unsigned long LastMillis = 0;
  time_t Time; 
  static time_t PreviousTime = 0; 
  static char str [16];
  
  //
  // Skip the rest if the mills () did not change.
  //
  
  Millis = millis();
  if (Millis == LastMillis) {
    return;
  }
  LastMillis = Millis;

  VfdRefresh ();
  
  //
  // Skip the rest if the time did not change - in other words, execute every second only.
  //

  Time = now ();
  if (Time == PreviousTime) {
    return;
  }
  PreviousTime = Time;   

  sprintf (str, "%2d:%2.2d:%2.2d %2d%-2d   ", hour (), minute (), second (), day (), month ());
  VfdDisplay (str);  
  VfdDot (10);
}
