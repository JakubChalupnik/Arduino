//*******************************************************************************
//*
//* Arduino based Mina Clock
//*
//*******************************************************************************
//* Processor:  Arduino Pro Mini
//*
//* Author      Date       Comment
//*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//* Kubik       28.7.2013 First release, just testing the HW
//* Kubik       30.7.2013 Added screen code from 3matrix clock
//* Kubik        4.8.2013 New font, time support, MBI5170 current adjust
//* Kubik       17.8.2013 MBI5170 current adjust removed, change to Arduino Pro Mini
//* Kubik       17.8.2013 Display turned upside down, code modified to support that
//* Kubik       18.8.2013 Segment display support added
//* Kubik       18.8.2013 RFM12 support, packet decoding
//* Kubik       25.8.2013 Light intensity support added
//*******************************************************************************

//*******************************************************************************
//*                            HW details                                       *
//*******************************************************************************
// Matrix display is driven by four MBI5170 chips linked into a chain. 
// Next in the chain is HC595 driving PNP transistors selecting rows.
// All chips share clock, latch and output enable.
// Light sensor is on pin A0
//

//*******************************************************************************
//*                           Includes and defines                              *
//*******************************************************************************

#include "TimerOne.h"
#include <Time.h>
#include <JeeLib.h>
#include <PacketDefine.h> 

#include "LedSeg.h"

#define DEBUG_RMF12 0

//
// Pins used to drive the matrix. Note that when you change these, you have to 
// change the next paragraph too - direct HW access is used for time critical 
// pin handling. The HW mapping differs between different flavours of Arduino!
//

#define CLK  8    // PB0
#define SDI  9    // PB1
#define OE   6    // PD6
#define LE   7    // PD7
#define LES  5    // PD5
#define LED 13
#define LIGHT_SENSOR A0

#define MatrixClkHigh()        { PORTB |= (1 << 0); }
#define MatrixClkLow()         { PORTB &= ~(1 << 0); }
#define MatrixClkPulse()       { PORTB |= (1 << 0); PORTB &= ~(1 << 0); }
#define MatrixStrobeHigh()     { PORTD |= (1 << 7); }
#define MatrixStrobeLow()      { PORTD &= ~(1 << 7); }
#define MatrixStrobePulse()    { PORTD |= (1 << 7); PORTD &= ~(1 << 7); }
#define MatrixDataHigh()       { PORTB |= (1 << 1); }
#define MatrixDataLow()        { PORTB &= ~(1 << 1); }
#define MatrixOeHigh()         { PORTD |= (1 << 6); }
#define MatrixOeLow()          { PORTD &= ~(1 << 6); }
#define SegmentStrobePulse()   { PORTD |= (1 << 5); PORTD &= ~(1 << 5); }

//
// How columns of the LED matrix correspond to bits in the byte/dword stored in screen buffer
// e.g. column 4 corresponds to bit 7. 
//

#define C_MASK_7 0x04  
#define C_MASK_6 0x20  
#define C_MASK_5 0x40  
#define C_MASK_4 0x01  
#define C_MASK_3 0x80  
#define C_MASK_2 0x02
#define C_MASK_1 0x08  
#define C_MASK_0 0x10  

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
byte Segments [2][6];
volatile byte Flags;

#define FLAGS_FADING            0x08    // Do not use - interrupt routine uses this
#define FLAGS_FADE_PAGE         0x04    // Set for fading between screens (e.g. "flags |= FLAGS_FADE_PAGE;" )
#define FLAGS_FLIP_PAGE         0x02    // Set for flipping screens
#define FLAGS_PAGE_DISPLAYED    0x01    // Do not use - interrupt routine uses that
                                        // to keep track about what screen buffer is displayed
#define ScreenPageDisplayed()   (Flags & FLAGS_PAGE_DISPLAYED)
#define ScreenPageInactive()    (~Flags & FLAGS_PAGE_DISPLAYED)

int InnerTemperature;
int OuterTemperature; 
byte Brightness = 0;      // Display brightness. 0 means fully on, the number can go pretty high for lower brightness

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
  
  ShiftSendByte (~(1 << ((7 - Row) & 0x07)));
  
  Columns = Columns + 3;
  for (i = 0; i < SX; i++) {
    ShiftSendByte (MbiConvertColumns (*Columns--));
  }
  MatrixStrobePulse ();
}

//
// Send the data to segment shift register.
// <Segments> goes to MBI5170 chips, <Digit> is converted from 0..5 to bitmask and goes to HC595
//

void SegmentSend (byte Segments, byte Digit) {
  
  if (Digit > 4) {
    ShiftSendByte (0xFF);
    ShiftSendByte (0);
    SegmentStrobePulse ();
  } else {
    ShiftSendByte (~(1 << (Digit)));
    ShiftSendByte (Segments);
    SegmentStrobePulse ();
  }
}

//*******************************************************************************
//*                              Interrupt handler                              *
//*******************************************************************************

//
// Interrupt handler is now part of the MatrixCode file
//

//*******************************************************************************
//*                            RFM12 packet functions                           *
//*******************************************************************************

//----------------------------------------------------------------------------
//
// PacketDecode processes the packet received from RFM12 and updates the time and external temperature accordingly
//

void PacketDecode () {
  char c[4];
  TimePayload_t *Time;
  TemperaturePayload_t *Temp;

  switch (*rf12_data) {
    
    case RF12_PACKET_TIME:
      Time = (TimePayload_t *) rf12_data;
      setTime (Time->hour, Time->minute, 0, Time->day, Time->month, 2000 + Time->year);

#if DEBUG_RMF12
      Serial.print (F("Time packet: "));
      Serial.print (Time->day);
      Serial.print ('.');
      Serial.print (Time->month);
      Serial.print (' ');
      Serial.print (2000 + Time->year);
      Serial.print (' ');
      Serial.print (Time->hour);
      Serial.print (':');
      Serial.println (Time->minute);
#endif // DEBUG_RMF12

      break;

    case RF12_PACKET_TEMPERATURE:
      Temp = (TemperaturePayload_t *) rf12_data;
      InnerTemperature = Temp->temp2;
      OuterTemperature = Temp->temp1;

#if DEBUG_RMF12
      Serial.print (F("Temperature packet: "));
      Serial.print (InnerTemperature);
      Serial.print (' ');
      Serial.println (OuterTemperature);
#endif //DEBUG_RMF12

      break;

    default:

#if DEBUG_RMF12
      uint8_t Rf12Len;
      volatile uint8_t *Rf12Data;
      Rf12Len = rf12_len;
      Rf12Data = rf12_data;
      Serial.print (F("Unknown packet: "));
      while (Rf12Len > 0) {
        sprintf (c, "%02X ", *Rf12Data);
        Serial.print (c);
        Rf12Len--;
        Rf12Data++;
      }
      Serial.println ();
#endif // DEBUG_RMF12

      break;
  }
}
     
//*******************************************************************************
//*                            Arduino setup method                             *
//*******************************************************************************

void setup() {                
  
  Serial.begin(57600);
  Serial.println("\n[Mina Clock]\n");

  //
  // Initialize the pins used for commmunicating with the matrix
  // We're using Arduino methods here, this is not time critical
  //
  
  digitalWrite (CLK, LOW);
  digitalWrite (LE, LOW);
  digitalWrite (LES, LOW);
  digitalWrite (SDI, LOW);
  digitalWrite (OE, HIGH);
  
  pinMode (CLK, OUTPUT);
  pinMode (SDI, OUTPUT);
  pinMode (OE, OUTPUT);
  pinMode (LE, OUTPUT);
  pinMode (LES, OUTPUT);

  digitalWrite (OE, LOW);

  memset (Screen, 0, sizeof (Screen));
  memset (Segments, 0, sizeof (Segments));

  rf12_initialize(RF12_NET_NODE, RF12_868MHZ, RF12_NET_GROUP); 

  //
  // Initialize timer and attach the matrix interrupt to it.
  // The interrupt will be called every 1ms
  // 4ms already causes flickering.
  //
  
  Timer1.initialize (100);
  Timer1.attachInterrupt (MatrixInterrupt);
  
  setTime (0, 0, 0, 1, 1, 2013);    // Dummy time until the time gets synced
}

//*******************************************************************************
//*                              Main program loop                              *
//*******************************************************************************

unsigned int LightIntensity = 0;

void loop () {
  time_t Time;
  static time_t PreviousTime = 0;
  unsigned long Millis;
  static unsigned long LastMillis = 0;

  //
  // Any code that needs to be executed on every loop should go here
  //
  
  //
  // Check if any valid packet was received, and decode it
  //
  
  if (rf12_recvDone() && rf12_crc == 0) {
    PacketDecode ();
  }
 
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
  
  LightIntensity = (LightIntensity * 31 + analogRead (LIGHT_SENSOR)) / 32;
  if (LightIntensity > 950) {          // Really dark
    Brightness = 200;
  } else if (LightIntensity > 700) {   // Ambient light
    Brightness = 20;
  } else if (LightIntensity > 100) {   // Decent light
    Brightness = 10;
  } else {
    Brightness = 0;
  }

  //
  // Every time the time changes, display the new time on the screen
  // We're only displaying hours and minutes, but the code is called for every 
  //

  Time = now ();
  if (Time != PreviousTime) {
    PreviousTime = Time;
    
    digitalWrite (LED, second () & 0x01 ? HIGH : LOW);
    
//    Serial.print (LightIntensity);
//    Serial.print (" : ");
//    Serial.print (Brightness);
//    Serial.println ();

    //
    // PutChar writes to inactive screen page. Clear it first, fill in the data, update segment display and flip pages
    //
    
    memset (Screen [ScreenPageInactive()], 0, sizeof (Screen [0]));

    // 
    // Display time on the matrix display
    //
    
    if (hour () > 9) {
      PutChar (0, 0, (hour () / 10) + '0');
    }
    PutChar (7, 0, (hour () % 10) + '0');
    PutChar (15, 0, ':');
    PutChar (19, 0, minute () / 10 + '0');
    PutChar (26, 0, (minute () % 10) + '0');
    
    //
    // Display outer temperature on the segment display
    //
    
    byte Temp;
    
    // 
    // First digit is dash for negative temperatures, or blank
    //
    
    if (OuterTemperature < 0) {
      Segments [ScreenPageInactive()][0] = SEVEN_DASH;
      Temp = (-OuterTemperature + 8) / 16;
    } else {
      Segments [ScreenPageInactive()][0] = SEVEN_BLANK;
      Temp = (OuterTemperature + 8) / 16;
    }
   
    //
    // Second digit is tens of degrees
    //
    
    if (Temp >= 10) {
      Segments [ScreenPageInactive()][1] = SegHexTable [Temp / 10];
    } else {
      Segments [ScreenPageInactive()][1] = SEVEN_BLANK;
    }
    
    //
    // Rest is degrees and degrees of Celsius sign
    //
    
    Segments [ScreenPageInactive()][2] = SegHexTable [Temp % 10];
    Segments [ScreenPageInactive()][3] = SEVEN_DEGREE;
    Segments [ScreenPageInactive()][4] = SEVEN_C;
    Segments [ScreenPageInactive()][5] = SEVEN_BLANK;
    
    Flags |= FLAGS_FADE_PAGE;
  }
}

