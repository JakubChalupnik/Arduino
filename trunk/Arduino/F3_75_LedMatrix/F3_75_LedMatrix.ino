//*******************************************************************************
//*
//* Arduino driver for the 64x32 matrix LED (F3.75 display)
//* The display uses 74138 to select 1 of 16 rows, and 74595 to clock in data
//*
//*******************************************************************************
//* FileName:   main.c
//* Depends:
//* Processor:  Arduino Pro Mini 3.3V
//*
//* Author      Date       Comment
//*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//* Kubik       18.10.2014 First working version with Pro Mini
//* Kubik       18.10.2014 Added interrupt support
//* Kubik       18.10.2014 Added pixel, font and bitmap drawing
//* Kubik       15.11.2014 Fixed pixel overflow, added some font functions and dimming
//* Kubik       15.11.2014 Added 8x14 font, OR drawing method and RFM12B support (Jeelib)
//* Kubik       16.11.2014 Added photoresistor support (used for dimming)

//*******************************************************************************
//*                            HW details                                       *
//*******************************************************************************
// 
// Mostly using PORTC to drive row selectors.
// When changing R1 and R2, the corresponding LedR1Low... macros have to be changed!
// (digitalWrite is too slow)
// The display has 64 rows, 32 columns. 
// R1 is the data input to the upper 16 rows, R2 the same for lower 16 rows.

//*******************************************************************************
//*                           Includes and defines                              *
//*******************************************************************************
//
// Display size and pins used. Note that most of the pin definitions are basically
// unused as the access is done by direct port access!
//

#define LED_WIDTH   64
#define LED_HEIGHT  32
#define LED_HALF (LED_WIDTH * LED_HEIGHT / 8 / 2)
#define LED_SCAN_LINES 16
#define LED_SCAN_MASK 0x0F

#define LED_PIN_A 14
#define LED_PIN_B 15
#define LED_PIN_C 16
#define LED_PIN_D 17

#define LED_PIN_OE 7
#define LED_PIN_R1 8
#define LED_PIN_R2 9
#define LED_PIN_STB 18
#define LED_PIN_CLK 19

//
// Display related macros.
// The time critical macros do not use digitalWrite, but direct port access!
//

#define LedClkLow() PORTC &= ~0b00100000
#define LedClkHigh() PORTC |= 0b00100000

#define LedStrobeLow() PORTC &= ~0b00010000
#define LedStrobeHigh() PORTC |= 0b00010000

#define LedOeDisable() PORTD |= 0b10000000
#define LedOeEnable()  PORTD &= ~0b10000000

#define LedR1Low() PORTB &= ~0b00000001
#define LedR1High() PORTB |= 0b00000001

#define LedR2Low() PORTB &= ~0b00000010
#define LedR2High() PORTB |= 0b00000010

#define LedR1Set(val) {if (val) LedR1Low (); else LedR1High (); }
#define LedR2Set(val) {if (val) LedR2Low (); else LedR2High (); }

//*******************************************************************************
//*                               printf support                                *
//*******************************************************************************
#include <stdio.h>
static FILE uartout = {0} ;
static int uart_putchar (char c, FILE *stream) {
    Serial.write(c) ;
    return 0 ;
}  

//*******************************************************************************
//*                               Static variables                              *
//*******************************************************************************
//
// Screen buffer. Two pages for page flipping support
//

uint8_t Screen [2][LED_HALF * 2];

//
// This variable controls pretty much everything. 
// Main program can set FLAGS_FLIP_PAGE but should not touch other flags.
// Main program shall always update the screen buffer that is returned by ScreenPageInactive().
// All Led... routines do that already.
//

volatile byte Flags;

#define FLAGS_DRAW_OR           0x04    // When set, LED routines do not clear pixels
#define FLAGS_FLIP_PAGE         0x02    // Set for flipping screens
#define FLAGS_PAGE_DISPLAYED    0x01    // Do not use - interrupt routine uses that

// Used to keep track about what screen buffer is displayed
#define ScreenPageDisplayed()  (Flags & FLAGS_PAGE_DISPLAYED)
#define ScreenPageInactive()   (~Flags & FLAGS_PAGE_DISPLAYED)

//
// Defines the brightness of the display, 0 is most bright, 8 is darkest.
// Higher values could lead to picture flickering etc.
//

byte LedPwm = 4;

unsigned char Day = 0;
unsigned char Month = 0;
unsigned char Hour = 0;
unsigned char Minute = 0;
int InnerTemperature;
int OuterTemperature; 

//*******************************************************************************
//*                          Fonts and bitmaps                                  *
//*******************************************************************************
//
// Fonts by holger.klabunde@t-online.de - I found them in a package named
// "T6963 based LCD(8x8).zip"
// The fonts are in program memory space so they have to be accessed in a special way!
//

const byte Font8x6[96 * 8] PROGMEM = {
#include "FN6X8_reduced.h"
};

// #define Font8Byte(Index) pgm_read_byte (((PGM_P) Font8x6) + Index)

const byte Font8x14[256 * 8 * 2] PROGMEM = {
#include "Bold8x14.h"
};

const byte BitmapAda_P[] PROGMEM = {
//#include "BitmapAda.h"
};

//*******************************************************************************
//*                              Interrupt handler                              *
//*******************************************************************************
//
// Activated every 0.125ms, uses timer1.
// At the moment only updates the LED matrix.
//

void LedScan (void);
ISR (TIMER1_COMPA_vect) {
  LedScan ();
}

//*******************************************************************************
//*                              RFM12B support                                 *
//*******************************************************************************
#include <Time.h>
#include <JeeLib.h>
#include <PacketDefine.h>

#define DEBUG_RMF12 0

//----------------------------------------------------------------------------
//
// PacketDecode processes the packet received from RFM12 and updates the global variables for time, pressure, humidity and external temperature accordingly
//

void PacketDecode () {
  char c[4];
  MeteoPayload_t *Meteo;
  TimePayload_t *Time;
  TemperaturePayload_t *Temp;

  switch (*rf12_data) {
    
    case RF12_PACKET_TIME:
      Time = (TimePayload_t *) rf12_data;
      Day = Time->day;
      Month = Time->month;
      Hour = Time->hour;
      Minute = Time->minute;
      setTime (Hour, Minute, 0, Day, Month, 2000 + Time->year);

#if DEBUG_RMF12
      Serial.print (F("Time packet: "));
      Serial.print (Day);
      Serial.print ('.');
      Serial.print (Month);
      Serial.print (' ');
      Serial.print (2000 + Time->year);
      Serial.print (' ');
      Serial.print (Hour);
      Serial.print (':');
      Serial.println (Minute);
#endif // DEBUG_RMF12

      break;

    case RF12_PACKET_METEO:

      break;
      
    case RF12_PACKET_TEMPERATURE:
      Temp = (TemperaturePayload_t *) rf12_data;
      InnerTemperature = Temp->temp1;
      OuterTemperature = Temp->temp2;

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
//*                             LED matrix routines                             *
//*******************************************************************************
//
// Low level functions: LedSetRow, LedScan
// 

void LedSetRow (uint8_t Row) {

  PORTC = (PORTC & 0xF0) | Row;
}

//
// The routine updates the screen matrix. If page flip is required (FLAGS_FLIP_PAGE),
// we change the page as soon as the whole display is finished to avoid flickering
//

void LedScan (void) {
  uint8_t i, b, LedByteU, LedByteL;
  uint8_t *LedByteUPtr, *LedByteLPtr;
  static uint8_t LedRow = 0;         // Active row - the one that's just displayed
  static byte LedPwmCnt = 0;

  //
  //  Handle display dimming - this is done for every scan line by disabling the display for next LedPwm interrupts
  //
  
  if (LedPwmCnt > 0) {
    LedOeDisable ();
    LedPwmCnt --;
    return;
  }
  
  LedPwmCnt = LedPwm;

  //
  // Get the scan lines for both upper and lower half of the screen and bit bang it into the display buffer (74HC565).
  // 
  
  LedClkLow ();
  LedByteUPtr = Screen[ScreenPageDisplayed()] + LedRow * 8;
  LedByteLPtr = LedByteUPtr + LED_HALF;

  for (i = 0; i < 8; i++) {
    LedByteU = *LedByteUPtr++;
    LedByteL = *LedByteLPtr++;
    for (b = 0; b < 8; b++) {
      LedR1Set (LedByteU & 0x80);
      LedR2Set (LedByteL & 0x80);

      LedByteU <<= 1;
      LedClkHigh ();
      LedByteL <<= 1;
      LedClkLow ();
    }
  }

  // 
  // The screen line data are shifted into the 74HC565 internal buffers, now select the correct row (1..16) 
  // and write the data into output buffers
  //
  
  LedOeDisable ();
  LedSetRow (LedRow);
  LedStrobeLow ();
  LedStrobeHigh ();
  LedStrobeLow ();
  LedOeEnable ();

  LedRow = (LedRow + 1) & 0x0F;

  //
  // Handle page fliping - if the flag is set, wait until the whole buffer
  // is displayed (i.e. Row is 0 again) and then toggle the FLAGS_PAGE_DISPLAYED
  //

  if((LedRow == 0) && (Flags & FLAGS_FLIP_PAGE)) {       // If someone requested fliping page
    Flags ^= FLAGS_PAGE_DISPLAYED;  // Flip the pages - displayed becomes inactive and vice versa
    Flags &= ~FLAGS_FLIP_PAGE;      // and clear the flag to signal the page was flipped
  }

}

//
// Configures pins used for LED matrix communication
//

void LedConfig (void) {

  pinMode(LED_PIN_A, OUTPUT);
  pinMode(LED_PIN_B, OUTPUT);
  pinMode(LED_PIN_C, OUTPUT);
  pinMode(LED_PIN_D, OUTPUT);
  pinMode(LED_PIN_OE, OUTPUT);
  pinMode(LED_PIN_R1, OUTPUT);
  pinMode(LED_PIN_STB, OUTPUT);
  pinMode(LED_PIN_CLK, OUTPUT);
  pinMode(LED_PIN_R2, OUTPUT);

  LedStrobeLow ();
  LedClkLow ();
  LedOeDisable ();
}

//*******************************************************************************
//*                              Led Matrix Drawing                             *
//*******************************************************************************

//
// Clears the inactive screen page
//

void LedClear (void) {

  memset (Screen [ScreenPageInactive ()], 0x00, sizeof (Screen [0]));
}

//
// Sets/clears the pixel at coordinates x, y.
// It does not check any boundaries. 
// When called with wrong arguments, might crash the whole thing!
//

void LedPixelSet (uint8_t x, uint8_t y, uint8_t b) {
  uint8_t *ScreenPtr;
  uint8_t Mask;

  if ((Flags & FLAGS_DRAW_OR) && !b) return;
  Mask = 1 << (7 - (x & 0x07));
  ScreenPtr = Screen [ScreenPageInactive ()] + (y << 3) + (x >> 3); 
  if (b) {
    *ScreenPtr |= Mask;
  } else {
    *ScreenPtr &= ~Mask;
  }
}

//
// Gets the value of the pixel at coordinates x, y
// Does not check any boundaries, may return garbage on out-of-screen access
//

uint8_t LedPixelGet (uint8_t x, uint8_t y) {
  uint8_t *ScreenPtr;
  uint8_t Mask;

  Mask = 1 << (7 - (x & 0x07));
  ScreenPtr = Screen [ScreenPageInactive ()] + (y << 3) + (x >> 3); 

  return (*ScreenPtr & Mask) ? 1 : 0;
}

//
// Draws bitmap Bitmap from RAM at coordinates X, Y. Pixel size of the bitmap is XSize, YSize
// Check boundaries, and crops or bails out on invalid input.
//

void LedBitmap (uint8_t X, uint8_t Y, uint8_t XSize, uint8_t YSize, uint8_t *Bitmap) {
  uint8_t x, y;
  uint8_t Pixel, XSizeBytes;

  //
  // Test input parameters and either crop them or bail out completely.
  // X and Y can't be negative as they're defined as UINT.
  //

  if (X >= LED_WIDTH) {  
    return;
  }

  if (Y >= LED_HEIGHT) {  
    return;
  }

  if ((X + XSize) > LED_WIDTH) {  
    XSize =  LED_WIDTH - X;
  }

  if ((Y + YSize) > LED_HEIGHT) {  
    YSize =  LED_HEIGHT - Y;
  }

  XSizeBytes = (XSize + 7) >> 3;

  //
  // Go through all rows (Y) of the bitmap, and process every row bit by bit
  // To get the value of the pixel, find the corresponding byte in bitmap first:
  // Multiple current row by width of bitmap in bytes, then add X divided by 8 (8 bits per byte)
  //

  for (y = 0; y < YSize; y++) {
    for (x = 0; x < XSize; x++) {
      Pixel = Bitmap [(y * XSizeBytes) + (x >> 3)];
      LedPixelSet (X + x, Y + y, Pixel & (1 << (7 - (x & 0x07))));
    }
  }
}

//
// Draws bitmap Bitmap from PGM at coordinates X, Y. Pixel size of the bitmap is XSize, YSize
// Check boundaries, and crops or bails out on invalid input.
//

void LedBitmap_P (uint8_t X, uint8_t Y, uint8_t XSize, uint8_t YSize, PGM_P Bitmap) {
  uint8_t x, y;
  uint8_t Pixel, XSizeBytes;
  
  //
  // Test input parameters and either crop them or bail out completely.
  // X and Y can't be negative as they're defined as UINT.
  //

  if (X >= LED_WIDTH) {  
    return;
  }

  if (Y >= LED_HEIGHT) {  
    return;
  }

  if ((X + XSize) > LED_WIDTH) {  
    XSize =  LED_WIDTH - X;
  }

  if ((Y + YSize) > LED_HEIGHT) {  
    YSize =  LED_HEIGHT - Y;
  }

  XSizeBytes = (XSize + 7) >> 3;

  //
  // Go through all rows (Y) of the bitmap, and process every row bit by bit
  // To get the value of the pixel, find the corresponding byte in bitmap first:
  // Multiple current row by width of bitmap in bytes, then add X divided by 8 (8 bits per byte)
  //

  for (y = 0; y < YSize; y++) {
    for (x = 0; x < XSize; x++) {
      Pixel = pgm_read_byte (Bitmap + (y * XSizeBytes) + (x >> 3));
      LedPixelSet (X + x, Y + y, Pixel & (1 << (7 - (x & 0x07))));
    }
  }
}

//*******************************************************************************
//*                            Text support functions                           *
//*******************************************************************************
// Draws a character / string of Font8x6 at specified coordinates x, y. 
// Assumes first valid character to be ' ', draws garbage for anything below ' '
//

void PutChar8 (uint8_t x, uint8_t y, byte c) {

  LedBitmap_P (x, y, 6, 8, (PGM_P) (Font8x6 + 8 * (c - ' ')));
}

void PutString8 (uint8_t x, uint8_t y, const char *s) {
  
  while (*s != 0) {
    LedBitmap_P (x, y, 6, 8, (PGM_P) (Font8x6 + 8 * (*s - ' ')));
    s++;
    x += 6;
  }
}

void PutChar14 (uint8_t x, uint8_t y, byte c) {

  LedBitmap_P (x, y, 8, 14, (PGM_P) (Font8x14 + 14 * c));
}


const byte Sprite_p[] PROGMEM = {
  0x00, 0x00, 0x1F, 0x20, 0x29, 0x21, 0x21, 0x29, 0x20, 0x1F, 0xC0, 0x20, 0x20, 0xA0,
};



//*******************************************************************************
//*                            Arduino setup method                             *
//*******************************************************************************

void setup () {

  Serial.begin (115200);
  Serial.println (F("[F3.75 LedMatrix test]"));

  LedConfig ();

  Flags |= FLAGS_DRAW_OR;    // Use OR method of drawing

  //
  // Set timer1 interrupt at relatively quick rate (8kHz) to help with PWM dimming
  //

  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  OCR1A = 2000 - 1;       
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS10);  // No prescaler
  TIMSK1 |= (1 << OCIE1A);

  // fill in the UART file descriptor with pointer to writer.
  fdev_setup_stream (&uartout, uart_putchar, NULL, _FDEV_SETUP_WRITE);
  // The uart is the standard output device STDOUT.
  stdout = &uartout ; 

  //
  // Display name and copyright
  // 
  
  LedPwm = 0;
  LedClear ();
  PutString8 (11, 0, "F3.75-B");
  PutString8 (14, 8, "Matrix");
  PutString8 (8, 16, "(c) 2014");
  PutString8 (16, 24, "Kubik");

  Flags |= FLAGS_FLIP_PAGE;

  rf12_initialize(RF12_NET_NODE, RF12_868MHZ, RF12_NET_GROUP); 
  setTime (0, 0, 0, 1, 1, 2013);    // Dummy time until the time gets synced 
  
  delay (1000);
}

//*******************************************************************************
//*                              Main program loop                              *
//*******************************************************************************

void loop () {
  unsigned long Millis;
  static unsigned long LastMillis;
  static int Second = 0;
  time_t Time; 
  static time_t PreviousTime = 0;
  static unsigned int LightIntensity = 0; 
  
  //
  // Get the time stamp of this loop pass
  //
  
  Millis = millis();
 
  //
  // Check if any valid packet was received, and decode it
  //
  
  if (rf12_recvDone() && rf12_crc == 0) {
    PacketDecode ();
  }

  //
  // Every time the Millis changes, measure the light intensity, set the PWM to reflex it, and refresh the display
  //
  
  if (Millis = LastMillis) {
    return;
  }
  LastMillis = Millis;
  
  //
  // LedPwm can be from 0 (full) to ~8 (dimmed).
  // Light values are between single digit numbers for bright light to ~1000 for dark night.
  //
  
  LightIntensity = ((LightIntensity * 31) + analogRead(7)) / 32;
  if (LightIntensity < 100) {
    LedPwm = 0;
  } else if (LightIntensity < 200) {
    LedPwm = 1;
  } else if (LightIntensity < 500) {
    LedPwm = 2;
  } else {
    LedPwm = 8;
  }
  
  //
  // Did second change?
  //

  Time = now ();
  
  if (Time == PreviousTime) {
    return;
  }
  
  PreviousTime = Time; 
  
  //
  // Do the following every second
  //

  LedClear ();
  if (hour () > 9) {
    PutChar8 (12 + 1, 2, (hour () / 10) + '0');
    PutChar14 (5, 20, (hour () / 10) + '0');
  }

  PutChar8 (12 + 7, 2, (hour () % 10) + '0');
  PutChar8 (12 + 11, 2, ':');
  PutChar8 (12 + 15, 2, (minute () / 10) + '0');
  PutChar8 (12 + 21, 2, (minute () % 10) + '0');
  PutChar8 (12 + 25, 2, ':');
  PutChar8 (12 + 29, 2, (second () / 10) + '0');
  PutChar8 (12 + 35, 2, (second () % 10) + '0');

  PutChar14 (13, 20, (hour () % 10) + '0');
  PutChar14 (19, 20, ':');
  PutChar14 (25, 20, (minute () / 10) + '0');
  PutChar14 (33, 20, (minute () % 10) + '0');
  PutChar14 (39, 20, ':');
  PutChar14 (45, 20, (second () / 10) + '0');
  PutChar14 (53, 20, (second () % 10) + '0');

  Flags |= FLAGS_FLIP_PAGE;
}

