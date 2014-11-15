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
//*                               Static variables                              *
//*******************************************************************************
//
// Screen buffer. For the moment just filled with a static picture.
//

uint8_t Screen [2][LED_HALF * 2];

//
// This variable controls pretty much everything. 
// Main program can set FLAGS_FLIP_PAGE but should not touch other flags.
// Main program shall always update the screen buffer that is returned by ScreenPageInactive().
// All Led... routines do that already.
//

volatile byte Flags;

#define FLAGS_FLIP_PAGE         0x02    // Set for flipping screens
#define FLAGS_PAGE_DISPLAYED    0x01    // Do not use - interrupt routine uses that

// to keep track about what screen buffer is displayed
#define ScreenPageDisplayed()  (Flags & FLAGS_PAGE_DISPLAYED)
#define ScreenPageInactive()   (~Flags & FLAGS_PAGE_DISPLAYED)


//*******************************************************************************
//*                          Fonts and bitmaps                                  *
//*******************************************************************************
//
// Fonts by holger.klabunde@t-online.de - I found them in a package named
// "T6963 based LCD(8x8).zip"
// The fonts are in program memory space so they have to be accessed in a special way!
//

const byte font_8x6[96 * 8] PROGMEM = {
#include "FN6X8_reduced.h"
};

#define FontByte(Index) pgm_read_byte (((PGM_P) font_8x6) + Index)

const byte BitmapAda_P[] PROGMEM = {
#include "BitmapAda.h"
};


//*******************************************************************************
//*                              Interrupt handler                              *
//*******************************************************************************
//
// Activated every 0.5ms, uses timer1.
// At the moment only updates the LED matrix.
//

void LedScan (void);
ISR (TIMER1_COMPA_vect) {
  LedScan ();
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

  //  LedRow++;
  //  
  //  if (LedRow >= 32) {
  //    LedRow = 0;
  //  } else if (LedRow > 15) {
  //    LedOeDisable ();
  //    return;  
  //  }
  //  

  LedOeDisable ();

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

  LedSetRow (LedRow);
  LedStrobeLow ();
  LedStrobeHigh ();
  LedStrobeLow ();

  LedRow = (LedRow + 1) & 0x0F;

  //
  // Handle page fliping - if the flag is set, wait until the whole buffer
  // is displayed (i.e. Row is 0 again) and then toggle the FLAGS_PAGE_DISPLAYED
  //

  if((LedRow == 0) && (Flags & FLAGS_FLIP_PAGE)) {       // If someone requested fliping page
    Flags ^= FLAGS_PAGE_DISPLAYED;  // Flip the pages - displayed becomes inactive and vice versa
    Flags &= ~FLAGS_FLIP_PAGE;      // and clear the flag to signal the page was flipped
  }

  LedOeEnable ();
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
// Sets/clears the pixel at coordinates x, y
// It does not check any boundaries. 
// When called with wrong arguments, might crash the whole thing!
//

void LedPixelSet (uint8_t x, uint8_t y, uint8_t b) {
  uint8_t *ScreenPtr;
  uint8_t Mask;

  Mask = 1 << (7 - (x & 0x07));
  ScreenPtr = Screen [ScreenPageInactive ()] + (y << 3) + (x >> 3); 
  if (b) {
    *ScreenPtr |= Mask;
  } 
  else {
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

//
// Draws a character at specified coordinates x, y. 
// Assumes first valid character to be ' ', draws garbage for anything below ' '
//

void PutChar(uint8_t x, uint8_t y, byte c) {

  LedBitmap_P (x, y, 6, 8, (PGM_P) (font_8x6 + 8 * (c - ' ')));
}

const byte Sprite_P [] PROGMEM = {
  0x70,		// .XXX....
  0x88,		// X...X...
  0x08,		// ....X...
  0x30,		// ..XX....
  0x20,		// ..X.....
  0x00,		// ........
  0x20,		// ..X.....
}; 

//*******************************************************************************
//*                            Arduino setup method                             *
//*******************************************************************************

void setup () {

  Serial.begin (115200);
  Serial.println (F("[F3.75 LedMatrix test]"));

  LedConfig ();

  //
  // Set timer1 interrupt at 2 kHz
  //

  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  OCR1A = 8000 - 1;       
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS10);  
  TIMSK1 |= (1 << OCIE1A);

  //  LedBitmap_P (0, 0, 64, 32, (PGM_P) BitmapAda_P);
}

//*******************************************************************************
//*                              Main program loop                              *
//*******************************************************************************

void loop () {
  int i;

  //  for (i = 0; i < 24; i++) {
  //    LedClear ();
  //    LedBitmap_P (i, i, 6, 7, (PGM_P) Sprite_P);
  //    Flags |= FLAGS_FLIP_PAGE;
  //    delay (100);
  //  }

  LedClear ();
  LedBitmap_P (0, 0, 64, 32, (PGM_P) BitmapAda_P);
  Flags |= FLAGS_FLIP_PAGE;
  delay (100);
}

