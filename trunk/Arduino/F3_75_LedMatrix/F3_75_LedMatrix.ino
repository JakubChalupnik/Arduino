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
// Display size and pins used
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

#define LedR1Set(val) {if (val) LedR1High (); else LedR1Low (); }
#define LedR2Set(val) {if (val) LedR2High (); else LedR2Low (); }

//*******************************************************************************
//*                               Static variables                              *
//*******************************************************************************

//
// Screen buffer. For the moment just filled with a static picture.
//

uint8_t Screen [2][LED_HALF * 2] ={
 0, 0, 130, 192, 63, 255, 255, 255, 0, 0, 133, 224, 3, 255, 255, 255, 0, 0, 131, 224, 1, 13, 255, 255, 0, 1, 135, 240, 0, 162, 255, 255, 0, 1, 147, 96, 0, 24, 23, 255, 0, 1, 179, 192, 0, 11, 175, 255, 0, 1, 55, 128, 0, 0, 23, 255, 0, 1, 254, 0, 0, 0, 5, 127, 0, 3, 254, 32, 0, 0, 2, 191, 0, 3, 218, 0, 0, 0, 0, 239, 0, 1, 145, 128, 0, 0, 0, 31, 0, 1, 128, 0, 0, 0, 0, 31, 0, 0, 128, 0, 0, 0, 0, 7, 0, 0, 0, 0, 0, 0, 0, 23, 0, 0, 96, 0, 0, 1, 0, 7, 0, 0, 0, 0, 0, 1, 0, 11, 0, 0, 0, 64, 0, 3, 0, 7, 0, 0, 0, 0, 0, 3, 128, 3, 0, 0, 0, 0, 0, 7, 0, 7, 0, 0, 0, 0, 2, 5, 32, 3, 0, 0, 48, 0, 2, 7, 248, 7, 0, 1, 204, 0, 7, 6, 224, 7, 0, 3, 206, 0, 7, 135, 112, 7, 0, 6, 222, 0, 7, 207, 192, 15, 0, 15, 223, 0, 7, 156, 0, 14, 0, 12, 190, 0, 12, 12, 0, 63, 128, 12, 216, 0, 0, 0, 1, 255, 224, 12, 153, 0, 0, 0, 15, 255, 248, 12, 111, 0, 0, 0, 255, 255, 255, 12, 111, 0, 0, 3, 254, 139, 127, 198, 60, 0, 0, 31, 255, 254, 15, 254, 8, 0, 0, 127, 223, 255
};

//
// This variable controls pretty much everything. Main program can set FLAGS_FADE_PAGE
// or FLAGS_FLIP_PAGE (see above) but should not touch other flags.
// Main program shall always update the screen buffer that is returned by ScreenPageInactive()
//

volatile byte Flags;

#define FLAGS_FADING            0x08    // Do not use - interrupt routine uses this
#define FLAGS_FADE_PAGE         0x04    // Set for fading between screens (e.g. "flags |= FLAGS_FADE_PAGE;" )
#define FLAGS_FLIP_PAGE         0x02    // Set for flipping screens
#define FLAGS_PAGE_DISPLAYED    0x01    // Do not use - interrupt routine uses that
                                        // to keep track about what screen buffer is displayed
#define ScreenPageDisplayed()   (Flags & FLAGS_PAGE_DISPLAYED)
#define ScreenPageInactive()    (~Flags & FLAGS_PAGE_DISPLAYED)

//*******************************************************************************
//*                              Interrupt handler                              *
//*******************************************************************************

//
// Activated every 1ms
//

void LedScan (void);
ISR (TIMER1_COMPA_vect) {
  LedScan ();
}

//*******************************************************************************
//*                             LED matrix routines                             *
//*******************************************************************************

void LedClear (void) {

  memset (Screen, 0, sizeof (Screen));
}

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

  LedRow = (LedRow + 1) & (0x1F);

  if (LedRow > 15) {
    LedOeDisable ();
    return;  
  }
  
  LedClkLow ();
  LedByteUPtr = Screen[0] + LedRow * 8;
  LedByteLPtr = LedByteUPtr + LED_HALF;
  LedOeDisable ();

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

  LedOeEnable ();
}

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
//*                            Arduino setup method                             *
//*******************************************************************************

void setup () {

  Serial.begin (115200);
  Serial.println (F("[F3.75 LedMatrix test]"));

  LedConfig ();

  //
  // Set timer1 interrupt at 1 kHz
  //
  
  cli();

  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  OCR1A = 8000 - 1;       
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS10);  
  TIMSK1 |= (1 << OCIE1A);

  sei();
}
  
//*******************************************************************************
//*                              Main program loop                              *
//*******************************************************************************

void loop () {
  static uint32_t lastCountTime = 0;

  if (millis() != lastCountTime) {
    lastCountTime = millis();
  }
}
