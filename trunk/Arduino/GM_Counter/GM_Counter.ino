#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>

//
// pin 3 - Serial clock out (SCLK)
// pin 4 - Serial data out (DIN)
// pin 5 - Data/Command select (D/C)
// pin 6 - LCD chip select (CS)
// pin 7 - LCD reset (RST)
//

Adafruit_PCD8544 display = Adafruit_PCD8544 (3, 4, 5, 6, 7);

//static unsigned char PROGMEM logo16_glcd_bmp[] = { 
//  B00000000, B11000000,
//  B00000001, B11000000,
//  B00000001, B11000000,
//  B00000011, B11100000,
//  B11110011, B11100000,
//  B11111110, B11111000,
//  B01111110, B11111111,
//  B00110011, B10011111,
//  B00011111, B11111100,
//  B00001101, B01110000,
//  B00011011, B10100000,
//  B00111111, B11100000,
//  B00111111, B11110000,
//  B01111100, B11110000,
//  B01110000, B01110000,
//  B00000000, B00110000 
//};
//

volatile uint32_t ClickCounter = 0;
#define COUNTER_INPUT 2
#define COUNTER_IRQ 0

void ClickIrq () {
  ClickCounter++;
}
    
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
  display.setContrast (52);
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

uint32_t CpsTime = 0, CpsCount = 0, CpsAverage = 0;
uint32_t CpmTime = 0, CpmCount = 0, CpmAverage = 0;

void loop (void) {
  uint32_t Millis = millis ();
  
  if (CpmTime == 0) {
    
    //
    // Start measuring CPM
    //
  
    CpmTime = Millis;
    CpmCount = ClickCounter;
  } else if ((CpmTime + 60000) < Millis) {

    //
    // One minute passed, show the CPM and reset CpmTime to start again
    //
    
    CpmAverage = ClickCounter - CpmCount;
//    CpmAverage = ((CpmAverage * 3) + CpmCount) >> 2;
    CpmTime = 0;
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
    display.println (CpmAverage);
//    display.print (" / ");
//    display.println (CpmAverage);

#define CPS_PER_MR_H 29
    display.print ("Dose ");
    display.print (CpmAverage / (CPS_PER_MR_H * 60));
    display.println ("mR/h");

    display.print ("Counter ");
    display.println (ClickCounter);
    display.display();
    CpsTime = 0;
  }
}

