//*******************************************************************************
//*
//* Arduino based router for RF24Netword handheld with Nokia screen
//* Based on example sketch for Adafruit Monochrome Nokia 5110 LCD Displays
//* Based on "helloworld" from RF24Network library by J. Coliz <maniacbug@ymail.com> / TMRh20
//*
//*******************************************************************************
//* Processor:  Arduino Pro Mini 5V/16MHz
//* Author      Date       Comment
//*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//* Kubik       13.1.2015 First version, just basic code for HW tests
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
//  Buttons         7, 8
//

//*******************************************************************************
//*                           Includes and defines                              *
//******************************************************************************* 

#define VERSION 0

#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>
#include <RF24Network.h>
#include <RF24.h>
#include <SPI.h>
#include <Bounce2.h>
#include <Rf24PacketDefine.h>

#define BUTTON_UP_PIN 7
#define BUTTON_DOWN_PIN 8 

#define THIS_NODE 00    // Address of our node in Octal format. This is main router, thus 00
#define MAX_NODE_COUNT 32
  

typedef struct {
  uint16_t Address;
  uint8_t BattLevel;
  uint8_t Temperature[4];
  char Id[8];
  uint16_t Flags;
} Node_t;
  
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

Adafruit_PCD8544 display = Adafruit_PCD8544 (2, 3, 4, 5, 6);

//
// RF24 variables
//

RF24 Radio (9, 10);              // nRF24L01(+) radio attached using standard pins 9,10
RF24Network Network (Radio);    // Network uses that radio
Node_t Nodes [MAX_NODE_COUNT];
uint8_t NodeCount = 0;

//
// Buttons support
//

Bounce ButtonUp = Bounce();  
Bounce ButtonDown = Bounce();  

//*******************************************************************************
//*                            Arduino setup method                             *
//******************************************************************************* 

void setup (void) {
  
  //
  // Setup LCD, print some basic info
  //
  
  display.begin ();
  display.setContrast (48);
  display.clearDisplay ();   // clears the screen and buffer
  display.setTextSize (1);
  display.setTextColor (BLACK);
  display.setCursor (0, 0);
  display.print (F("RF24RouterNode"));
  display.setCursor (0, 10);
  display.print (F("(c) 2015 Kubik"));
  display.setCursor (0, 20);
  display.print (F("Version "));
  display.print (VERSION);
  display.display ();

  //
  // Serial support init
  //
  
  Serial.begin (57600);
  Serial.println (F("[RF24RouterNode]"));
 
  //
  // RF24Network init
  //
  
  SPI.begin ();
  Radio.begin ();
  Network.begin (RF24_CHANNEL, THIS_NODE);
  
  //
  // Button support
  //
  
  pinMode (BUTTON_UP_PIN, INPUT_PULLUP);
  pinMode (BUTTON_DOWN_PIN, INPUT_PULLUP);

  ButtonUp.attach (BUTTON_UP_PIN);
  ButtonDown.attach (BUTTON_DOWN_PIN);
  ButtonUp.interval(5);
  ButtonDown.interval(5); 

  //
  // Other setup stuff
  //
  
  memset (Nodes, 0, sizeof (Nodes));
  delay (2000);
}

//*******************************************************************************
//*                              Main program loop                              *
//******************************************************************************* 

void loop (void) {
  static uint32_t PacketCounter = 0;
//  static uint32_t LastFails = 0xFFFFFFFF, LastOks = 0xFFFFFFFF;
  uint32_t Fails, Oks;
  RF24NetworkHeader Header;        // If so, grab it and print it out
  PayloadTemperature_t *Payload;
//  static enum {E_BATT, E_TEMP} State = E_BATT;
  uint8_t i;
//  PayloadDefault_t *PayloadDefault;
//  PayloadTime_t *PayloadTime;
//  PayloadTemperature_t *PayloadTemperature;
  
  PayloadId_t *PayloadId;
  uint8_t Buffer[32];
  
  Payload = (PayloadTemperature_t *) Buffer;
  PayloadId = (PayloadId_t *) Buffer;


  
  //
  // Things that have to be done every loop pass
  //
  
  Network.update ();                 // Check the network regularly
  ButtonUp.update ();
  ButtonDown.update ();

//  Network.failures (&Fails, &Oks);

  while (Network.available ()) {     // Is there anything ready for us?
    Network.read (Header, &Buffer, sizeof (Buffer));
    PacketCounter++;
    
    for (i = 0; i < NodeCount; i++) {
      if (Header.from_node == Nodes [i].Address) {
        break;
      }
    }
    
    Nodes [i].Address = Header.from_node;
    Nodes [i].BattLevel = Payload->BattLevel;
    switch (Header.type) {
      case RF24_TYPE_TEMP:
        Nodes [i].Temperature [0] = Payload->Temperature [0];
        Nodes [i].Temperature [1] = Payload->Temperature [1];
        Nodes [i].Temperature [2] = Payload->Temperature [2];
        Nodes [i].Temperature [3] = Payload->Temperature [3];
        break;
        
      case RF24_TYPE_ID:
        memcpy (Nodes [i].Id, PayloadId->Id, 8);
        Nodes [i].Flags = PayloadId->Flags;
        break;
        
      default:
        break;
    }

    if ((i == NodeCount) && (NodeCount < (MAX_NODE_COUNT - 1)))  {        // New node reporting itself
      NodeCount++;
    }

//    Serial.print("Received packet #");
//    Serial.print(payload.counter);
//    Serial.print(" at ");
//    Serial.println(payload.ms);
  }
  
//  int value = debouncer.read();
//
//  // Turn on or off the LED as determined by the state :
//  if ( value == LOW ) {   
  
  
  display.clearDisplay ();   // clears the screen and buffer
  display.setCursor (0, 0);
  display.print (F("Packets: "));
  display.print (PacketCounter);
  
  for (i = 0; i < NodeCount; i++) {
    char s[16];
//    sprintf (s, "%2.2o %3d %2d", Nodes [i].Address, Nodes [i].BattLevel, Nodes [i].Temperature [0]);
    sprintf (s, "%2.2o %8s %2d", Nodes [i].Address, Nodes [i].Id, Nodes [i].Temperature [0]);
    display.setCursor (0, (i + 1) * 8);
    display.print (s);
//    display.print (Nodes [i].Address);
//    display.print (F(" "));
//    display.print (Nodes [i].BattLevel);
//    display.print (F(" "));
//    display.print (Nodes [i].Temperature [0]);
  }
  display.display ();
  
}

