//*******************************************************************************
//*
//* Arduino Due based central node
//* Based on example sketches for Adafruit_GFX by Adafruit
//* Based on example sketches for EtherCard library
//*
//*******************************************************************************
//* Processor:  Arduino Due
//* Author      Date       Comment
//*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//* Kubik       18.2.2015 First version, just basic code for HW tests
//* Kubik       20.2.2015 Added two RF24 modules
//* Kubik       28.2.2015 Added three LEDs
//* Kubik       1.3.2015  Added time support
//*******************************************************************************

//*******************************************************************************
//*                            HW details                                       *
//*******************************************************************************
//
// Used pins:
//  ILI9341 display: HW SPI and
//   pin 51 - DC
//   pin 52 - CS
//   pin 53 - RESET
//
//  nRF24L01+ radios: HW SPI and
//   pin 10 - CSN 1
//   pin 23 - CE 1
//   pin 4  - CSN 2
//   pin 22 - CE 2
//
//  ENC28J60 (partially configured within the library!):
//   pin 50 - MISO
//   pin 49 - MOSI
//   pin 48 - SCK
//   pin 47 - RESET
//   pin 46 - CS
//
//  Indication LEDs
//   pin 11 - white LED
//   pin 12 - yellow LED
//   pin 13 - red LED
//

//*******************************************************************************
//*                        User configurable SW settings                        *
//******************************************************************************* 

#define DEBUG 1
#define TIME_UPDATE_PERIOD 3600
#define STATIC 0  		// set to 1 to disable DHCP (adjust myip/gwip values below)
#define MAX_TEMP_SENSORS 10

//*******************************************************************************
//*                           Includes and defines                              *
//******************************************************************************* 

#define VERSION 0 

#include <ILI9341_due_gText.h>
#include "fonts\Arial_bold_14.h"
#include <ILI9341_due.h>
#include <EtherCard.h>
#include <SPI.h>
#include <Time.h>
#include <Timezone.h>    // https://github.com/JChristensen/Timezone   
#include "RF24.h"
#include "RF24Network.h"
#include "Rf24PacketDefine.h"

#define TFT_DC 51
#define TFT_CS 52
#define TFT_RESET 53

#define NET_RST  47
#define NET_CS   46

#define RF24_1_CSN 10
#define RF24_1_CE 23
#define RF24_2_CSN 4
#define RF24_2_CE 22

#define LED_WHITE 11
#define LED_YELLOW 12
#define LED_RED 13

typedef struct {
  uint16_t Address;
  uint8_t BattLevel;
  uint16_t Temperature[2];
  char Id[NODE_ID_SIZE];
  uint16_t Flags;
} NetworkNode_t;

//*******************************************************************************
//*                               Static variables                              *
//******************************************************************************* 

//
// RF24 related
// Radio1 is used to communicate with sensors and send out broadcasts messages 
// for all standalone nodess that might care.
// Radio2 is used for RF24Network
//

RF24 Radio1 (RF24_1_CE, RF24_1_CSN);
RF24 Radio2 (RF24_2_CE, RF24_2_CSN);
RF24Network Network (Radio2);   

//
// Display related
//

ILI9341_due tft = ILI9341_due (TFT_CS, TFT_DC);
ILI9341_due_gText Text (&tft);

//
// Network related. 
// Static addresses are used either when STATIC is enabled, or DHCP fails
// Ethernet MAC address must be unique on the network
// buffer is used as tcp/ip send and receive buffer 
//

static const byte myip[] = {192, 168, 10, 200};
static const byte gwip[] = {192, 168, 10, 1};

static const byte mymac[] = { 0x74,0x69,0x69,0x33,0x30,0x31 };

byte Ethernet::buffer[500]; 

//
// Arrays of all found sensors and their latest payloads
//

SensorPayloadTemperature_t TempSensors [MAX_TEMP_SENSORS];
int TempSensorsCount = 0;

//
// Globally available and used flags
//

uint16_t Flags = 0;
#define F_ETHERNET 0x0001    // Ethernet controller operational

//
// Time related for Central European Time (Frankfurt, Paris)
//

TimeChangeRule CEST = {"CEST", Last, Sun, Mar, 2, 120};     //Central European Summer Time
TimeChangeRule CET = {"CET ", Last, Sun, Oct, 3, 60};       //Central European Standard Time
Timezone CE (CEST, CET);   

//*******************************************************************************
//*                          Debug support                                      *
//*******************************************************************************

#if DEBUG
  #define Debug(...) {Serial.print(__VA_ARGS__);}
  #define Debugln(...) {Serial.println(__VA_ARGS__);}
  #define Dprintf(Format, ...) {Serial.print(Format, __VA_ARGS__);}
#else
  #define Debug(...)
  #define Debugln(...)
  #define Dprintf(Format, ...)
#endif 

//*******************************************************************************
//*                            Arduino setup method                             *
//******************************************************************************* 

void setup () {
  char buff [64];
  
  //
  // Initialize pins used for resets of various peripherals, and reset all of them
  //
  
  pinMode (TFT_RESET, OUTPUT);
  pinMode (NET_RST, OUTPUT);
  digitalWrite (TFT_RESET, LOW);
  digitalWrite (NET_RST, LOW);
  delay (1);
  digitalWrite (TFT_RESET, HIGH);
  digitalWrite (NET_RST, HIGH);
  delay (10);
  
  //
  // Init pins used for LED, turn all of them off
  //

  analogWriteResolution (8);
  pinMode (LED_WHITE, OUTPUT);
  analogWrite (LED_WHITE, 0);
  pinMode (LED_YELLOW, OUTPUT);
  analogWrite (LED_YELLOW, 0);
  pinMode (LED_RED, OUTPUT);
  analogWrite (LED_RED, 0);
  
  //
  // Initialize serial port and send info
  //
  
  Serial.begin (57600);
  while (!Serial); 
  Serial.println ("[Arduino Due Central Node!]"); 

  //
  //  Initialize the display, set small text and red color for status messages
  //
  
  tft.begin ();
  tft.setRotation(iliRotation270);
  tft.fillScreen (ILI9341_BLACK);
  
  Text.defineArea (0, 0, 320, 240);
  Text.selectFont (Arial_bold_14);
  Text.setFontLetterSpacing (5);
  Text.setFontMode (gTextFontMode_Solid);
  Text.setFontColor (ILI9341_WHITE, ILI9341_BLACK);

  //
  // Initialise RF24 stuff.
  // The first radio is used to communicate with sensors and send broadcasts.
  // No acks here (at least for the moment), using two pipes, 
  //  - one for receiving info from sensors
  //  - one for sending broadcasts 
  
  Radio1.begin ();
  Radio1.setPayloadSize (sizeof (SensorPayloadTemperature_t));
  Radio1.setAutoAck (false);
  Radio1.setPALevel (RF24_PA_HIGH);
  Radio1.setDataRate (RF24_250KBPS);
  Radio1.setChannel (RF24_RADIO_CHANNEL);
  Radio1.openReadingPipe (1, RF24_SENSOR_PIPE);
  Radio1.openWritingPipe (RF24_BROADCAST_PIPE);
#if DEBUG  
  Serial.println ("=================================================");
  Serial.println ("Radio1 details");
  Radio1.printDetails ();
#endif
  Radio1.startListening();

  Radio2.begin ();
  Radio2.setPALevel (RF24_PA_HIGH);
#if DEBUG  
  Serial.println ("=================================================");
  Serial.println ("Radio2 details");
  Radio2.printDetails ();
#endif

  Network.begin (RF24_NETWORK_CHANNEL, 000);

  //
  // Initialize Ethernet network
  //
  
  if (ether.begin (sizeof Ethernet::buffer, mymac, NET_CS) == 0) {
    Text.println ("Failed to access Ethernet controller");
  } else {
    Text.println ("Ethernet controller initialized");
    Flags |= F_ETHERNET;
  }

  if (Flags | F_ETHERNET) {  
    #if STATIC
    ether.staticSetup (myip, gwip);
    #else
    if (!ether.dhcpSetup ()) {
      Text.println ("DHCP failed, using static IP");
      ether.staticSetup (myip, gwip);
    }
    #endif
  
    sprintf (buff, "IP = %d.%d.%d.%d", ether.myip[0], ether.myip[1], ether.myip[2], ether.myip[3]);
    Text.println (buff);
    sprintf (buff, "GW = %d.%d.%d.%d", ether.gwip[0], ether.gwip[1], ether.gwip[2], ether.gwip[3]);
    Text.println (buff);
    sprintf (buff, "DNS = %d.%d.%d.%d", ether.dnsip[0], ether.dnsip[1], ether.dnsip[2], ether.dnsip[3]);
    Text.println (buff);
  }
}

//*******************************************************************************
//*                              Main program loop                              *
//******************************************************************************* 

void loop() {
  SensorPayloadTemperature_t TempPayload;
  uint8_t *p = (uint8_t *) &TempPayload;
  int i;
  static unsigned long LastTime = 0, LastTimeMs = 0;
  static byte LedWhite, LedYellow;
  static char buff[64];
  time_t t;
  
  UpdateTimeNtp ();  
  Network.update ();

  //
  // Any payloads from sensors available?
  //BUGBUG don't assume all sensors send TEMP payload!
  //
  
  if (Radio1.available ()) {
    LedWhite = 255;
    analogWrite (LED_WHITE, LedWhite);
    
    Radio1.read (&TempPayload, sizeof (SensorPayloadTemperature_t));
    #if DEBUG
    sprintf (buff, "Packet %d: %c%c %3d %d", TempPayload.PacketType, (char) (TempPayload.SensorId >> 8), (char) (TempPayload.SensorId & 0xFF), TempPayload.BattLevel, TempPayload.Temperature[0]);
    Debugln (buff);
    #endif
    
    //
    // Search through all the known sensors and see if this one has reported already.
    // If found, copy the new values over the old one.
    // If not found, add the new entry into the array of known sensors
    //
    
    for (i = 0; i < TempSensorsCount; i++) {
      if (TempPayload.SensorId == TempSensors[i].SensorId) {
        break;
      }
    }
    
    memcpy (&TempSensors[i], &TempPayload, sizeof (TempPayload));
    if (i == TempSensorsCount) {
      TempSensorsCount++;
    }
  }

  Network.update ();
  
  //
  // Execute the following code every 5ms. Use the recommended subtracting method to handle millis() overflow
  //

  if ((millis () - LastTimeMs) >= 5) {
    LastTimeMs = millis ();
    
    if (LedWhite > 0) {
      analogWrite (LED_WHITE, --LedWhite);
    }

    if (LedYellow > 0) {
      analogWrite (LED_YELLOW, --LedYellow);
    }
  }

  //
  // Execute the following code every second. Use the recommended subtracting method to handle millis() overflow
  //

  if ((millis () - LastTime) >= 1000) {
    LastTime = millis ();

    LedYellow = 127;
    analogWrite (LED_YELLOW, LedYellow);
  
    t = now ();
    sprintf (buff, "%d:%2.2d:%2.2d  ", hour (t), minute (t), second (t));
    // Text.cursorToXY (0, 70);
    // Text.println (buff);
    Text.drawString (buff, 0, 70);

    Text.cursorToXY (0, 85);
    for (i = 0; i < TempSensorsCount; i++) {
      sprintf (buff, "%c%c %4dmV %5.1fC", (char) (TempSensors[i].SensorId >> 8), (char) (TempSensors[i].SensorId & 0xFF), TempSensors[i].BattLevel * 10 + 2000, TempSensors[i].Temperature[0] / 10.0);
      Text.println (buff);
    }
  }
} 
