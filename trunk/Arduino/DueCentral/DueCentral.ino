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

//*******************************************************************************
//*                           Includes and defines                              *
//******************************************************************************* 

#define VERSION 0 

#include <ILI9341_due_gText.h>
#include <ILI9341_due.h>
#include <EtherCard.h>
#include <SPI.h>
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

#define STATIC 0  // set to 1 to disable DHCP (adjust myip/gwip values below)
#define DEBUG 1

#define MAX_TEMP_SENSORS 10

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

const char page[] PROGMEM =
"HTTP/1.0 503 Service Unavailable\r\n"
"Content-Type: text/html\r\n"
"Retry-After: 600\r\n"
"\r\n"
"<html>"
  "<head><title>"
    "Service Temporarily Unavailable"
  "</title></head>"
  "<body>"
    "<h3>This service is currently unavailable</h3>"
    "<p><em>"
      "The main server is currently off-line.<br />"
      "Please try again later."
    "</em></p>"
  "</body>"
"</html>"
; 

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
  // Initialize serial port and send info
  //
  
  Serial.begin (57600);
  while (!Serial); 
  Serial.println ("[Arduino Due Central Node!]"); 

  //
  //  Initialize the display, set small text and red color for status messages
  //
  
  tft.begin ();
  tft.fillScreen (ILI9341_BLACK);
  tft.setCursor (0, 0);
  tft.setTextColor (ILI9341_RED);  
  tft.setTextSize (1);

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
    tft.println ("Failed to access Ethernet controller");
  } else {
    tft.println ("Ethernet controller initialized");
    Flags |= F_ETHERNET;
  }

  if (Flags | F_ETHERNET) {  
    #if STATIC
    ether.staticSetup (myip, gwip);
    #else
    if (!ether.dhcpSetup ()) {
      tft.println ("DHCP failed, using static IP");
      ether.staticSetup (myip, gwip);
    }
    #endif
  
    sprintf (buff, "IP = %d.%d.%d.%d", ether.myip[0], ether.myip[1], ether.myip[2], ether.myip[3]);
    tft.println (buff);
    sprintf (buff, "GW = %d.%d.%d.%d", ether.gwip[0], ether.gwip[1], ether.gwip[2], ether.gwip[3]);
    tft.println (buff);
    sprintf (buff, "DNS = %d.%d.%d.%d", ether.dnsip[0], ether.dnsip[1], ether.dnsip[2], ether.dnsip[3]);
    tft.println (buff);
  }
}

//*******************************************************************************
//*                              Main program loop                              *
//******************************************************************************* 

void loop() {
  SensorPayloadTemperature_t TempPayload;
  uint8_t *p = (uint8_t *) &TempPayload;
  int i;
  static unsigned long LastTime = 0;
  static char buff[64];
  
  Network.update ();

  if (Flags | F_ETHERNET) {
    // wait for an incoming TCP packet, but ignore its contents
    if (ether.packetLoop (ether.packetReceive())) {
      memcpy_P (ether.tcpOffset(), page, sizeof page);
      ether.httpServerReply (sizeof page - 1);
    }
  }
  
  Network.update ();

  //
  // Any payloads from sensors available?
  //BUGBUG don't assume all sensors send TEMP payload!
  //
  
  if (Radio1.available ()) {
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
  // Execute the following code every second. Use the recommended subtracting method to handle millis() overflow
  //

  if ((millis () - LastTime) >= 1000) {
    LastTime = millis ();
//    tft.fillScreen (ILI9341_BLACK);

    tft.setCursor (0, 0);
    tft.setTextColor (ILI9341_WHITE);  
    tft.setTextSize (2);
    
    for (i = 0; i < TempSensorsCount; i++) {
      sprintf (buff, "%c%c %3d %d", (char) (TempSensors[i].SensorId >> 8), (char) (TempSensors[i].SensorId & 0xFF), TempSensors[i].BattLevel, TempSensors[i].Temperature[1]);
      tft.println (buff);
    }
  }
} 
