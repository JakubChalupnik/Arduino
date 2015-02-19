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

#define TFT_DC 51
#define TFT_CS 52
#define TFT_RESET 53

#define NET_RST  47
#define NET_CS   46

#define STATIC 0  // set to 1 to disable DHCP (adjust myip/gwip values below)
#define DEBUG

//*******************************************************************************
//*                               Static variables                              *
//******************************************************************************* 

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

static byte myip[] = {192, 168, 10, 200};
static byte gwip[] = {192, 168, 10, 1};

static byte mymac[] = { 0x74,0x69,0x69,0x33,0x30,0x31 };

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
// Globally available and used flags
//

uint16_t Flags = 0;
#define F_ETHERNET 0x0001    // Ethernet controller operational

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
  // Initialize network
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

void loop(){
  
  if (Flags | F_ETHERNET) {
    // wait for an incoming TCP packet, but ignore its contents
    if (ether.packetLoop(ether.packetReceive())) {
      memcpy_P(ether.tcpOffset(), page, sizeof page);
      ether.httpServerReply(sizeof page - 1);
    }
  }
} 
