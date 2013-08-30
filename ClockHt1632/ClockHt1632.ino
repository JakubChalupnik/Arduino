//*******************************************************************************
//*
//* Arduino based matrix clock with Ethernet and NRF24L01
//*
//*******************************************************************************
//* Processor:  Arduino Pro Mini
//* Library:    http://code.google.com/p/ht1632c
//*             https://github.com/jcw/ethercard
//* Author      Date       Comment
//*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//* Kubik       30.8.2013 First release, testing the HW
//* Kubik       30.8.2013 Ethernet support added (ENC26J80)
//*******************************************************************************

//*******************************************************************************
//*                            HW details                                       *
//*******************************************************************************
// Using 32x16 bicolor display from SeeedStudio (?)
// CS = 4, CLK = 5, WR = 6, Data = 7
//
// Note - the ht1632c library includes various fonts by default.
// To reduce the code size, some of the fonts can be disabled in ht1632c.h
//
// Using ENC28J60 from eBay for cca 3USD
// SO = 12, SI = 11, SCK = 13, CS = 8
//

//*******************************************************************************
//*                           Includes and defines                              *
//*******************************************************************************

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <ht1632c.h>
#include <EtherCard.h>

#define F_ETH_OK  0x01

//*******************************************************************************
//*                               Static variables                              *
//*******************************************************************************

ht1632c Display = ht1632c (&PORTD, 7, 6, 5, 4, GEOM_32x16, 2);
static byte mymac[] = {                     // ethernet mac address - must be unique on your network
  0x74,0x69,0x69,0x2D,0x30,0x32 };   
byte Ethernet::buffer[500];                 // tcp/ip send and receive buffer

volatile byte Flags = 0;

//*******************************************************************************
//*                               HTTP page code                                *
//*******************************************************************************

char page[] PROGMEM =
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

//*******************************************************************************
//*                               Display code                                  *
//*******************************************************************************

void DisplayPrint (char *s, byte x, byte y, byte Step, byte Color) {
     
  while (*s != '\0') {
    Display.putchar (x,  y, *s++, Color);
    x += Step;
  }
}

//*******************************************************************************
//*                            Arduino setup method                             *
//*******************************************************************************

void setup () {

  Serial.begin (57600);
  Serial.println (F("[HTCLK]"));
  Display.clear ();
  Display.pwm (1);

  Display.setfont (FONT_5x7W);
  DisplayPrint ("HTLCK", 1, 0, 6, RED);
  Display.sendframe ();
  
  Display.setfont (FONT_4x6);
  if (ether.begin (sizeof Ethernet::buffer, mymac) == 0) {
    Serial.println ("Failed to access Ethernet controller");
    DisplayPrint ("ETH fail", 0, 10, 4, RED);
    Display.sendframe ();
  }

  if (ether.dhcpSetup ()) {
    Flags |= F_ETH_OK;
    DisplayPrint ("ETH OK", 4, 10, 4, RED);
  } else {
    Serial.println ("DHCP failed");
    DisplayPrint ("DHCPfail", 0, 10, 4, RED);
  }
  Display.sendframe ();
}

//*******************************************************************************
//*                              Main program loop                              *
//******************************************************************************* 

void loop () {

  if (ether.packetLoop (ether.packetReceive ())) {   // wait for an incoming TCP packet, but ignore its contents
    memcpy_P(ether.tcpOffset(), page, sizeof page);
    ether.httpServerReply(sizeof page - 1);
  }

}

