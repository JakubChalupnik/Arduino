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
//* Kubik       30.8.2013 Time support added
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
#include <Time.h>
#include <Timezone.h>    // https://github.com/JChristensen/Timezone 

#define ETHERNET_BUFFER_SIZE 550
#define TIME_UPDATE_PERIOD 30

//
// Debug options
//

#define DEBUG_ETHERNET 0

//
// Flags 
//

#define F_ETH_OK  0x01
#define F_TIME_UPDATED 0x02

//
// Ethernet related defines
//

#define EthernetLedOn()
#define EthernetLedOff()

#if DEBUG_ETHERNET
  #define DebugEthernet(...) Serial.print(__VA_ARGS__)
  #define DebugEthernetln(...) Serial.println(__VA_ARGS__)
#else
  #define DebugEthernet(...)
  #define DebugEthernetln(...)
#endif


//*******************************************************************************
//*                               Static variables                              *
//*******************************************************************************

ht1632c Display = ht1632c (&PORTD, 7, 6, 5, 4, GEOM_32x16, 2);
static byte mymac[] = {                     // ethernet mac address - must be unique on your network
  0x74,0x69,0x69,0x2D,0x30,0x32 };   
byte Ethernet::buffer[ETHERNET_BUFFER_SIZE];  // tcp/ip send and receive buffer

volatile byte Flags = 0;

//Central European Time (Frankfurt, Paris)
TimeChangeRule CEST = {"CEST", Last, Sun, Mar, 2, 120};     //Central European Summer Time
TimeChangeRule CET = {"CET ", Last, Sun, Oct, 3, 60};       //Central European Standard Time
Timezone CE(CEST, CET); 

//*******************************************************************************
//*                               HTTP page code                                *
//*******************************************************************************

char PageHeader [] PROGMEM =
"<html>"
  "<head><title>"
    "Arduino HT-CLK"
  "</title></head>"
  "<body>"
    "<h3>Arduino HT-CLK</h3>"
;

char PageFooter [] PROGMEM =
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

void DisplayTime (byte Color = RED) {
  
  if (timeStatus () == timeNotSet) {
    Display.setfont(FONT_7x13);
    Display.putchar(1,  -2, '-', Color);
    Display.putchar(8,  -2, '-', Color);
    Display.putchar(18,  -2, '-', Color);
    Display.putchar(25,  -2, '-', Color);
  } else {
    Display.setfont(FONT_5x7W);
    Display.putchar(14,  1, ':', Color);
    Display.setfont(FONT_7x13);
    if (hour () > 9) Display.putchar(1,  -2, hour () / 10 + '0', Color);
    Display.putchar(8,  -2, hour () % 10 + '0', Color);
    Display.putchar(18,  -2, minute () / 10 + '0', Color);
    Display.putchar(25,  -2, minute () % 10 + '0', Color);    
  }
  Display.sendframe ();
}

//*******************************************************************************
//*                            Arduino setup method                             *
//*******************************************************************************

void setup () {

  Serial.begin (57600);
  Serial.println (F("[ClockHt1632]"));
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

  DnsLookup ();
  Display.clear ();
}

//*******************************************************************************
//*                              Main program loop                              *
//******************************************************************************* 

void loop () {
  static byte LastSecond = 61;       // Remember last value of second() to toss tle flag up when it changes

  if (ether.packetLoop (ether.packetReceive ())) {   // wait for an incoming TCP packet, but ignore its contents
    memcpy_P(ether.tcpOffset(), PageHeader, sizeof (PageHeader));
    memcpy_P(ether.tcpOffset() + sizeof (PageHeader), PageFooter, sizeof (PageFooter));
    ether.httpServerReply(sizeof (PageHeader) + sizeof (PageFooter) - 1);
  }

  UpdateTimeNtp ();                         // Process NTP time sync now and then

  //
  // Did the second just change? If not, leave, and don't execute the rest
  //

  if (LastSecond != second ()) {
    LastSecond = second ();
  } else {
    return;
  }

  DisplayTime ();
}

