#include <Time.h>

#include <EtherCard.h>
#define TIME_UPDATE_PERIOD 30
#define ETHERNET_BUFFER_SIZE 550

#include <JeeLib.h>
#include <PacketDefine.h>

#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>

//
// Nokia display variables
//

Adafruit_PCD8544 display = Adafruit_PCD8544(18, 19, 20, 22, 21);
const byte PinBacklight = 15;

byte LcdFlags = 0;
#define LCD_NEEDS_UPDATE 0x01
#define LCD_NEEDS_CLS 0x02

//
// EtherCard variables
//

static uint8_t mymac[6] = { 0x54,0x55,0x58,0x10,0x00,0x25};
static uint8_t myip[4] = { 0,0,0,0 };      // IP and netmask allocated by DHCP
static uint8_t mynetmask[4] = { 0,0,0,0 };
static uint8_t gwip[4] = { 0,0,0,0 };
static uint8_t dnsip[4] = { 0,0,0,0 };
static uint8_t dhcpsvrip[4] = { 0,0,0,0 };

// Packet buffer, must be big enough to packet and payload
byte Ethernet::buffer[ETHERNET_BUFFER_SIZE];

//
// ShiftPWM support
//

#define SHIFTPWM_NOSPI        // Don't use the SPI port
const int ShiftPWM_latchPin = 24;
const int ShiftPWM_dataPin = 26;
const int ShiftPWM_clockPin = 25;

const bool ShiftPWM_invertOutputs = true; // LED's turn on if the pin is low
const bool ShiftPWM_balanceLoad = false;

#include <ShiftPWM.h>   // include ShiftPWM.h after setting the pins!

//
// Here you set the number of brightness levels, the update frequency and the number of shift registers.
// These values affect the load of ShiftPWM.
// Choose them wisely and use the PrintInterruptLoad() function to verify your load.
//

unsigned char maxBrightness = 15;
//unsigned char pwmFrequency = 75;
int numRegisters = 1;
int numOutputs = numRegisters*8;
int numRGBLeds = numRegisters*8/3;
int fadingMode = 0; //start with all LED's off.

//unsigned long startTime = 0; // start time for the chosen fading mode

//
// HBus definitions and variables
//

#include "typedefs.h"
#include "hbus.h"
#include "hbus_cfg.h"

//
// Other global variables
//

int InTemp = 0;
int OutTemp = 0;

//================================================
// Packet decode
//

void PacketDecode () {
  TemperaturePayload_t *Temp;

  switch (*rf12_data) {
    
    case RF12_PACKET_TIME:
      // Ignore, we do not need it as we use NTP
      break;
      
    case RF12_PACKET_METEO:
      // No active sender yet
      break;
      
    case RF12_PACKET_TEMPERATURE:
      Temp = (TemperaturePayload_t *) rf12_data;
      InTemp = Temp->temp1;
      OutTemp = Temp->temp2;
      break;
      
    default:
      break;
  }
}

//================================================
// Tasks
//

//
// Time task.
// Time task uses third display line
//

static const char DayStrings[] PROGMEM = "NePoUtStCtPaSo";

byte TimeTask (void) {
  time_t t;
  static time_t LastTime = 0;    // Init to invalid value to make sure time will be updated after start
  byte Weekday, i;
  
  t = now ();
  if (LastTime != t) { 
    LastTime = t;
    display.fillRect (0, 16, 84, 8, WHITE);
    display.setCursor(0, 16);
    display.print(hour (t));
    display.print(":");
    display.print(minute (t));
    display.print(":");
    display.print(second (t));
    display.print("  ");

    Weekday = weekday (t) - 1;    // 1 is Sunday, but our array of strings starts at 0
    for (i = 0; i < 2; i++) {
      display.write (pgm_read_byte(&(DayStrings[Weekday * 2 + i])));
    }
    
    return 1;
  } else {
    return 0;
  }
}

//
// HBUS task.
// HBUS uses first display line
//

byte HbusTask (void) {
  static uint16_t counter = 0;
  
  if ((millis () % 10000) == 0) {
    send_buff[0] = MSG_BROADCAST;
    send_buff[1] = MSG_SET_7SEG;
    send_buff[2] = (byte) ~ 0xEE;
    send_buff[3] = (byte) ~ 0x2E;
    send_buff[4] = (byte) ~ 0x3A;
    send_buff[5] = (byte) ~ 0x78;
    send_buff[6] = 1;             // 3.0 seconds
    send_message(7);
    display.fillRect (0, 0, 84, 8, WHITE);
    display.setCursor(0,0);
    display.print("HBUS  #");
    display.print(counter++);
    return 1;
  }
  return 0;
}

//
// RFM12 task
// RFM12 uses second display line
//

byte Rfm12Task (void) {
  static uint16_t counter = 0;

  if (rf12_recvDone() && rf12_crc == 0) {
    ShiftPWM.SetOne (3, 31);
    PacketDecode ();
    display.fillRect (0, 8, 84, 8, WHITE);
    display.setCursor(0, 8);
    display.print("RFM12 #");
    display.print(counter++);
//    ShiftPWM.SetOne (3, 0);
    return 1;
  } 
  return 0;
}

//
// Temperature task
// Temperature uses fourth display line
//

void DisplayTemp (signed int Temp) {
  byte TempSign;
  
  if (Temp < 0) {
    display.print("-");
    Temp = -Temp;
  }
 
  Temp = (Temp + 8) / 16;
  display.print(Temp);  
}      

byte TemperatureTask (void) {
  static int LastInTemp = 32767;
  static int LastOutTemp = 32767;

  if ((LastInTemp != InTemp) || (LastOutTemp != OutTemp)) {
    LastInTemp = InTemp;
    LastOutTemp = OutTemp;
    display.fillRect (0, 24, 84, 8, WHITE);
    display.setCursor(0, 24);
    DisplayTemp(InTemp);
    display.print("oC  ");
    DisplayTemp(OutTemp);
    display.print("oC");
    return 1;
  } 
  return 0;
}

//================================================
// Main code - setup and loop
//

void setup () {
  
  Serial.begin (57600);     // Opens serial port used for debugging
  delay (1000);

  //
  // Configure ShiftPWM
  //
  
  ShiftPWM.SetAmountOfRegisters (numRegisters);
  ShiftPWM.Start (75, maxBrightness);
  ShiftPWM.PrintInterruptLoad ();
  ShiftPWM.SetAll(0);

  //
  // Configure Nokia display
  //
  
  display.begin();
  display.setContrast(60);    // 60 seems to be a good value, original value of 50 is too low
  display.clearDisplay();   // clears the screen and buffer
  
  pinMode (PinBacklight, OUTPUT);    // Configure display backlight brightness
  analogWrite (PinBacklight, 255);
  
  //
  // Display intro message
  //
  
  display.setTextSize(2);
  display.setTextColor(BLACK);
  display.setCursor(0,0);
  display.print("Central");
  display.setCursor(18,16);
  display.print("Unit");
  display.setTextSize(1);
  display.setCursor(3,32);
  display.print("Arduino based");
  display.setCursor(3,40);
  display.print("(c)2013 Kubik");
  display.display();
  
  // 
  // Configure RS-485 - configure driver pin, disable driver (set to RECEIVE) and configure Serial1 speed
  // Configure RS485 LEDs and turn them off
  //
  
  Rs485DisableDriver ();
  pinMode (Rs485DirectionPin, OUTPUT);
  Serial1.begin (RS485_BAUD);     // Opens serial port used for RS485, sets data rate
  
//  pinMode (Rs485RxLedPin, OUTPUT);
//  pinMode (Rs485TxLedPin, OUTPUT);
//  
//  Rs485RxLedOff ();
//  Rs485TxLedOff ();

  //
  // Configure RFM12
  //
  
  rf12_initialize(RF12_NET_NODE_CPU, RF12_868MHZ, RF12_NET_GROUP);
  
  //
  // EtherCard init
  //
  
  uint8_t rev = ether.begin(sizeof Ethernet::buffer, mymac, 4);
  Serial.print( F("\nENC28J60 Revision ") );
  Serial.println( rev, DEC );
  if ( rev == 0)
    Serial.println( F( "Failed to access Ethernet controller" ) );

  Serial.println( F( "Setting up DHCP" ));
  if (!ether.dhcpSetup())
    Serial.println( F( "DHCP failed" ));
  
  ether.printIp("My IP: ", ether.myip);
  ether.printIp("Netmask: ", ether.mymask);
  ether.printIp("GW IP: ", ether.gwip);
  ether.printIp("DNS IP: ", ether.dnsip);

  //
  // Time init - set default value
  //
  
  setTime (0, 0, 0, 1, 1, 2013);  // Set time to dummy value before ntp will catch up  
  
  display.clearDisplay ();        // Clear display buffer but does not display it yet -> 
                                  // init screen will remain until the main loop really writes someting
  DnsLookup ();                                  
}

void loop() {
  byte LcdNeedsRedraw;
  static unsigned long int LastTime = 0;
  
  //
  // Following variable gets ORed with return value of all tasks, and if any of them requests screen redraw, 
  // it will be done so at the end of the loop
  //
  
  LcdNeedsRedraw = 0;
  
  //
  // Process all the tasks
  //
  
  UpdateTimeNtp ();                  // Process NTP time sync now and then
  LcdNeedsRedraw |= HbusTask ();     // Send any scheduled HBUS messages
  LcdNeedsRedraw |= Rfm12Task ();    // Check for any incoming RFM12 packets and process them
  LcdNeedsRedraw |= TimeTask ();     // Display time if it has changed
  LcdNeedsRedraw |= TemperatureTask ();     // Display temperature if it has changed

  if (second () & 0x01) {
    ShiftPWM.SetOne (1, 3);
  } else {
    ShiftPWM.SetOne (1, 0);
  }    
  
  //
  // End of the loop, do all the housekeeping
  //
  
  if (LcdNeedsRedraw) {
    display.display();
  }
  
  if (LastTime != (millis () >> 2)) {
    LastTime = millis () >> 2;
   
    if (ShiftPWM.m_PWMValues[2] > 0) {
      ShiftPWM.m_PWMValues[2]--;
    }
      
    if (ShiftPWM.m_PWMValues[3] > 0) {
      ShiftPWM.m_PWMValues[3]--;
    }
  }
  
}

