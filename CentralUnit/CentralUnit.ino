#include <OneWire.h>
#include <Time.h>

#include <EtherCard.h>
#define TIME_UPDATE_PERIOD 30
#define ETHERNET_BUFFER_SIZE 550

#include <JeeLib.h>
#include <PacketDefine.h>

#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>

//
// Debug options
//

#define DEBUG_OW_TEMP 1
#define DEBUG_ETHERNET 0

//
// OneWire support
//

#define OW_PIN_INT 27
OneWire OneWireInternal(OW_PIN_INT);
#if DEBUG_OW_TEMP
  #define DebugOwTemp(...) Serial.print(__VA_ARGS__)
  #define DebugOwTempln(...) Serial.println(__VA_ARGS__)
#else
  #define DebugOwTemp(...)
  #define DebugOwTempln(...)
#endif

//
// Nokia display variables
//

Adafruit_PCD8544 display = Adafruit_PCD8544(18, 19, 20, 22, 21);
const byte PinLcdShiftPwm = 15;    // The LED backlight is odd, does not quite work as expected

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

#if DEBUG_ETHERNET
  #define DebugEthernet(...) Serial.print(__VA_ARGS__)
  #define DebugEthernetln(...) Serial.println(__VA_ARGS__)
#else
  #define DebugEthernet(...)
  #define DebugEthernetln(...)
#endif

// Packet buffer, must be big enough to packet and payload
byte Ethernet::buffer[ETHERNET_BUFFER_SIZE];

//
// ShiftPWM support
//

#define SHIFTPWM_NOSPI        // Don't use the SPI port
const int ShiftPWM_latchPin = 24;
const int ShiftPWM_dataPin = 26;
const int ShiftPWM_clockPin = 25;

const bool ShiftPWM_invertOutputs = false; // LED's turn on if the pin is high
const bool ShiftPWM_balanceLoad = false;

#include <ShiftPWM.h>   // include ShiftPWM.h after setting the pins!

//
// Here you set the number of brightness levels, the update frequency and the number of shift registers.
// These values affect the load of ShiftPWM.
// Choose them wisely and use the PrintInterruptLoad() function to verify your load.
//

unsigned char maxBrightness = 15;
int numRegisters = 1;
int numOutputs = numRegisters*8;
int numRGBLeds = numRegisters*8/3;
int fadingMode = 0; //start with all LED's off.

//#define LED_l 7
//#define LED_2 6
//#define LED_3 5
//#define LED_4 4

#define LED_ETHERNET  7
#define LED_HEART     6
#define LED_RFM12     5
#define LED_HBUS      4

#define LED_OFF 0
#define LED_ON 1 //maxBrightness

//
// HBus definitions and variables
//

#include "typedefs.h"
#include "hbus.h"
#include "hbus_cfg.h"

//
// Analog support (light intensity)
//

const byte PinLightIntensity = A6;    // Analog pin 6
byte LightIntensity;                  // cca 218 for darkness, close to 0 for bright light

//
// Other global variables
//

int InTemp = 0;
int OutTemp = 0;
byte Flags = 0;
#define F_TIME_UPDATED 0x01
#define F_TIME_CHANGED 0x02
TimePayload_t TimePayload;

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
// 7 segment support (for HBUS devices)
//

const unsigned char SegmentCharactersTable [] PROGMEM = {
//  ABCDEFG
  ~B11111100, // zero
  ~B01100000, // one
  ~B11011010, // two
  ~B11110010, // three
  ~B01100110, // four
  ~B10110110, // five
  ~B10111110, // six
  ~B11100000, // seven
  ~B11111110, // eight
  ~B11110110, // nine
  ~B00000000, // off       10
  ~B11000110, // degree    11
  ~B10011100, // C         12
  ~B00000010, // dash      13
};

#define SegChar(c) pgm_read_byte(SegmentCharactersTable + c)
#define SegCharC pgm_read_byte(SegmentCharactersTable + 12)
#define SegCharDegree pgm_read_byte(SegmentCharactersTable + 11)
#define SegCharDash pgm_read_byte(SegmentCharactersTable + 13)
#define SegCharBlank pgm_read_byte(SegmentCharactersTable + 10)

//
// Following function takes temperature and creates corresponding
// LED encoded text in the four byte buffer
//

void SegConvertTemp (int Temp, byte *OutBuff) {

  if (Temp >= 0) {                  // Positive temperatures are displayed differently than negative ones
    Temp = (Temp + 8) >> 4;
    OutBuff [0] = (Temp >= 10) ? SegChar (Temp / 10) : SegCharBlank;
    OutBuff [1] = SegChar (Temp % 10);
    OutBuff [2] = SegCharDegree;
    OutBuff [3] = SegCharC;
  } else {                          // Negative temperatures have leading '-' and omit the 'C'
    Temp = (-Temp + 8) >> 4;
    OutBuff [0] = SegCharDash;
    OutBuff [1] = (Temp >= 10) ? SegChar (Temp / 10) : SegCharBlank;
    OutBuff [2] = SegChar (Temp % 10);
    OutBuff [3] = SegCharDegree;
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

  if (Flags & F_TIME_CHANGED) {
    if ((second () == 0) && ((minute () % 5) == 0) && (timeStatus() == timeSet)) {
      send_buff[0] = MSG_BROADCAST;
      send_buff[1] = MSG_SET_CLOCK;
      send_buff[2] = month ();
      send_buff[3] = day ();
      send_buff[4] = hour ();
      send_buff[5] = minute ();
      send_buff[6] = 0;
      send_message(7);
//      send_buff[0] = MSG_BROADCAST;
//      send_buff[1] = MSG_SET_7SEG;
//      send_buff[2] = (byte) ~ 0xEE;
//      send_buff[3] = (byte) ~ 0x2E;
//      send_buff[4] = (byte) ~ 0x3A;
//      send_buff[5] = (byte) ~ 0x78;
//      send_buff[6] = 10;             // 1.0 seconds
//      send_message(7);
      display.fillRect (0, 0, 84, 8, WHITE);
      display.setCursor(0,0);
      display.print("HBUS time #");
      display.print(counter++);
      return 1;
    }

#ifdef HBUS_SET_LEDS
    send_buff[0] = MSG_BROADCAST;
    send_buff[1] = MSG_SET_LEDS;
    if ((second () & 0x01) == 0) {
      send_buff[2] = 0;
    } else {
      send_buff[2] = 0xFF;
    }
    send_message(3);
#endif
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
    ShiftPWM.SetOne (LED_RFM12, LED_ON);
    PacketDecode ();
    display.fillRect (0, 8, 84, 8, WHITE);
    display.setCursor(0, 8);
    display.print("RFM12 #");
    display.print(counter++);
    ShiftPWM.SetOne (LED_RFM12, LED_OFF);
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

//
// Light intensity task
//

byte LightIntensityTask (void) {
  static unsigned int Average = 0;
  
  Average = ((Average * 31) + (analogRead(PinLightIntensity) >> 2)) / 32; 
  
  LightIntensity = Average;
//  return 0;
  display.fillRect (0, 32, 84, 8, WHITE);
  display.setCursor(0, 32);
  display.print(Average);
  display.print("     ");
  return 1;
}

byte LcdBacklightTask (void) {

  analogWrite (PinLcdShiftPwm, LightIntensity + 30);
  return 0;
}
  
//
// Temperature sensor variables
//

typedef enum {
  S_IDLE, S_WAIT, S_READ, S_COMP, S_IDLE2, S_WAIT2, S_READ2, S_COMP2}
state_t;

byte type_s1;
byte data1[12];
byte addr1[8];
byte type_s2;
byte data2[12];
byte addr2[8];

TemperaturePayload_t TempPayload;

//
// Temperature sensor support
//

void TempPoll(void) {
  byte i;
  byte present = 0;
  unsigned int raw;
  static state_t State = S_IDLE;
  static uint32_t WaitTime;

  switch (State) {
  case S_IDLE:
    OneWireInternal.reset();
    OneWireInternal.select(addr1);
    OneWireInternal.write(0x44,1);         // start conversion, with parasite power on at the end
    WaitTime = millis () + 1000;
    State = S_WAIT;
    break;

  case S_WAIT:
    if (millis () > WaitTime) {
      State = S_READ;
    }
    break;

  case S_READ:
    present = OneWireInternal.reset();
    OneWireInternal.select(addr1);
    OneWireInternal.write(0xBE);         // Read Scratchpad
    State = S_COMP;
    break;

  case S_COMP:
    for ( i = 0; i < 9; i++) {           // we need 9 bytes
      data1[i] = OneWireInternal.read();
    }

    // convert the data to actual temperature

    raw = (data1[1] << 8) | data1[0];
    if (type_s1) {
      raw = raw << 3; // 9 bit resolution default
      if (data1[7] == 0x10) {
        // count remain gives full 12 bit resolution
        raw = (raw & 0xFFF0) + 12 - data1[6];
      }
    }
    else {
      byte cfg = (data1[4] & 0x60);
      if (cfg == 0x00) raw = raw << 3;  // 9 bit resolution, 93.75 ms
      else if (cfg == 0x20) raw = raw << 2; // 10 bit res, 187.5 ms
      else if (cfg == 0x40) raw = raw << 1; // 11 bit res, 375 ms
      // default is 12 bit resolution, 750 ms conversion time
    }

    DebugOwTemp("Temp ");
    DebugOwTempln(raw, HEX);
    InTemp = raw;
    TempPayload.temp1 = raw;
    State = S_IDLE2;
    break;

  case S_IDLE2:
    OneWireInternal.reset();
    OneWireInternal.select(addr2);
    OneWireInternal.write(0x44,1);         // start conversion, with parasite power on at the end
    WaitTime = millis () + 1000;
    State = S_WAIT2;
    break;

  case S_WAIT2:
    if (millis () > WaitTime) {
      State = S_READ2;
    }
    break;

  case S_READ2:
    present = OneWireInternal.reset();
    OneWireInternal.select(addr2);
    OneWireInternal.write(0xBE);         // Read Scratchpad
    State = S_COMP2;
    break;

  case S_COMP2:
    for ( i = 0; i < 9; i++) {           // we need 9 bytes
      data2[i] = OneWireInternal.read();
    }

    // convert the data to actual temperature

    raw = (data2[1] << 8) | data2[0];
    if (type_s2) {
      raw = raw << 3; // 9 bit resolution default
      if (data2[7] == 0x10) {
        // count remain gives full 12 bit resolution
        raw = (raw & 0xFFF0) + 12 - data2[6];
      }
    }
    else {
      byte cfg = (data2[4] & 0x60);
      if (cfg == 0x00) raw = raw << 3;  // 9 bit resolution, 93.75 ms
      else if (cfg == 0x20) raw = raw << 2; // 10 bit res, 187.5 ms
      else if (cfg == 0x40) raw = raw << 1; // 11 bit res, 375 ms
      // default is 12 bit resolution, 750 ms conversion time
    }

    DebugOwTemp("Temp2 ");
    DebugOwTempln(raw, HEX);
    OutTemp = raw;
    TempPayload.temp2 = raw;
    State = S_IDLE;
    break;

  default:
    State = S_IDLE;
    break;
  }
}

void OwInitTemp (void) {
  byte i;
  
  //
  // Initialise the first temperature sensor
  //

  if ( !OneWireInternal.search(addr1)) {
    DebugOwTempln(F("No more addresses."));
    OneWireInternal.reset_search();
    delay(250);
  }

#if DEBUG_OW_TEMP
  DebugOwTemp(F("ROM ="));
  for( i = 0; i < 8; i++) {
    DebugOwTemp(" ");
    DebugOwTemp(addr1[i], HEX);
  }
#endif

  if (OneWire::crc8(addr1, 7) != addr1[7]) {
    DebugOwTempln(F("CRC is not valid!"));
    return;
  }
  DebugOwTempln();

  // the first ROM byte indicates which chip
  switch (addr1[0]) {
  case 0x10:
    DebugOwTempln(F("  Chip = DS18S20"));  // or old DS1820
    type_s1 = 1;
    break;
  case 0x28:
    DebugOwTempln(F("  Chip = DS18B20"));
    type_s1 = 0;
    break;
  case 0x22:
    DebugOwTempln(F("  Chip = DS1822"));
    type_s1 = 0;
    break;
  default:
    DebugOwTempln(F("Device is not a DS18x20 family device."));
    return;
  }

  //
  // Initialise the second temperature sensor
  //

  if ( !OneWireInternal.search(addr2)) {
    DebugOwTempln(F("No more addresses."));
    OneWireInternal.reset_search();
    delay(250);
  }

#if DEBUG_OW_TEMP
  DebugOwTemp(F("ROM ="));
  for( i = 0; i < 8; i++) {
    DebugOwTemp(" ");
    DebugOwTemp(addr2[i], HEX);
  }
#endif

  if (OneWire::crc8(addr2, 7) != addr2[7]) {
    DebugOwTempln(F("CRC is not valid!"));
    return;
  }
  DebugOwTempln();

  // the first ROM byte indicates which chip
  switch (addr2[0]) {
  case 0x10:
    DebugOwTempln(F("  Chip = DS18S20"));  // or old DS1820
    type_s2 = 1;
    break;
  case 0x28:
    DebugOwTempln(F("  Chip = DS18B20"));
    type_s2 = 0;
    break;
  case 0x22:
    DebugOwTempln(F("  Chip = DS1822"));
    type_s2 = 0;
    break;
  default:
    DebugOwTempln(F("Device is not a DS18x20 family device."));
    return;
  }

  TempPayload.type = RF12_PACKET_TEMPERATURE;
}

//===============================================================================================
// Main code - setup
//

void setup () {

  Serial.begin (57600);     // Opens serial port used for debugging
  delay (1000);

  //
  // Configure ShiftPWM
  //

  ShiftPWM.SetAmountOfRegisters (numRegisters);
  ShiftPWM.Start (75, maxBrightness);
  ShiftPWM.SetAll(LED_OFF);

  //
  // Configure Nokia display
  //

  display.begin();
  display.setContrast(60);    // 60 seems to be a good value, original value of 50 is too low
  display.clearDisplay();   // clears the screen and buffer

  pinMode (PinLcdShiftPwm, OUTPUT);
  analogWrite (PinLcdShiftPwm, 128);
  
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
  
  ShiftPWM.SetOne (LED_ETHERNET, LED_ON);
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

  DnsLookup ();
  ShiftPWM.SetOne (LED_ETHERNET, LED_OFF);
  
  //
  // Time init - set default value
  //
  
//  setTime (0, 0, 0, 1, 1, 2013);  // Set time to dummy value before ntp will catch up  
  
  display.clearDisplay ();        // Clear display buffer but does not display it yet -> 
                                  // init screen will remain until the main loop really writes someting
  //
  // Initialize OneWire temperature sensors
  //

  OwInitTemp ();

  TimePayload.type = RF12_PACKET_TIME;
  
  //
  // Speed up the ADC, we don't care about precise values but speed
  //
  
  // Define various ADC prescaler
  #define PS_16  ((1 << ADPS2))
  #define PS_32  ((1 << ADPS2) | (1 << ADPS0))
  #define PS_64  ((1 << ADPS2) | (1 << ADPS1))
  #define PS_128 ((1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0))

  ADCSRA &= ~PS_128;  // remove bits set by Arduino library
  ADCSRA |= PS_16;    // // choose a prescaler from above
}

//===============================================================================================
// Main code - loop
//

void loop() {
  byte LcdNeedsRedraw;
  static unsigned long int LastTime = 0;
  static unsigned long int LastTempPacketTime = 0;
  MilliTimer wait;                   // radio needs some time to power up
  static byte LastSecond = 61;       // Remember last value of second() to toss tle flag up when it changes 

  //
  // Did the second just change?
  //
  
  if (LastSecond != second ()) {
    Flags |= F_TIME_CHANGED;
    LastSecond = second ();
  }
  
  //
  // Following variable gets ORed with return value of all tasks, and if any of them requests screen redraw,
  // it will be done so at the end of the loop
  //

  LcdNeedsRedraw = 0;

  //
  // Process all the tasks
  //

  TempPoll ();

  UpdateTimeNtp ();                         // Process NTP time sync now and then
  
  LcdNeedsRedraw |= HbusTask ();            // Send any scheduled HBUS messages
  LcdNeedsRedraw |= Rfm12Task ();           // Check for any incoming RFM12 packets and process them
  LcdNeedsRedraw |= TimeTask ();            // Display time if it has changed
  LcdNeedsRedraw |= TemperatureTask ();     // Display temperature if it has changed
  LcdNeedsRedraw |= LightIntensityTask ();  // Detect the light intensity
  LcdNeedsRedraw |= LcdBacklightTask ();    // Update the LCD backlight

  TempPoll ();

  if (Flags & F_TIME_CHANGED) {
    if (second () & 0x01) {
      ShiftPWM.SetOne (LED_HEART, LED_ON);
    } else {
      ShiftPWM.SetOne (LED_HEART, LED_OFF);
    }    
  }
  
  //
  // End of the loop, do all the housekeeping
  //
  
  if (LcdNeedsRedraw) {
    display.display();
  }

  TempPoll ();

//  if (LastTime != (millis () >> 2)) {
//    LastTime = millis () >> 2;
//
//    if (ShiftPWM.m_PWMValues[2] > 0) {
//      ShiftPWM.m_PWMValues[2]--;
//    }
//      
//    if (ShiftPWM.m_PWMValues[3] > 0) {
//      ShiftPWM.m_PWMValues[3]--;
//    }
//  }

  //
  // Send temperature and time packets via RF12
  //

  if (millis () > (LastTempPacketTime + 3000)) {
    LastTempPacketTime = millis ();

    while (!rf12_canSend()) {
      TempPoll ();
      rf12_recvDone();
    }

    rf12_sendStart(0, &TempPayload, sizeof TempPayload, 1); // sync mode!
    send_buff[0] = MSG_BROADCAST;
    send_buff[1] = MSG_SET_7SEG;
    SegConvertTemp (InTemp, send_buff + 2);  // In and Out temp are in fact Out and In
    send_buff[6] = 10;             // 1.0 seconds
    send_message(7);
  }

  if ((Flags & F_TIME_UPDATED) && (second () == 0) && (timeStatus() == timeSet)) {
    Flags &= ~F_TIME_UPDATED;
    TimePayload.year = year () - 2000;
    TimePayload.month = month ();
    TimePayload.day = day ();
    TimePayload.hour = hour ();
    TimePayload.minute = minute ();

    while (!rf12_canSend()) {
      TempPoll ();
      rf12_recvDone();
    }

    rf12_sendStart(0, &TimePayload, sizeof TimePayload, 1); // sync mode!
  }

  //
  // Clear all flags that were valid for just one loop
  //

  Flags &= ~F_TIME_CHANGED;
}

