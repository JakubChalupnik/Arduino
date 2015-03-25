//*******************************************************************************
//*
//* Arduino Due based central node
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
//* Kubik       8.3.2015  Removed display support
//*******************************************************************************

//*******************************************************************************
//*                            HW details                                       *
//*******************************************************************************
//
// Used pins:
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
//  RFM12B radio: HW SPI and
//   pin 52 - CS  (Configured in the library itself!)
//
//  Indication LEDs
//   pin 11 - white LED
//   pin 12 - yellow LED
//   pin 13 - red LED
//
//  Button(s)
//   pin 24 - button 1
//
//  Others
//   pin A0 - light sensor (photoresistor)
//

//*******************************************************************************
//*                        User configurable SW settings                        *
//******************************************************************************* 

#define DEBUG 1
#define TIME_UPDATE_PERIOD 3600
#define STATIC 0      // set to 1 to disable DHCP (adjust myip/gwip values below)
#define MAX_SENSORS 16
#define DELAY_DEBOUNCE 10
#define DELAY_REPEAT_START 40
#define DELAY_REPEAT 25 

//*******************************************************************************
//*                           Includes and defines                              *
//******************************************************************************* 

#define VERSION 0 

#include <EtherCard.h>
#include <SPI.h>
#include <Time.h>
#include <Timezone.h>    // https://github.com/JChristensen/Timezone   
#include "RF24.h"
#include "RF24Network.h"
#include "Rf24PacketDefine.h"

#define NET_RST  47
#define NET_CS   46

#define RF24_1_CSN 10
#define RF24_1_CE 23
#define RF24_2_CSN 4
#define RF24_2_CE 22

#define LED_WHITE_PIN 11
#define LED_YELLOW_PIN 12
#define LED_RED_PIN 13

#define BUTTON_1_PIN 24      // Pin used by button 1
#define BUTTON_1 0x01        // Mask corresponding to button 1

#define LIGHT_SENSOR A0

typedef struct {
  SensorPayload_t Payload;
  time_t LastReport;
} SensorNode_t;
  
typedef struct {
  uint16_t Address;
  uint8_t BattLevel;
  uint16_t Temperature[2];
  char Id[NODE_ID_SIZE];
  uint16_t Flags;
} NetworkNode_t;

typedef enum {S_SENSORS, S_TIME, S_STATUS} Screen_t;

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

SensorNode_t Sensors [MAX_SENSORS];
int SensorsCount = 0;

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

//
// Buttons support. Corresponding bit will be set when the button was pressed recently.
// To confirm the button was read, clear the corresponding bit
//

uint8_t Buttons = 0;   

//
// Other variables that have to be global
//

uint16_t LightIntensity = 0;
const uint8_t LuminosityTable[] PROGMEM = {
  0, 1, 2, 3, 4, 5, 7, 9, 12, 15, 18, 22, 27, 32, 37, 44, 50, 58, 66, 75, 85, 96, 107, 120, 133, 147, 163, 179, 196, 215, 234, 255
};

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
//*                              LED support                                    *
//******************************************************************************* 

typedef struct {
  uint8_t Pin;
  uint8_t Value;
  uint8_t Desired;
} Led_t;

Led_t Leds[] = {
  {LED_WHITE_PIN, 0, 0},
  {LED_YELLOW_PIN, 0, 0},
  {LED_RED_PIN, 0, 0}
};

#define LED_COUNT (sizeof (Leds) / sizeof (Led_t))

#define LED_WHITE 0
#define LED_YELLOW 1
#define LED_RED 2

void InitLeds (void) {
  uint8_t i;
  
  for (i = 0; i < LED_COUNT; i++) {
    pinMode (Leds[i].Pin, OUTPUT);
    SetLed (i, 0);
  }
}

void SetLed (uint8_t Led, uint8_t Value) {
  
  if (Led >= LED_COUNT) {
    return;
  }
  
  if (Value >= sizeof (LuminosityTable)) {
    Value = 31;
  }
  
  analogWrite (Leds[Led].Pin, LuminosityTable[Value]);
  Leds[Led].Desired = Leds[Led].Value = Value;
}

void ChangeLed (uint8_t Led, uint8_t Value) {
  
  if (Led >= LED_COUNT) {
    return;
  }
  
  if (Value >= sizeof (LuminosityTable)) {
    Value = 31;
  }
  
  Leds[Led].Desired = Value;
}

void BlinkLed (uint8_t Led) {
  uint8_t Max;
  
  if (Led >= LED_COUNT) {
    return;
  }
  
  Max = (LightIntensity < 100) ? 3 : 31;
  
  analogWrite (Leds[Led].Pin, LuminosityTable[Max]);
  Leds[Led].Value = Max;
  Leds[Led].Desired = 0;
}

void UpdateLeds (void) {
  int i;
  
  for (i = 0; i < LED_COUNT; i++) {
    if (Leds[i].Value == Leds[i].Desired) {
      continue;
    } 
    if (Leds[i].Value > Leds[i].Desired) {
      Leds[i].Value--;
    } else {
      Leds[i].Value++;
    }
    analogWrite (Leds[i].Pin, LuminosityTable[Leds[i].Value]);
  }
}
  
//*******************************************************************************
//*                          Input button(s) support                            *
//******************************************************************************* 

//
// Reads all the buttons and sets the corresponding bits of the return value
//

uint8_t GetButtons (void) {
 uint8_t Buttons = 0;
 
 if (!digitalRead (BUTTON_1_PIN)) {    // 0 means button pressed
   Buttons |= BUTTON_1;
 }
 return Buttons;  
}

//
// Updates the buttons status, takes care about autorepeat etc.
// Must be called periodically at regular intervals, e.g. every 10ms
//

void UpdateButtons (void) {
  static byte id = 0x00;
  static word key_counter = 0;
  byte c;
  
  c = GetButtons ();
  if (c == 0) {                   // No key pressed
    id = 0;
    key_counter = 0;
  } else if (c != id) {           // New key differs from the previous one
    id = c;
    key_counter = 0;
  } else {                        // New key is the same as previous one
    key_counter++;
  }
  
  if (key_counter == DELAY_DEBOUNCE) {    // Debouncing complete - set key pressed
    Buttons = id;
  } else if (key_counter == DELAY_REPEAT_START) { // Repeated key
    Buttons = id;
    key_counter -= DELAY_REPEAT;
  } 
}

//*******************************************************************************
//*                            Screen output helpers                            *
//******************************************************************************* 

char *PayloadToString (int Index) {
  static char Buffer[64];
  char *b;
  SensorPayload_t *p = &Sensors[Index].Payload;

  b = Buffer + sprintf (Buffer, "%c%c ", (char) (p->SensorId >> 8), (char) (p->SensorId & 0xFF));
  
  if (p->BattLevel == 255) {
    b += sprintf (b, "NoBat  ");
  } else if (p->BattLevel == 0) {
    b += sprintf (b, "BatLow ");
  } else {
    b += sprintf (b, "%4.2fV  ", p->BattLevel * 10 + 2000);
  }
  
  switch (p->PacketType) {
    case RF24_SENSOR_TYPE_TEMP:
      b += sprintf (b, "%5.1fC ", (p->TemperatureInt + 0.05) / 10.0); 
      if (p->TemperatureExt < 65535) {
        b += sprintf (b, "%5.1fC ", (p->TemperatureExt + 0.05) / 10.0); 
      }
      break;
      
    case RF24_SENSOR_TYPE_METEO:
      b += sprintf (b, "%5.1fC %4dhPa ", (p->Temperature + 0.05) / 10.0, p->Pressure); 
      break;
      
    default:
      break;
  }
  
  return Buffer;
}

//*******************************************************************************
//*                            Other small stuff                                *
//******************************************************************************* 

inline uint16_t GetLightIntensity (void) {
  return 1023 - analogRead (LIGHT_SENSOR);  
}

//*******************************************************************************
//*                            Arduino setup method                             *
//******************************************************************************* 

void setup () {
  char buff [64];
  
  //
  // Initialize pins used for resets of various peripherals, and reset all of them
  //
  
  pinMode (NET_RST, OUTPUT);
  digitalWrite (NET_RST, LOW);
  delay (1);
  digitalWrite (NET_RST, HIGH);
  delay (10);
  
  //
  // Init pins used for LED, turn all of them off
  //

  analogWriteResolution (8);
  InitLeds ();
  
  //
  // Button support
  //
  
  pinMode (BUTTON_1_PIN, INPUT_PULLUP);

  //
  // Initialize serial port and send info
  //
  
  Serial.begin (57600);
  while (!Serial); 
  Serial.println ("[Arduino Due Central Node!]"); 

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
    Debugln ("Failed to access Ethernet controller");
  } else {
    Debugln ("Ethernet controller initialized");
    Flags |= F_ETHERNET;
  }

  if (Flags | F_ETHERNET) {  
    #if STATIC
    ether.staticSetup (myip, gwip);
    #else
    if (!ether.dhcpSetup ()) {
      Debugln ("DHCP failed, using static IP");
      ether.staticSetup (myip, gwip);
    }
    #endif
  }

  //
  // Other init
  //
  
  LightIntensity = GetLightIntensity ();
}

//*******************************************************************************
//*                              Main program loop                              *
//******************************************************************************* 

void loop() {
  SensorPayload_t Payload;
  int i;
  static unsigned long LastTime = 0, LastTimeMs = 0;
  static char buff[64];
  time_t t;
  static Screen_t Screen = S_SENSORS;
  bool ScreenNeedsUpdate = false;
  bool ScreenNeedsClear = false;
  
  UpdateTimeNtp ();  
  Network.update ();
  
  //
  // If button is pressed, switch the screen
  //
  
  if (Buttons & BUTTON_1) {
    Buttons &= ~BUTTON_1;
    
    switch (Screen) {
      case S_SENSORS:
        Screen = S_TIME;
        break;
      case S_TIME:
        Screen = S_STATUS;
        break;
      case S_STATUS:
        Screen = S_SENSORS;
        break;
      default:
        Screen = S_SENSORS;
        break;
    }
    ScreenNeedsUpdate = true;
    ScreenNeedsClear = true;
  }
        
  //
  // Any payloads from sensors available?
  //
  
  if (Radio1.available ()) {
    BlinkLed (LED_WHITE);
    ScreenNeedsUpdate = true;
    
    Radio1.read (&Payload, sizeof (SensorPayload_t));
    
    //
    // Search through all the known sensors and see if this one has reported already.
    // If found, copy the new values over the old one.
    // If not found, add the new entry into the array of known sensors
    //
    
    for (i = 0; i < SensorsCount; i++) {
      if (Payload.SensorId == Sensors[i].Payload.SensorId) {
        break;
      }
    }
    
    memcpy (&Sensors[i].Payload, &Payload, sizeof (Payload));
    Sensors[i].LastReport = now ();
    if (i == SensorsCount) {
      SensorsCount++;
    }
  }

  Network.update ();
  
  //
  // Execute the following code every 10ms. Use the recommended subtracting method to handle millis() overflow
  //

  if ((millis () - LastTimeMs) >= 10) {
    LastTimeMs = millis ();
    UpdateLeds ();
    UpdateButtons (); 
    LightIntensity = ((LightIntensity * 7) + GetLightIntensity ()) / 8;
  }

  //
  // Execute the following code every second. Use the recommended subtracting method to handle millis() overflow
  //

  if ((millis () - LastTime) >= 10000) {
    LastTime = millis ();

    BlinkLed (LED_YELLOW);
    if (Screen == S_TIME) {
      ScreenNeedsUpdate = true;
    }
    
  }

  //
  // Display screen if necessary
  //
  
  if (!ScreenNeedsUpdate) {
    return;
  }
  
  for (i = 0; i < SensorsCount; i++) {
    Serial.println (PayloadToString (i));
  }

} 
