#include <Time.h>
#include <JeeLib.h>
#include <PacketDefine.h>
 
//#include <Wire.h>

const int PinBlank = 3;
const int PinClk = 17;
const int PinLoad = 15;
const int PinDin = 16;
const int PinPwm = 9;

#define DEBUG_RMF12 0
#define METEO_SUPPORT 0

#define DIGIT7_OFFSET 20

const unsigned long DIGIT7_1 = (0x00000008L << DIGIT7_OFFSET);
const unsigned long DIGIT7_2 = (0x00000004L << DIGIT7_OFFSET);
const unsigned long DIGIT7_3 = (0x00000002L << DIGIT7_OFFSET);
const unsigned long DIGIT7_4 = (0x00000001L << DIGIT7_OFFSET);

const unsigned long SEGMENT7_G = (0x00000100L << DIGIT7_OFFSET);
const unsigned long SEGMENT7_F = (0x00000200L << DIGIT7_OFFSET);
const unsigned long SEGMENT7_E = (0x00000400L << DIGIT7_OFFSET);
const unsigned long SEGMENT7_D = (0x00000800L << DIGIT7_OFFSET);
const unsigned long SEGMENT7_C = (0x00000010L << DIGIT7_OFFSET);
const unsigned long SEGMENT7_B = (0x00000040L << DIGIT7_OFFSET);
const unsigned long SEGMENT7_A = (0x00000080L << DIGIT7_OFFSET);
const unsigned long SEGMENT7_DP = (0x00000020L << DIGIT7_OFFSET);

const unsigned long DIGIT16_1 = 0x00000002L;
const unsigned long DIGIT16_2 = 0x00000001L;

const unsigned long SEGMENT16_2 = 0x00040000L;
const unsigned long SEGMENT16_3 = 0x00080000L;
const unsigned long SEGMENT16_4 = 0x00020000L;
const unsigned long SEGMENT16_5 = 0x00010000L;
const unsigned long SEGMENT16_6 = 0x00008000L;
const unsigned long SEGMENT16_7 = 0x00004000L;
const unsigned long SEGMENT16_8 = 0x00002000L;
const unsigned long SEGMENT16_9 = 0x00001000L;
const unsigned long SEGMENT16_10 = 0x00000800L;
const unsigned long SEGMENT16_13 = 0x00000008L;
const unsigned long SEGMENT16_14 = 0x00000004L;
const unsigned long SEGMENT16_15 = 0x00000010L;
const unsigned long SEGMENT16_16 = 0x00000020L;
const unsigned long SEGMENT16_17 = 0x00000040L;
const unsigned long SEGMENT16_18 = 0x00000080L;
const unsigned long SEGMENT16_19 = 0x00000100L;
const unsigned long SEGMENT16_20 = 0x00000400L;
const unsigned long SEGMENT16_21 = 0x00000200L;

unsigned long LastMillis = 0;
unsigned long Digit7 [4] = {0, 0, 0, 0};
unsigned long Digit16 [2] = {0, 0};

//unsigned int Pressure = 0;
unsigned char Day = 0;
unsigned char Month = 0;
unsigned char Hour = 0;
unsigned char Minute = 0;
int InnerTemperature;
int OuterTemperature;


#define _A_SEG SEGMENT7_A
#define _B_SEG SEGMENT7_B
#define _C_SEG SEGMENT7_C
#define _D_SEG SEGMENT7_D
#define _E_SEG SEGMENT7_E
#define _F_SEG SEGMENT7_F
#define _G_SEG SEGMENT7_G
#define _P_SEG SEGMENT7_DP

#define seg_mac(a,b,c,d,e,f,g,p) (((a * _A_SEG) | (b * _B_SEG) | (c * _C_SEG) | (d * _D_SEG) | (e * _E_SEG) | (f * _F_SEG) | (g * _G_SEG) | (p * _P_SEG)))

//----------------------------------------------------------------------------
// 7seg definitions

const unsigned long PROGMEM seg_hex_table[] = {
    seg_mac(1, 1, 1, 1, 1, 1, 0, 0),             // 0
    seg_mac(0, 1, 1, 0, 0, 0, 0, 0),             // 1
    seg_mac(1, 1, 0, 1, 1, 0, 1, 0),             // 2
    seg_mac(1, 1, 1, 1, 0, 0, 1, 0),             // 3
    seg_mac(0, 1, 1, 0, 0, 1, 1, 0),             // 4
    seg_mac(1, 0, 1, 1, 0, 1, 1, 0),             // 5
    seg_mac(1, 0, 1, 1, 1, 1, 1, 0),             // 6
    seg_mac(1, 1, 1, 0, 0, 0, 0, 0),             // 7
    seg_mac(1, 1, 1, 1, 1, 1, 1, 0),             // 8
    seg_mac(1, 1, 1, 1, 0, 1, 1, 0),             // 9
    seg_mac(1, 1, 1, 0, 1, 1, 1, 0),             // A
    seg_mac(0, 0, 1, 1, 1, 1, 1, 0),             // B
    seg_mac(1, 0, 0, 1, 1, 1, 0, 0),             // C
    seg_mac(0, 1, 1, 1, 1, 0, 1, 0),             // D
    seg_mac(1, 0, 0, 1, 1, 1, 1, 0),             // E
    seg_mac(1, 0, 0, 0, 1, 1, 1, 0),             // F
    seg_mac(0, 0, 0, 0, 0, 0, 1, 0),             // -
    seg_mac(0, 0, 0, 0, 0, 0, 0, 0),             // space
};

#define SEVEN_dash  0x10
#define SEVEN_space 0x11

const unsigned long PROGMEM seg_alpha_table[] = {
  SEGMENT16_3 | SEGMENT16_21 | SEGMENT16_20 | SEGMENT16_14 | SEGMENT16_13 | SEGMENT16_9 | SEGMENT16_8 | SEGMENT16_4,
  SEGMENT16_13 | SEGMENT16_9,
  SEGMENT16_20 | SEGMENT16_14 | SEGMENT16_13 | SEGMENT16_15 | SEGMENT16_5 | SEGMENT16_4 | SEGMENT16_8,
  SEGMENT16_20 | SEGMENT16_14 | SEGMENT16_16 | SEGMENT16_15 | SEGMENT16_9 | SEGMENT16_8 | SEGMENT16_4,
  SEGMENT16_21 | SEGMENT16_19 | SEGMENT16_15 | SEGMENT16_13 | SEGMENT16_9,
  SEGMENT16_20 | SEGMENT16_14 | SEGMENT16_21 | SEGMENT16_19 | SEGMENT16_7 | SEGMENT16_8 | SEGMENT16_4,
  SEGMENT16_20 | SEGMENT16_14 | SEGMENT16_21 | SEGMENT16_3 | SEGMENT16_4 | SEGMENT16_8 | SEGMENT16_9 | SEGMENT16_19 | SEGMENT16_15,
  SEGMENT16_20 | SEGMENT16_14 | SEGMENT16_16 | SEGMENT16_5,
  SEGMENT16_20 | SEGMENT16_14 | SEGMENT16_21 | SEGMENT16_13 | SEGMENT16_19 | SEGMENT16_15 | SEGMENT16_3 | SEGMENT16_9 | SEGMENT16_4 | SEGMENT16_8,
  SEGMENT16_20 | SEGMENT16_14 | SEGMENT16_21 | SEGMENT16_19 | SEGMENT16_15 | SEGMENT16_13 | SEGMENT16_9 | SEGMENT16_8,

  SEGMENT16_3 | SEGMENT16_21 | SEGMENT16_20 | SEGMENT16_14 | SEGMENT16_13 | SEGMENT16_15 | SEGMENT16_19,  // P
  SEGMENT16_17 | SEGMENT16_6 | SEGMENT16_15 | SEGMENT16_9,                                                // h
  SEGMENT16_13 | SEGMENT16_14 | SEGMENT16_15 | SEGMENT16_17,  // degree
  SEGMENT16_3 | SEGMENT16_21 | SEGMENT16_20 | SEGMENT16_14 | SEGMENT16_4 | SEGMENT16_8,  // C
  SEGMENT16_17 | SEGMENT16_21 | SEGMENT16_20 | SEGMENT16_19 | SEGMENT16_6 | SEGMENT16_15 | SEGMENT16_9 | SEGMENT16_8 | SEGMENT16_5 | SEGMENT16_16,  // %
  SEGMENT16_20 | SEGMENT16_14 | SEGMENT16_17 | SEGMENT16_6,   // T
  SEGMENT16_3 | SEGMENT16_21 | SEGMENT16_13 | SEGMENT16_9 | SEGMENT16_8 | SEGMENT16_4, // U
  SEGMENT16_14 | SEGMENT16_20 | SEGMENT16_21 | SEGMENT16_19 | SEGMENT16_15 | SEGMENT16_9 | SEGMENT16_8 | SEGMENT16_4, // S
  SEGMENT16_3 | SEGMENT16_21 | SEGMENT16_18 | SEGMENT16_7 | SEGMENT16_9 | SEGMENT16_13, // N
  SEGMENT16_20 | SEGMENT16_14 | SEGMENT16_21 | SEGMENT16_19 | SEGMENT16_3 | SEGMENT16_4 | SEGMENT16_8, // E
  SEGMENT16_3 | SEGMENT16_21 | SEGMENT16_20 | SEGMENT16_14 | SEGMENT16_13 | SEGMENT16_9 | SEGMENT16_8 | SEGMENT16_4, // O
  SEGMENT16_19 | SEGMENT16_15, // -
  SEGMENT16_3 | SEGMENT16_20 | SEGMENT16_14 | SEGMENT16_21 | SEGMENT16_13 | SEGMENT16_9 | SEGMENT16_19 | SEGMENT16_15, // A
};

#define ALPHA_P     10
#define ALPHA_h     11
#define ALPHA_deg   12
#define ALPHA_C     13
#define ALPHA_perc  14
#define ALPHA_T     15
#define ALPHA_U     16
#define ALPHA_S     17
#define ALPHA_N     18
#define ALPHA_E     19
#define ALPHA_O     20
#define ALPHA_dash  21
#define ALPHA_A     22

const byte day_of_week [] = {
  ALPHA_N, ALPHA_E, 
  ALPHA_P, ALPHA_O, 
  ALPHA_U, ALPHA_T, 
  ALPHA_S, ALPHA_T, 
  ALPHA_C, ALPHA_T, 
  ALPHA_P, ALPHA_A, 
  ALPHA_S, ALPHA_O, 
};

//----------------------------------------------------------------------------
// 
// VFD functions
//

//
// VfdSend sends the dword to the VFD drivers. 
// This is the lowest level interface to VFD, just setting the corresponding anodes or grids to Vbb or Gnd
//

void VfdSend (unsigned long DataIn) {
  int i;

  digitalWrite (PinLoad, LOW);
//  digitalWrite (PinClk, LOW);
    PORTC &= ~_BV(3);    


  for (i = 0; i < 32; i++) {
    if (DataIn & 0x00000001) {
//      digitalWrite (PinDin, HIGH);
      PORTC |= _BV(2);
    } else {
//      digitalWrite (PinDin, LOW);
      PORTC &= ~_BV(2);    
    }
//    digitalWrite (PinClk, HIGH);
//    digitalWrite (PinClk, LOW);
    PORTC |= _BV(3);
    PORTC &= ~_BV(3);    
    DataIn >>= 1;
  }

  digitalWrite (PinLoad, HIGH);
  digitalWrite (PinLoad, LOW);
}

//
// VfdDisplay displays the six characters on the display. The input can be either a number or a character code - see seg_hex_table and seg_alpha_table
// In fact, it just takes the input and converts that to a corresponding bitmap, then stores it into Digit7 and Digit16 arrays.
//

void VfdDisplay (byte D1, byte D2, byte D3, byte D4, byte D5, byte D6) {

  Digit7 [0] = pgm_read_dword_near (seg_hex_table + D1);
  Digit7 [1] = pgm_read_dword_near (seg_hex_table + D2);
  Digit7 [2] = pgm_read_dword_near (seg_hex_table + D3);
  Digit7 [3] = pgm_read_dword_near (seg_hex_table + D4);
  Digit16 [0] = pgm_read_dword_near (seg_alpha_table + D5);
  Digit16 [1] = pgm_read_dword_near (seg_alpha_table + D6);
}

void VfdDisplayDot (byte D1, byte D2, byte D3, byte D4) {

  if (D1) Digit7 [0] |= SEGMENT7_DP;
  if (D2) Digit7 [1] |= SEGMENT7_DP;
  if (D3) Digit7 [2] |= SEGMENT7_DP;
  if (D4) Digit7 [3] |= SEGMENT7_DP;
}

//
// VfdRefresh is called every millisecond and displays just one 7seg and one alpha digit. As it's callled repeatedly, it takes care about multiplexing.
//

void VfdRefresh (void) {
  static byte Digit = 0;
  
  switch (Digit) {
    case 0:
      VfdSend (DIGIT7_1 | Digit7 [0] |
               DIGIT16_1 | Digit16 [0]);
      Digit++;
      break;
      
    case 1:
      VfdSend (DIGIT7_2 | Digit7 [1] |
               DIGIT16_1 | Digit16 [0]);
      Digit++;
      break;
      
    case 2:
      VfdSend (DIGIT7_3 | Digit7 [2] |
               DIGIT16_2 | Digit16 [1]);
      Digit++;
      break;
      
    case 3:
      VfdSend (DIGIT7_4 | Digit7 [3] |
               DIGIT16_2 | Digit16 [1]);
      // Fall through
      
    default:               
      Digit = 0;
      break;
  }
}   

//----------------------------------------------------------------------------
//
// BMP085 function to normalize the measured pressure to sea level
//

#if METEO_SUPPORT
float bmp085GetSeaLevelPressure(float stationPressure, float trueAltitude) {
  // return the pressure in hundredths of a millibar that corresponds to the station pressure in hundredths of a millibar and true altitude in meters
  return stationPressure / pow((1-trueAltitude/44330), 5.255);
}
#endif // METEO_SUPPORT

//----------------------------------------------------------------------------
//
// PacketDecode processes the packet received from RFM12 and updates the global variables for time, pressure, humidity and external temperature accordingly
//

void PacketDecode () {
  char c[4];
  MeteoPayload_t *Meteo;
  TimePayload_t *Time;
  TemperaturePayload_t *Temp;

  switch (*rf12_data) {
    
    case RF12_PACKET_TIME:
      Time = (TimePayload_t *) rf12_data;
      Day = Time->day;
      Month = Time->month;
      Hour = Time->hour;
      Minute = Time->minute;
      setTime (Hour, Minute, 0, Day, Month, 2000 + Time->year);

#if DEBUG_RMF12
      Serial.print (F("Time packet: "));
      Serial.print (Day);
      Serial.print ('.');
      Serial.print (Month);
      Serial.print (' ');
      Serial.print (2000 + Time->year);
      Serial.print (' ');
      Serial.print (Hour);
      Serial.print (':');
      Serial.println (Minute);
#endif // DEBUG_RMF12

      break;

    case RF12_PACKET_METEO:

#if METEO_SUPPORT
      Meteo = (MeteoPayload_t *) rf12_data;
      Pressure = (uint16_t) (bmp085GetSeaLevelPressure((float) Meteo->pres, 519.0) / 100 + 0.5);
      Temperature = (Meteo->temp + 5) / 10;

#if DEBUG_RMF12
      Serial.print (F("Meteo packet: "));
      Serial.print (Temperature);
      Serial.print (F ("oC "));
      Serial.print (Pressure);
      Serial.print (F ("hPa "));
      Serial.print ((Meteo->dht_temp + 5) / 10);
      Serial.print (F ("oC "));
      Serial.print ((Meteo->humidity + 5) / 10);
      Serial.print ('%');
      Serial.println ();
#endif // DEBUG_RMF12
#endif // METEO_SUPPORT

      break;
      
    case RF12_PACKET_TEMPERATURE:
      Temp = (TemperaturePayload_t *) rf12_data;
      InnerTemperature = Temp->temp1;
      OuterTemperature = Temp->temp2;

#if DEBUG_RMF12
      Serial.print (F("Temperature packet: "));
      Serial.print (InnerTemperature);
      Serial.print (' ');
      Serial.println (OuterTemperature);
#endif //DEBUG_RMF12

      break;

    default:

#if DEBUG_RMF12
      uint8_t Rf12Len;
      volatile uint8_t *Rf12Data;
      Rf12Len = rf12_len;
      Rf12Data = rf12_data;
      Serial.print (F("Unknown packet: "));
      while (Rf12Len > 0) {
        sprintf (c, "%02X ", *Rf12Data);
        Serial.print (c);
        Rf12Len--;
        Rf12Data++;
      }
      Serial.println ();
#endif // DEBUG_RMF12

      break;
  }
}

void DisplayTemp (signed int Temp) {
  byte TempSign;
  
  if (Temp < 0) {
    TempSign = SEVEN_dash;
    Temp = -Temp;
  } else {
    TempSign = SEVEN_space;
  }
 
  Temp = (Temp + 8) / 16;
  
  if (Temp < 10) {
    VfdDisplay (SEVEN_space, SEVEN_space, TempSign, Temp, ALPHA_deg, ALPHA_C);
  } else {
    VfdDisplay (SEVEN_space, TempSign, Temp / 10, Temp % 10, ALPHA_deg, ALPHA_C);
  }      
}      

//----------------------------------------------------------------------------
//
// setup ()
//

void setup() {

  TCCR1B = TCCR1B & 0b11111000 | 0x01; // Change the PWM frequency of pins 9 and 10 to ~31kHz - see http://playground.arduino.cc/Main/TimerPWMCheatsheet
  pinMode(PinClk, OUTPUT);
  pinMode(PinLoad, OUTPUT);
  pinMode(PinDin, OUTPUT);
  pinMode(PinPwm, OUTPUT); 
  
  analogWrite(PinPwm, 192);  

  Serial.begin(57600);
  Serial.println("\n[Rf12Clock]\n");
  rf12_initialize(RF12_NET_NODE, RF12_868MHZ, RF12_NET_GROUP);

  VfdSend (0x0);

  pinMode(PinBlank, OUTPUT);
  digitalWrite (PinBlank, LOW);
  
  setTime (0, 0, 0, 1, 1, 2013);    // Dummy time until the time gets synced
//  Pressure = 9999;

}

//----------------------------------------------------------------------------
//
// loop ()
//

void loop() {
  unsigned long Millis;
  static int Second = 0;
  time_t Time; 
  static time_t PreviousTime = 0;
  static unsigned int LightIntensity = 0;
//  static byte BrightnessCounter = 4; // Maximum usable value is 4, the flickering shows for higher values

  //
  // Get the time stamp of this loop pass
  //
  
  Millis = millis();

  //
  // Check if any valid packet was received, and decode it
  //
  
  if (rf12_recvDone() && rf12_crc == 0) {
    PacketDecode ();
  }

  //
  // Every time the Millis changes, measure the light intensity, set the PWM to reflex it, and refresh the display
  //
  
  if (Millis != LastMillis) {
    LastMillis = Millis;
    LightIntensity = ((LightIntensity * 31) + analogRead(0)) / 32;
    
    //
    // So far, it looks like minimal usable PWM value for VFD is ~48.
    // Light values for night are below 50, dark room with monitor on is ~300, light on boosts the light value to the max.
    //
    
    if (LightIntensity < 48) {      // Night mode, set minimal brightness
      analogWrite(PinPwm, 48);  
    } else if (LightIntensity < 256) {
      analogWrite(PinPwm, 96);  
    } else {
      analogWrite(PinPwm,((LightIntensity - 256) / 5) + 96);
    }
    
//    LightIntensity = ((LightIntensity * 127) + (analogRead(0) >> 2)) >> 7;
//    if (LightIntensity < 5) LightIntensity = 5;
//    if (BrightnessCounter > 0) {
//      digitalWrite (PinBlank, HIGH);
//      BrightnessCounter--;
//    } else {
//      digitalWrite (PinBlank, LOW);
      VfdRefresh ();
//      BrightnessCounter = 2;
//    }      
  }

  Time = now ();
  
  if (Time != PreviousTime) {
    PreviousTime = Time;
//    Serial.println(LightIntensity);
    
    switch (second () % 10) {
      case 0:
      case 1:
      case 2:
      case 3:
        VfdDisplay ((hour () / 10) > 0 ? (hour () / 10) : SEVEN_space, 
                    hour () % 10, minute () / 10, minute () % 10, second () / 10, second () % 10);
        break;
        
      case 4:
//      case 5:
        VfdDisplay ((day () / 10) > 0 ? (day () / 10) : SEVEN_space, 
                    day () % 10, 
                    (month () / 10) > 0 ? month () / 10 : month () % 10,
                    (month () / 10) > 0 ? month () % 10 : SEVEN_space,
                    day_of_week [(weekday () - 1) * 2], 
                    day_of_week [(weekday () - 1) * 2 + 1]);
                    
        VfdDisplayDot (0, 1, 0, 0);
        break;

      case 6:
//      case 7:
        DisplayTemp (OuterTemperature);
        break;

      case 8:
//      case 9:
        DisplayTemp (InnerTemperature);
        break;

      default:      
        break;
    }
  }
}
