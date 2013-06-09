#include <OneWire.h>

// Ports demo, reads out a BMP085 sensor connected via I2C
// 2009-02-17 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

// 2010-05-22: added support for all resolution modes
// 2010-05-25: extended to also broadcast all readings over wireless
// 2010-06-17: add power saving logic, should reduce consumption by over 90%
// 2010-06-24: improved power savings, several "hot spots" optimized

// see http://news.jeelabs.org/2010/06/20/battery-savings-for-the-pressure-plug/
// see http://news.jeelabs.org/2010/06/30/going-for-gold-with-the-bmp085/

#include <JeeLib.h>
#include <PacketDefine.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>

Port DCF77 (1);
MilliTimer timer;
TimePayload_t payload;
OneWire ds(5);  

// This power-saving code was shamelessly stolen from the rooms.pde sketch,
// see http://code.jeelabs.org/viewvc/svn/jeelabs/trunk/jeemon/sketches/rooms/

EMPTY_INTERRUPT(WDT_vect); // just wakes us up to resume

static void watchdogInterrupts (char mode) {
  MCUSR &= ~(1<<WDRF); // only generate interrupts, no reset
  cli();
  WDTCSR |= (1<<WDCE) | (1<<WDE); // start timed sequence
  WDTCSR = mode >= 0 ? bit(WDIE) | mode : 0;
  sei();
}

static void lowPower (byte mode) {
  // prepare to go into power down mode
  set_sleep_mode(mode);
  // disable the ADC
  byte prrSave = PRR, adcsraSave = ADCSRA;
  ADCSRA &= ~ bit(ADEN);
  PRR |=  bit(PRADC);
  // zzzzz...
  sleep_mode();
  // re-enable the ADC
  PRR = prrSave;
  ADCSRA = adcsraSave;
}

static byte loseSomeTime (word msecs) {
  // only slow down for periods longer than the watchdog granularity
  if (msecs >= 16) {
    // watchdog needs to be running to regularly wake us from sleep mode
    watchdogInterrupts(0); // 16ms
    for (word ticks = msecs / 16; ticks > 0; --ticks) {
      lowPower(SLEEP_MODE_PWR_DOWN); // now completely power down
      // adjust the milli ticks, since we will have missed several
      extern volatile unsigned long timer0_millis;
      timer0_millis += 16;
    }
    watchdogInterrupts(-1); // off
    return 1;
  }
  return 0;
}

// End of new power-saving code.


static uint16_t dcfWidth;
static uint8_t dcfLevels, dcfBits, dcfParity;
static uint8_t dcfValue[8];
static uint8_t year, month, day, hour, minute, dst;

static prog_uint8_t daysInMonth[] = { 
  31,28,31,30,31,30,31,31,30,31,30,31 };

// number of days since 2000/01/01, valid for 2001..2099
static uint16_t date2days(uint8_t y, uint8_t m, uint8_t d) {
  uint16_t days = d;
  for (uint8_t i = 1; i < m; ++i)
    days += pgm_read_byte(daysInMonth + i - 1);
  if (m > 2 && y % 4 == 0)
    ++days;
  return days + 365 * y + (y - 1) / 4;
}

static uint32_t unixTime(uint16_t days, uint8_t h, uint8_t m, uint8_t s, uint8_t dst) {
  uint32_t secs = 946681200L; // 2000/01/01 00:00:00 CET
  return secs + ((((days * 24L + h + (dst ? -1 : 0)) * 60) + m) * 60) + s;
}

static uint8_t dcfExtract(uint8_t pos, uint8_t len) {
  uint16_t *p = (uint16_t*) (dcfValue + (pos >> 3));
  uint8_t val = (*p >> (pos & 7)) & ((1 << len) - 1);
  return val - (val / 16) * 6; // bcd -> dec
}

static uint8_t dcfMinute() {
  year = dcfExtract(50, 8);
  month = dcfExtract(45, 5);
  day = dcfExtract(36, 6);
  hour = dcfExtract(29, 6);
  minute = dcfExtract(21, 7);
  dst = dcfExtract(17, 1);
  return 1 <= year && year <= 99 && 1 <= month && month <= 12 &&
    1 <= day && day <= 31 && hour <= 23 && minute <= 59;
}

static uint8_t dcfPoll(uint8_t signal) {
  uint8_t ok = 0;
  static uint32_t last;
  if (millis() != last) {
    last = millis();

    // track signal levels using an 8-bit shift register
    dcfLevels = (dcfLevels << 1) | signal;
    if (dcfLevels == 0x07F) {
      if (dcfWidth > 1000) {
        if (dcfBits == 59)
          ok = dcfMinute();
        memset(dcfValue, 0, sizeof dcfValue);
        dcfBits = 0;
      }
      dcfWidth = 0;
    } 
    else if (dcfLevels == 0xFE) {
      if (0) {
        Serial.print("dcf ");
        Serial.println((int) dcfWidth);
      }
      if (dcfWidth >= 144) {
        dcfValue[dcfBits>>3] |= _BV(dcfBits & 7);
        dcfParity ^= 1;
      }
      switch (++dcfBits) {
      case 15: 
        dcfParity = 0;
      case 29: 
      case 36: 
      case 59: 
        if (dcfParity) dcfBits = 0;
      }
      dcfWidth = 0;
    }
    ++dcfWidth;
  }
  return ok;
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
//  float celsius;
  unsigned int raw;
  signed int sraw = raw;
  static state_t State = S_IDLE;
  static uint32_t WaitTime;

  switch (State) {
  case S_IDLE:
    ds.reset();
    ds.select(addr1);
    ds.write(0x44,1);         // start conversion, with parasite power on at the end
    WaitTime = millis () + 1000;
    State = S_WAIT;
    break;

  case S_WAIT:
    if (millis () > WaitTime) {
      State = S_READ;
    }
    break;

  case S_READ:
    present = ds.reset();
    ds.select(addr1);    
    ds.write(0xBE);         // Read Scratchpad
    State = S_COMP;
    break;

  case S_COMP:  
    for ( i = 0; i < 9; i++) {           // we need 9 bytes
      data1[i] = ds.read();
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
    if (1) {
      Serial.print("Temp ");
      Serial.print(raw, HEX);
      Serial.print(" ");
      sraw = raw;
      if (sraw < 0) {
        Serial.print("-");
        sraw = - sraw;
      }
  
      Serial.print(sraw / 16);
      Serial.print(".");
      Serial.println(sraw & 0x000F);
    }
    
    TempPayload.temp1 = raw;
    State = S_IDLE2;
    break;

  case S_IDLE2:
    ds.reset();
    ds.select(addr2);
    ds.write(0x44,1);         // start conversion, with parasite power on at the end
    WaitTime = millis () + 1000;
    State = S_WAIT2;
    break;

  case S_WAIT2:
    if (millis () > WaitTime) {
      State = S_READ2;
    }
    break;

  case S_READ2:
    present = ds.reset();
    ds.select(addr2);    
    ds.write(0xBE);         // Read Scratchpad
    State = S_COMP2;
    break;

  case S_COMP2:  
    for ( i = 0; i < 9; i++) {           // we need 9 bytes
      data2[i] = ds.read();
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
//    celsius = (float)raw / 16.0;
//    Serial.print("Temp2 ");
//    Serial.println(celsius);
//    Serial.println(raw, HEX);
    TempPayload.temp2 = raw;
    State = S_IDLE;
    break;

  default:
    State = S_IDLE;
    break;
  }        

}


void setup() {
  byte i;  

  delay(2500);          // Add a bit of delay after startup to give me some time to start the serial monitor
  Serial.begin(57600);
  Serial.print("\n[RF12TimeNode]");

  rf12_initialize(RF12_NET_NODE_TIME, RF12_868MHZ, RF12_NET_GROUP);

  payload.type = RF12_PACKET_TIME;
  DCF77.mode(INPUT);
  DCF77.digiWrite(1); // pull-up

  //
  // Initialise the first temperature sensor
  //

  if ( !ds.search(addr1)) {
    Serial.println("No more addresses.");
    Serial.println();
    ds.reset_search();
    delay(250);
  }

  Serial.print("ROM =");
  for( i = 0; i < 8; i++) {
    Serial.write(' ');
    Serial.print(addr1[i], HEX);
  }

  if (OneWire::crc8(addr1, 7) != addr1[7]) {
    Serial.println("CRC is not valid!");
    return;
  }
  Serial.println();

  // the first ROM byte indicates which chip
  switch (addr1[0]) {
  case 0x10:
    Serial.println("  Chip = DS18S20");  // or old DS1820
    type_s1 = 1;
    break;
  case 0x28:
    Serial.println("  Chip = DS18B20");
    type_s1 = 0;
    break;
  case 0x22:
    Serial.println("  Chip = DS1822");
    type_s1 = 0;
    break;
  default:
    Serial.println("Device is not a DS18x20 family device.");
    return;
  } 

  //
  // Initialise the second temperature sensor
  //

  if ( !ds.search(addr2)) {
    Serial.println("No more addresses.");
    Serial.println();
    ds.reset_search();
    delay(250);
  }

  Serial.print("ROM =");
  for( i = 0; i < 8; i++) {
    Serial.write(' ');
    Serial.print(addr2[i], HEX);
  }

  if (OneWire::crc8(addr2, 7) != addr2[7]) {
    Serial.println("CRC is not valid!");
    return;
  }
  Serial.println();

  // the first ROM byte indicates which chip
  switch (addr2[0]) {
  case 0x10:
    Serial.println("  Chip = DS18S20");  // or old DS1820
    type_s2 = 1;
    break;
  case 0x28:
    Serial.println("  Chip = DS18B20");
    type_s2 = 0;
    break;
  case 0x22:
    Serial.println("  Chip = DS1822");
    type_s2 = 0;
    break;
  default:
    Serial.println("Device is not a DS18x20 family device.");
    return;
  } 
  TempPayload.type = RF12_PACKET_TEMPERATURE;
}


void loop() {

  static uint32_t LastTempPacket = 0;
  MilliTimer wait;                 // radio needs some time to power up, why?
  
  //  // spend most of the waiting time in a low-power sleep mode
  //  // note: the node's sense of time is no longer 100% accurate after sleeping
  //  loseSomeTime(timer.remaining()); // go into a (controlled) comatose state
  //
  if (dcfPoll(DCF77.digiRead())) {
    rf12_sleep(RF12_WAKEUP);         // turn radio back on at the last moment

    payload.year = year;
    payload.month = month;
    payload.day = day;
    payload.hour = hour;
    payload.minute = minute;

//    Serial.print(2000 + year);
//    Serial.print(' ');
//    Serial.print((int) month);
//    Serial.print(' ');
//    Serial.print((int) day);
//    Serial.print(' ');
//    Serial.print((int) hour);
//    Serial.print(' ');
//    Serial.println((int) minute);

    while (!wait.poll(5)) {
      rf12_recvDone();
      dcfPoll(DCF77.digiRead());
      lowPower(SLEEP_MODE_IDLE);
    }

    while (!rf12_canSend()) {
      dcfPoll(DCF77.digiRead());
      rf12_recvDone();
    }

    rf12_sendStart(0, &payload, sizeof payload, 1); // sync mode!
    LastTempPacket = millis ();
    dcfPoll(DCF77.digiRead());
    rf12_sleep(RF12_SLEEP);          // turn the radio off
    dcfPoll(DCF77.digiRead());
  }
  
  TempPoll ();
  
  if (millis () > (LastTempPacket + 3000)) {
    LastTempPacket = millis ();
    while (!wait.poll(5)) {
      rf12_recvDone();
      dcfPoll(DCF77.digiRead());
      lowPower(SLEEP_MODE_IDLE);
    }

    while (!rf12_canSend()) {
      dcfPoll(DCF77.digiRead());
      rf12_recvDone();
    }

    rf12_sendStart(0, &TempPayload, sizeof TempPayload, 1); // sync mode!
    dcfPoll(DCF77.digiRead());
    rf12_sleep(RF12_SLEEP);          // turn the radio off
    dcfPoll(DCF77.digiRead());
  }
}





