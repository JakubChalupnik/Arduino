#include <movingAvg.h>
#include <LibPrintf.h>
#include <ModbusRtu.h>
#include <SoftwareSerial.h>
#include <LiquidCrystal.h>

// Based on:
//   Modbus master example 2
//   created long time ago by smarmengol
//   modified 29 July 2016 by Helium6072

//
// Initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
// Also define custom characters.
//

const byte LcdBattery[8] = {0xe,0x11,0x11,0x11,0x11,0x1f,0x1f,0x1f};
const byte LcdSun[8] = {0x4,0x11,0x4,0xe,0x4,0x11,0x4, 0};
const byte LcdBulb[8] = {0xe,0x11,0x11,0x11,0xe,0xe,0xe,0x4};

const int rs = 9, en = 8, d4 = 7, d5 = 6, d6 = 5, d7 = 4;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

//
// Create a SoftwareSerial object so that we can use software serial. Use pins 2 and 3.
// We use software serial as the hw serial port is used for sketch uploads and debug outputs if necessary.
//

SoftwareSerial mySerial(2, 3);

/**
 *  Modbus object declaration
 *  u8id : node id = 0 for master, = 1..247 for slave
 *  port : serial port
 *  u8txenpin : 0 for RS-232 and USB-FTDI 
 *               or any pin number > 1 for RS-485
 */
Modbus master(0, mySerial, 10); // this is master and RS-232 or USB-FTDI via software serial

//
// Global variables used for statistics
//

uint16_t BtStatus, ChStatus;

movingAvg PvPowerAv (4);
movingAvg LdPowerAv (4);
movingAvg LdCurrentAv (4);
movingAvg BtCurrentAv (4);
movingAvg BtVoltageAv (4);

uint32_t ModbusErrors = 0;

//
// This is the task taking care about getting info from the Tracer.
// It's supposed to be called directly from the loop. Authors recommend not to use any delays!
// The code is basically a state machine sending telegram and waiting for response.
// The original code was duplicated to send more telegrams for various purposes (statistic, status etc.).
//

void ModbusTask (void) {
  static uint8_t u8state = 0;
  static unsigned long u32wait = millis ();
  static uint16_t ModbusData [0x10];
  modbus_t Telegram;
  int8_t Status;
  uint32_t u32;

  switch( u8state ) {
  case 0: 
    if (millis() > u32wait) u8state++; // wait state
    break;
  
  case 1: 
    Telegram.u8id = 1; // slave address
    Telegram.u8fct = 4; // function code (this one is registers read)
    Telegram.u16RegAdd = 0x3100; // start address in slave
    Telegram.u16CoilsNo = 0x10; // number of elements (coils or registers) to read
    Telegram.au16reg = ModbusData; // pointer to a memory array in the Arduino
    memset (ModbusData, 0, sizeof (ModbusData));
    master.query (Telegram); // send query (only once)
    u8state++;
    break;

  case 2:
    Status = master.poll(); // check incoming messages
    if (master.getState() == COM_IDLE) {
      u8state++;
      u32wait = millis() + 10; 

      if (Status > 0) {
        PvPowerAv.reading (ModbusData [2]);
        LdPowerAv.reading (ModbusData [0x0E]);
        LdCurrentAv.reading (ModbusData [0x0D]);
        BtCurrentAv.reading (ModbusData [5]);
        BtVoltageAv.reading (ModbusData [4]);
      } else {
        ModbusErrors++;
      }
    }
    break;

  case 3: 
    if (millis() > u32wait) u8state++; 
    break;
  
  case 4: 
    Telegram.u8id = 1;        
    Telegram.u8fct = 4; 
    Telegram.u16RegAdd = 0x3200;
    Telegram.u16CoilsNo = 0x02;
    Telegram.au16reg = ModbusData;
    memset (ModbusData, 0, sizeof (ModbusData));
    master.query (Telegram);
    u8state++;
    break;

  case 5:
    Status = master.poll(); // check incoming messages
    if (master.getState() == COM_IDLE) {
      u8state = 0;
      u32wait = millis() + 1000; 
      if (Status > 0) {
        BtStatus = ModbusData [0];
        ChStatus = ModbusData [1];
      } else {
        ModbusErrors++;
      }
    }
    break;
  }
}

//
// Setup ()
// Initialize all that needs to be intialized, including moving average variables.
//

void setup () {

  //
  // LCD init
  //
  
  lcd.begin (16, 2);
  lcd.noAutoscroll (); 
  lcd.createChar (1, LcdBattery);
  lcd.createChar (2, LcdSun);
  lcd.createChar (3, LcdBulb);

  lcd.home ();
  lcd.print (" EPEVER Tracer ");
  lcd.setCursor (0, 1);
  lcd.print ("   ver. 0.1    ");

  //
  // Serial init. Note we don't really use it but can be handy for debugging
  //
  
  Serial.begin (115200);
  Serial.println ("+-----------------------+");
  Serial.println ("+ EPEVER Tracer monitor +");
  Serial.println ("+       ver 0.1         +");
  Serial.println ("+-----------------------+");

  //
  // Initialize software serial, start Modbus
  //
  
  mySerial.begin (115200);//use the hardware serial if you want to connect to your computer via usb cable, etc.
  master.start (); // start the ModBus object.
  master.setTimeOut (2000); // if there is no answer in 2000 ms, roll over

  //
  // Initialize all moving averages
  //
  
  PvPowerAv.begin ();
  LdPowerAv.begin ();
  LdCurrentAv.begin ();
  BtCurrentAv.begin ();
  BtVoltageAv.begin ();
}

//
// Loop ()
// Main loop is separated to few parts:
//  - code that needs to be executed on every loop
//  - code to execute every millisecond
//  - code to execute every second
//  - code to execute every 10 seconds
// 

void loop () {
  unsigned long Millis = 0;
  static unsigned long LastMillis = 0;
  static unsigned long Seconds = 0;
  static unsigned long Seconds10 = 0;
  char buff [32];
  
  //
  // Any code that needs to be executed on every loop should go right after this comment
  // 

  ModbusTask ();

  // 
  // Any code above is executed on every loop
  //
  
  Millis = millis();
  if (Millis != LastMillis) {
    LastMillis = Millis;
  } else {
    return;
  } 

  //
  // Any code that needs to be executed every millisecond should go right after this comment
  //

  // 
  // Any code above is executed every millisecond
  //
  
  if ((millis () - Seconds) < 1000) {
    return;
  } else {
    Seconds = millis (); 
  }

  //
  // Following section of the loop will be executed once per second
  //

  lcd.clear ();
  lcd.home ();
  sprintf (buff, "%c %4.1fW  %c %4.1fW", 2, (float) PvPowerAv.getAvg () / 100.0, 3, (float) LdPowerAv.getAvg () / 100.0);
  lcd.print (buff);
  lcd.setCursor (0, 1);
  sprintf (buff, "%c %5.2fV  %+5.1fA", 1, (float) BtVoltageAv.getAvg () / 100.0, (float) ((int16_t) BtCurrentAv.getAvg () - (int16_t) LdCurrentAv.getAvg ()) / 100.0);
  lcd.print (buff);
  
  if ((millis () - Seconds10) < 10000) {
    return;
  } else {
    Seconds10 = millis (); 
  }

  //
  // Following section of the loop will be executed every 10 second
  //
}
