//*******************************************************************************
//*
//* Arduino based low power sensor unit using nRF24L01+ module
//*
//*******************************************************************************
//* Processor:  Arduino Pro Mini 3.3V/8MHz
//* Author      Date       Comment
//*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//* Kubik       11.1.2015 First version, just basic code for HW tests
//*******************************************************************************

//*******************************************************************************
//*                            HW details                                       *
//*******************************************************************************
// Standard NRF24L01+ module (eBay)
//
// Used pins:
//  NRF24L01+       SPI + 9, 10
//

//*******************************************************************************
//*                           Includes and defines                              *
//******************************************************************************* 

#define DEBUG 0

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "LowPower.h"

#if DEBUG
#include "printf.h"
#endif

//
// NRF24L01 related defines
//

#define RF24_BROADCAST_ADDR 0xE8E8F0F0E1LL
#define RF24_BROADCAST_PIPE 0

//*******************************************************************************
//*                           Data types                                        *
//******************************************************************************* 

typedef struct {
  uint8_t BattLevel;
  uint16_t Temp [5];
  uint8_t padding [5];
} Payload_t;

//*******************************************************************************
//*                               Static variables                              *
//*******************************************************************************
 
//
// RF24 variables
//

RF24 Rf24Radio (9,10);
const uint64_t BroadcastPipe = RF24_BROADCAST_ADDR;
Payload_t Payload;

//*******************************************************************************
//*                            Arduino setup method                             *
//******************************************************************************* 

void setup(void) {

  Serial.begin (57600);
  Serial.println ("[RF24_Sensor]");
#if DEBUG
  printf_begin ();
#endif

  //
  // Setup and configure rf radio
  //

  Rf24Radio.begin();

  //
  // Open broadcast pipe to other nodes for communication. 
  // This is just a test, real sensor will use dedicated pipe to send the value to the master.
  // Set the noAck, broadcasts cannot be acked.
  //

  Rf24Radio.openWritingPipe (BroadcastPipe);
  Rf24Radio.setAutoAck (false);

#if DEBUG
  //
  // Dump the configuration of the rf unit for debugging
  //

  Rf24Radio.printDetails();
  delay (1000);
#endif

  pinMode (8, OUTPUT);
  digitalWrite (8, LOW);

  pinMode (7, OUTPUT);
  digitalWrite (7, LOW);
  analogReference (INTERNAL);
}

//*******************************************************************************
//*                              Main program loop                              *
//******************************************************************************* 

void loop (void) {
  bool Ok;
  uint16_t BattLevel;
  
  LowPower.powerDown (SLEEP_8S, ADC_OFF, BOD_OFF);    

  digitalWrite (8, HIGH);
  memset (&Payload, 0x00, sizeof (Payload));
  
  //
  // Analog reference is 1.1V, divider is 1M / M33 -> 4.43V max -> 4.33mV per one
  // 2V corresponds to 462 - let's deduct this first.
  // Then, convert what remained to tens mV - multiply that by 0.433 (*13 /30)
  //
  
  
  BattLevel = analogRead (A0) - 462;
  BattLevel = BattLevel * 13 / 30;
    
#if DEBUG
  printf ("Battery level %d\n", BattLevel);
  Serial.print ("Now sending...");
#endif
  
  Payload.BattLevel = (uint8_t) BattLevel;
  Ok = Rf24Radio.write ((uint8_t *) &Payload, 8);

  if (Ok) {
#if DEBUG
    Serial.println ("ok");
    delay (100);
#endif    
  } else {
    digitalWrite (7, HIGH);
    delay (20);      
    digitalWrite (7, LOW);
#if DEBUG
    Serial.println ("failed");
    delay (100);
#endif    
  }

  digitalWrite (8, LOW);
}

