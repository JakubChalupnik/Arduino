#include "TimerOne.h" 
#include "led_seg.h"

#define LedAddDot (1 << _P_SEG)
#define LedBlank 0
#define LedDash (seg_hex_table [16])

unsigned char LedDisplay [4];
double pulse, frequency, capacitance, inductance;

void LedInterrupt (void) { 
  static unsigned char Digit = 0;

  PORTC = 0xFF;
  PORTC &= ~(1 << Digit);
  PORTD = ~LedDisplay [Digit];
  Digit = (Digit + 1) & 0x03;
}

void LedPrint (int Val) {

  LedDisplay [3] = seg_hex_table [Val % 10];
  Val /= 10;
  if (Val > 0) {
    LedDisplay [2] = seg_hex_table [Val % 10];
  } 
  else {
    LedDisplay [2] = LedBlank;
  }    
  Val /= 10;
  if (Val > 0) {
    LedDisplay [1] = seg_hex_table [Val % 10];
  } 
  else {
    LedDisplay [1] = LedBlank;
  }
  Val /= 10;
  if (Val > 0) {
    LedDisplay [0] = seg_hex_table [Val % 10];
  } 
  else {
    LedDisplay [0] = LedBlank;
  }

}

void LedDashed (void) {
  LedDisplay [0] = LedDash;
  LedDisplay [1] = LedDash;
  LedDisplay [2] = LedDash;
  LedDisplay [3] = LedDash;
}

void LedDots (void) {
  LedDisplay [0] |= LedAddDot;
  LedDisplay [1] |= LedAddDot;
  LedDisplay [2] |= LedAddDot;
  LedDisplay [3] |= LedAddDot;
}

void setup () {
  byte i;

  for (i = 0; i < 8; i++) {  
    digitalWrite (i, HIGH);
    pinMode(i, OUTPUT);
  }

  for (i = 14; i < 18; i++) {  
    digitalWrite (i, HIGH);
    pinMode(i, OUTPUT);
  }

  Timer1.initialize (4000);
  Timer1.attachInterrupt (LedInterrupt); 
  
  LedDashed ();

//  Serial.begin (115200);
//  pinMode (11, INPUT);
//  pinMode (13, OUTPUT);
//  Serial.println ("[LedInductorMeter]");
  delay (200);
}

void loop () {
  unsigned int Inductance;

  digitalWrite (13, HIGH);
  delay (5);//give some time to charge inductor.
  digitalWrite (13,LOW);
  delayMicroseconds (100); //make sure resination is measured
  pulse = pulseIn (11,HIGH,5000);//returns 0 if timeout
  if (pulse > 0.02){ //if a timeout did not occur and it took a reading:
    capacitance = 2.175E-6; //insert capacitance here. Currently using 2uF
    frequency = 1.E6 / (2 * pulse);
    inductance = 1. / (capacitance * frequency * frequency * 4. * 3.14159 * 3.14159);//one of my profs told me just do squares like this
    inductance *= 1E6; //note that this is the same as saying inductance = inductance*1E6
//    Serial.print ("High for uS:");
//    Serial.print ( pulse );
//    Serial.print ("\tfrequency Hz:");
//    Serial.print ( frequency );
//    Serial.print ("\tinductance uH:");
//    Serial.println ( inductance );

    if (inductance > 9999) {
      Inductance = (unsigned int) ((inductance + 5.) / 10.);
      LedPrint (Inductance);
      LedDots ();
    } else {
      Inductance = (unsigned int) (inductance + .5);
      LedPrint (Inductance);
    }
      
    delay (200);
  } else {
    LedDashed ();
  }
}

