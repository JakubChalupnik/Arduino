#include <SoftwareSerial.h>
#include <avr/pgmspace.h>
#define CIELPWM(a) (pgm_read_word_near(CIEL8 + a)) // CIE Lightness loopup table function
//#define SetBrightness(pin, val) analogWrite (pin, pgm_read_word_near(CIEL8 + 31 - (val & 0x1F)))
#define SetBrightness(pin, val) analogWrite (pin, 255 - val)

#define RS485_R 8
#define RS485_D 7
#define RS485_RE_DE 2
#define PWM_350MA 3
#define PWM_1A 5
#define PWM_12V_1 6
#define PWM_12V_2 9


byte Desired1A = 0;
byte Current1A = 0;
byte Desired350mA = 0;
byte Current350mA = 0;
byte Desired12V_1 = 0;
byte Current12V_1 = 0;
byte Desired12V_2 = 0;
byte Current12V_2 = 0;

SoftwareSerial mySerial(RS485_R, RS485_D);

prog_uint8_t CIEL8[] PROGMEM = {
    0,    1,    2,    3,    4,    5,    7,    9,    12,
    15,    18,    22,    27,    32,    38,    44,    51,    58,
    67,    76,    86,    96,    108,    120,    134,    148,    163,
    180,    197,    216,    235,    255
};

typedef struct {
  unsigned long code;
  byte brightness;
} event_t;

event_t EventTable[] = {
  {0x8AF58877, 0},
  {0x8AF520DF, (31 * 1 + 4) / 9},
  {0x8AF5A05F, (31 * 2 + 4) / 9},
  {0x8AF5609F, (31 * 3 + 4) / 9},
  {0x8AF510EF, (31 * 4 + 4) / 9},
  {0x8AF5906F, (31 * 5 + 4) / 9},
  {0x8AF550AF, (31 * 6 + 4) / 9},
  {0x8AF530CF, (31 * 7 + 4) / 9},
  {0x8AF5B04F, (31 * 8 + 4) / 9},
  {0x8AF5708F, 31},
};

//==========================================================
//
// Button support
//

#define PIN_KEY_1 16
#define PIN_KEY_2 14
#define PIN_KEY_3 17
#define PIN_KEY_4 15

#define KEY_1 0x01
#define KEY_2 0x02
#define KEY_3 0x04
#define KEY_4 0x08

#define DELAY_DEBOUNCE 30
#define DELAY_REPEAT_START 240
#define DELAY_REPEAT 120

byte key = 0;

byte KeyRead (void ) {
  byte key = 0;

  if (digitalRead (PIN_KEY_1) == LOW) {
     key |= KEY_1;
  }  

  if (digitalRead (PIN_KEY_2) == LOW) {
     key |= KEY_2;
  }  
  
  if (digitalRead (PIN_KEY_3) == LOW) {
     key |= KEY_3;
  }  

  if (digitalRead (PIN_KEY_4) == LOW) {
     key |= KEY_4;
  }  
  
  return key;
}

void UpdateKey (void) {
  uint8_t c;
  static uint8_t id = 0x00;
  static uint8_t key_counter = 0;
  static long lasttime = 0;

  if (millis() == lasttime) {
    return;
  }

  lasttime = millis();

  c = KeyRead();
  if(c == 0) {                    // No key pressed
      id = 0;
      key_counter = 0;
  } else if(c != id) {            // New key differs from the previous one
      id = c;
      key_counter = 0;
  } else {                        // New key is the same as previous one
      key_counter++;
  }
  if(key_counter == DELAY_DEBOUNCE) {     // Debouncing complete - set key pressed
      key = id;
  } else if(key_counter == DELAY_REPEAT_START) {  // Repeated key
      key = id;
      key_counter -= DELAY_REPEAT;
  }
}

//==========================================================
//
// Setup
//

void setup() {
  
  Serial.begin(57600);
  Serial.println("[Monitor Stand Light Controller 1]");

  //
  // set the data rate for the SoftwareSerial port
  //
  
  mySerial.begin(9600);
  pinMode(RS485_RE_DE, OUTPUT);
  digitalWrite(RS485_RE_DE, LOW);
  
  //
  // clear all the global variables - all PWM's to 0
  //
  
  analogWrite (PWM_350MA, 255);
  analogWrite (PWM_1A, 255);
  analogWrite (PWM_12V_1, 255);
  analogWrite (PWM_12V_2, 255);

  //
  // Make input & enable pull-up resistors on switch pins
  //
  
  pinMode (PIN_KEY_1, INPUT);
  digitalWrite (PIN_KEY_1, HIGH);
  pinMode (PIN_KEY_2, INPUT);
  digitalWrite (PIN_KEY_2, HIGH);
  pinMode (PIN_KEY_3, INPUT);
  digitalWrite (PIN_KEY_3, HIGH);
  pinMode (PIN_KEY_4, INPUT);
  digitalWrite (PIN_KEY_4, HIGH);
  
//  TCCR2B = TCCR2B & 0b11111000 | 0x01;
}

//==========================================================
//
// Main loop
//

void loop() {
  static byte i = 0;
  static long int lasttime = 0;
  static long int lasttime10 = 0;
  
  UpdateKey ();
  
  //
  // Detect what key is pressed and set the desired light intensity accordingly
  //
  
#define LED_HIGH 255
#define LED_LOW 7

  if (key & KEY_1) {
    if (Desired350mA == 0) {
      Desired350mA = 127;
    } else {
      Desired350mA = 0;
    }
  }
  
  if (key & KEY_2) {
    if (Desired1A == 0) {
      Desired1A = LED_LOW;
    } else if (Desired1A == LED_LOW) {
      Desired1A = LED_HIGH;
    } else {
      Desired1A = 0;
    }
  }
  
  key = 0;
 
  if (lasttime == millis ()) {
    return;
  }
  
  lasttime = millis ();
  
  //
  // Following section of the loop will be executed once per millisecond
  //

  if ((lasttime10 + 10) > millis ()) {
    return;
  }
  
  lasttime10 = millis ();
  
  //
  // Rest of the loop will be executed once per 10 milliseconds
  //
  
  if (Desired350mA > Current350mA) {
    Current350mA++;
    SetBrightness(PWM_350MA, Current350mA);
  } else if (Desired350mA < Current350mA) {
    Current350mA--;
    SetBrightness(PWM_350MA, Current350mA);
  }
 
  if (Desired1A > Current1A) {
    Current1A++;
    SetBrightness(PWM_1A, Current1A);
  } else if (Desired1A < Current1A) {
    Current1A--;
    SetBrightness(PWM_1A, Current1A);
  }
 
}
