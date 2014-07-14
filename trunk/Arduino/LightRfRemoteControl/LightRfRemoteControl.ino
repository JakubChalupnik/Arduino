/* ==============================================================================================
Light controller using RF remote from Pollin
Additionally uses PWM to control the LED strip under the desk,
and eventually an RS-485 interface to control any other lights

Version info:
5.5.2013   - PWM for LED strips support added, changed debouncing code
20.5.2013  - Added PWM for 24V LEDs, support for inverted PWM, and dimming support
             HW has changed a little, pin 9 is now PWM instead of RF remote control, and button pins are swapped
24.5.2013  - RF remote changed to 433MHz sender (major HW change)

Next steps:
 - add SoftSerial support and RS-485 driver, messages will follow the HBUS format / style

// ============================================================================================== */

//
// Define pin for IR remote receiver
//

const byte IrRemotePin = 2;

//
// Define pins for 74HC595
//

const byte SER_Pin = 4;   //pin 14 on the 75HC595
const byte RCLK_Pin = 7;  //pin 12 on the 75HC595
const byte SRCLK_Pin = 8; //pin 11 on the 75HC595

//
// Define pins for buttons A and B
//

const byte ButtonAPin = 17;
const byte ButtonBPin = 16;
const byte ButtonCPin = 15;
const byte ButtonDPin = 19;

//
// Define PWM pins
//

const byte Pwm12VPin = 10;
const byte Pwm24VPin = 9;

//
// Define pin used for 433MHz transmitter
//

const byte RfTransmitterPin = 14;

//
// Pin 13 has an LED connected
//

const byte pin_led = 13;

//
// Define PWM step value for fast PWM inc/dec
//

const byte PwmFastStep = 8;

// ======================================================
// IR support - load library and define used pin. Declare the receiver object and result variable
//

#include <IRremote.h>

IRrecv IrReceive (IrRemotePin);
decode_results IrResult;

// ======================================================
// RF sender support - uses RCSwitch library and 433MHz sender
//

#include <RCSwitch.h>
RCSwitch Rf433Switch = RCSwitch();

// ======================================================
// Define buttons
//

#include <Bounce.h>

typedef struct {
  Bounce Button;
  const byte Pin;
} Button_t;

Button_t Buttons [] = {
  {Bounce (ButtonAPin, 10), ButtonAPin},
  {Bounce (ButtonBPin, 10), ButtonBPin},
  {Bounce (ButtonCPin, 10), ButtonCPin},
  {Bounce (ButtonDPin, 10), ButtonDPin},
};

#define ButtonsSize (sizeof (Buttons) / sizeof (Button_t))

#define BUTTON_A Buttons[0].Button
#define BUTTON_B Buttons[1].Button
#define BUTTON_C Buttons[2].Button
#define BUTTON_D Buttons[3].Button

//
// For button inputs, defines the 'short' press used for turning lights on/off, or for longer presses defines after how long the PWM UP event will be signaled.
// Times are in miliseconds.
//

#define PRESS_SHORT 300L
#define PRESS_INC   10L

// ======================================================
// IR and Light event related defines
//

//
// Defines type of the event - ON for forcing the light on, OFF for off and TOGGLE for toggling the state
// (state info stored in the Lights array)
// PWM_ events manipulate the PWM lights, either increasing/decreasing by one, or by bigger step
//

#define LIGHT_ON 1
#define LIGHT_OFF 2
#define LIGHT_TOGGLE 3
#define LIGHT_PWM_UP 4
#define LIGHT_PWM_UP_FAST 5
#define LIGHT_PWM_DOWN 6
#define LIGHT_PWM_DOWN_FAST 7

//
// Defines type of light - either controlled by RF remote, or by local PWM pin (direct or inverted), or by message via RS-485
//

#define LIGHT_TYPE_RF 1
#define LIGHT_TYPE_PWM 2
#define LIGHT_TYPE_RS485 3
#define LIGHT_TYPE_PWM_INV 4

//
// Defines existing lights - serves as symbolic name for indexes into Lights[]
//

#define LIGHT_ROOM 0
#define LIGHT_UNDER_DESK 1
#define LIGHT_24V_LED 2
#define LIGHT_ALL 0xFF

//
// Defines IR events - what remote and button corresponds to what event
//

typedef struct {
  byte DecodeType;      // NEC, SHARP etc
  unsigned long Value;  // 32bit IR value
  byte LightIndex;      // Index into the Light[] table, 0xFF means all lights
  byte Event;           // Light event type - on, off, toggle
} IrEvent_t;

//
// Defines all available lights. 
// Pin specifies pin controlling the light or message for RS-485 lights
// Status keeps the current status of the particular light
// Type specifies local PWM light, RF remote light or RS-485 based light
//

typedef struct {
  byte Pin;
  byte Status;
  byte Type;
} Light_t;

//
// Stores the information about status of the particular light, and describes how to work with it
// Position of the light in this table is important as it corresponds to LIGHT_ROOM, LIGHT_SWITCH_B etc...
// LIGHT_TYPE_ define describes how to use the light - RF for remote controlled wall sockets, 
// PWM for PWM pins and LIGHT_TYPE_RS485 for message controlled lights
//

Light_t Lights[] = {    
  {0, 0, LIGHT_TYPE_RF},
  {Pwm12VPin, 0, LIGHT_TYPE_PWM},
  {Pwm24VPin, 0, LIGHT_TYPE_PWM_INV},
};

#define LightsSize (sizeof (Lights) / sizeof (Light_t))

// ======================================================
// IR event table - assigns IR codes to light events
//

IrEvent_t IrEvents[] = {
  // Black multimedia
  {NEC, 0x0687CBD2, LIGHT_ROOM, LIGHT_TOGGLE},
  {NEC, 0x0687CBF0, LIGHT_UNDER_DESK, LIGHT_TOGGLE},
  {NEC, 0x0687CBC8, LIGHT_24V_LED, LIGHT_TOGGLE},
  {NEC, 0x0687CBE8, LIGHT_ALL, LIGHT_TOGGLE},
  
  {NEC, 0x0687CBF2, LIGHT_UNDER_DESK, LIGHT_PWM_DOWN_FAST},
  {NEC, 0x0687CBF8, LIGHT_UNDER_DESK, LIGHT_PWM_UP_FAST},
  {NEC, 0x0687CBC4, LIGHT_24V_LED, LIGHT_PWM_UP_FAST},
  {NEC, 0x0687CBE4, LIGHT_24V_LED, LIGHT_PWM_DOWN_FAST},

  // Beige SHARP
  {NEC, 0x14EBC03F, LIGHT_ROOM, LIGHT_TOGGLE},
  {NEC, 0x14EB20DF, LIGHT_UNDER_DESK, LIGHT_TOGGLE},
  {NEC, 0x14EBA05F, LIGHT_ALL, LIGHT_TOGGLE},
  
  // L&S
  {NEC, 0x8AF520DF, LIGHT_ROOM, LIGHT_TOGGLE},
  {NEC, 0x8AF5A05F, LIGHT_UNDER_DESK, LIGHT_TOGGLE},
  {NEC, 0x8AF5609F, LIGHT_ALL, LIGHT_TOGGLE},
  
  {NEC, 0x8AF510EF, LIGHT_24V_LED, LIGHT_TOGGLE},
  {NEC, 0x8AF5906F, LIGHT_24V_LED, LIGHT_PWM_UP_FAST},
  {NEC, 0x8AF550AF, LIGHT_24V_LED, LIGHT_PWM_DOWN_FAST},  
};

#define IrEventsSize (sizeof (IrEvents) / sizeof (IrEvent_t))

// ======================================================
// 74HC595 support
//

void ShiftOutByte (byte b) {
  byte i;

  digitalWrite(RCLK_Pin, LOW);
  digitalWrite(SRCLK_Pin, LOW);

  for (i = 0; i < 8; i++) {
    digitalWrite(SER_Pin, (b & 0x01) ? HIGH : LOW);
    b >>= 1;
    digitalWrite(SRCLK_Pin, HIGH);
    digitalWrite(SRCLK_Pin, LOW);
  }
  digitalWrite(RCLK_Pin, HIGH);
  digitalWrite(RCLK_Pin, LOW);
}

// ======================================================
// RF communication routines
// 'Pin' is index of the light description in the RfLights table 
//

typedef struct {
  char *Group;
  char *Switch;
} RfLight_t;

RfLight_t RfLights [] = {
  "10001", "10000",
};
#define RfLightsSize (sizeof (RfLights) / sizeof (RfLight_t))

void RfSendOn (byte Pin) {

  Rf433Switch.switchOn(RfLights[Pin].Group, RfLights[Pin].Switch);
  digitalWrite(pin_led, HIGH);
}

void RfSendOff (byte Pin) {

  Rf433Switch.switchOff(RfLights[Pin].Group, RfLights[Pin].Switch);
  digitalWrite(pin_led, LOW);
}

// ======================================================
// Light events processing - send single event, common event...
// Takes care about various types of lights, RF, PWM inverted PWM and RS-485
//

void ProcessSingleLightEvent (byte Index, byte Event) {
  
  switch (Event) {
    case LIGHT_ON:                      // Force the light on
      if (Lights[Index].Type == LIGHT_TYPE_RF) {
        RfSendOn (Lights[Index].Pin);
        Lights[Index].Status = 1;
      } else if (Lights[Index].Type == LIGHT_TYPE_PWM) {
        analogWrite (Lights[Index].Pin, 255);
        Lights[Index].Status = 255;
      } else if (Lights[Index].Type == LIGHT_TYPE_PWM_INV) {
        analogWrite (Lights[Index].Pin, 0);
        Lights[Index].Status = 255;
      }
      break;
    
    case LIGHT_OFF:                      // Force the light off
      if (Lights[Index].Type == LIGHT_TYPE_RF) {
        RfSendOff (Lights[Index].Pin);
      } else if (Lights[Index].Type == LIGHT_TYPE_PWM) {
        analogWrite (Lights[Index].Pin, 0);
      } else if (Lights[Index].Type == LIGHT_TYPE_PWM_INV) {
        analogWrite (Lights[Index].Pin, 255);
      }
      Lights[Index].Status = 0;
      break;
    
    case LIGHT_TOGGLE:                  // Toggle the light 
      if (Lights[Index].Status) {
        if (Lights[Index].Type == LIGHT_TYPE_RF) {
          RfSendOff (Lights[Index].Pin);
        } else if (Lights[Index].Type == LIGHT_TYPE_PWM) {
          analogWrite (Lights[Index].Pin, 0);
        } else if (Lights[Index].Type == LIGHT_TYPE_PWM_INV) {
          analogWrite (Lights[Index].Pin, 255);
        }
        Lights[Index].Status = 0;
      } else {
        if (Lights[Index].Type == LIGHT_TYPE_RF) {
          RfSendOn (Lights[Index].Pin);
          Lights[Index].Status = 1;
        } else if (Lights[Index].Type == LIGHT_TYPE_PWM) {
          analogWrite (Lights[Index].Pin, 255);
          Lights[Index].Status = 255;
        } else if (Lights[Index].Type == LIGHT_TYPE_PWM_INV) {
          analogWrite (Lights[Index].Pin, 0);
          Lights[Index].Status = 255;
        }
      }
      break;
    
    case LIGHT_PWM_UP:                  // Dimming support - increases PWM by one
      if (Lights[Index].Status < 255) {
        if (Lights[Index].Type == LIGHT_TYPE_PWM) {
          Lights[Index].Status++;
          analogWrite (Lights[Index].Pin, Lights[Index].Status);
        } else if (Lights[Index].Type == LIGHT_TYPE_PWM_INV) {
          Lights[Index].Status++;
          analogWrite (Lights[Index].Pin, 255 - Lights[Index].Status);
        }
      }
      break;

    case LIGHT_PWM_UP_FAST:              // Dimming support - increases by PwmFastStep
      if (Lights[Index].Status == 255) {
        break;
      }
        
      if (Lights[Index].Status > (255 - PwmFastStep)) {
        Lights[Index].Status = 255 - PwmFastStep;
      }
        
      if (Lights[Index].Status <= (255 - PwmFastStep)) {
        if (Lights[Index].Type == LIGHT_TYPE_PWM) {
          Lights[Index].Status += PwmFastStep;
          analogWrite (Lights[Index].Pin, Lights[Index].Status);
        } else if (Lights[Index].Type == LIGHT_TYPE_PWM_INV) {
          Lights[Index].Status += PwmFastStep;
          analogWrite (Lights[Index].Pin, 255 - Lights[Index].Status);
        }
      }
      break;

    case LIGHT_PWM_DOWN:                // Dimming support - decreases PWM by one
      if (Lights[Index].Status > 0) {
        if (Lights[Index].Type == LIGHT_TYPE_PWM) {
          Lights[Index].Status--;
          analogWrite (Lights[Index].Pin, Lights[Index].Status);
        } else if (Lights[Index].Type == LIGHT_TYPE_PWM_INV) {
          Lights[Index].Status--;
          analogWrite (Lights[Index].Pin, 255 - Lights[Index].Status);
        }
      }
      break;

    case LIGHT_PWM_DOWN_FAST:           // Dimmind support - decreases PWM by PwmFastStep
      if (Lights[Index].Status == 0) {
        break;
      }

      if (Lights[Index].Status < PwmFastStep) {
        Lights[Index].Status = PwmFastStep;
      }
        
      if (Lights[Index].Status >= PwmFastStep) {
        if (Lights[Index].Type == LIGHT_TYPE_PWM) {
          Lights[Index].Status -= PwmFastStep;
          analogWrite (Lights[Index].Pin, Lights[Index].Status);
        } else if (Lights[Index].Type == LIGHT_TYPE_PWM_INV) {
          Lights[Index].Status -= PwmFastStep;
          analogWrite (Lights[Index].Pin, 255 - Lights[Index].Status);
        }
      }
      break;

    default:
      break;
  }
}

void ProcessLightEvent (byte Index, byte Event) {
  byte i;
  byte Toggle = 0;

  //
  // For "Toggle All Lights", search through all the lights and see if any is on
  //
  
  if ((Index == LIGHT_ALL) && (Event == LIGHT_TOGGLE)) {    
    for (i = 0; i < LightsSize; i++) {                  
      if (Lights[i].Status) {                          // Any light is on? If yes, set the flag
        Toggle = 1;
      }
    }
    
    if (Toggle) {
      Event = LIGHT_OFF;
    } else {
      Event = LIGHT_ON;
    }
  }
  
  if (Index == LIGHT_ALL) {
    for (i = 0; i < LightsSize; i++) {                  
      ProcessSingleLightEvent (i,  Event);
    }
  } else {
    ProcessSingleLightEvent (Index,  Event);
  }
}

// ======================================================
// Setup routine initializes hardware
//

void setup() {
  byte i;

  Serial.begin(57600);
  Serial.println("\n[LightRfRemoteControl]\n");
  //
  // Initialize 433MHz sender connected to Arduino Pin #14 (A0)
  //

  Rf433Switch.enableTransmit(RfTransmitterPin);

  //
  // Init and switch off the LED
  //

  digitalWrite(pin_led, LOW);
  pinMode(pin_led, OUTPUT);

  //
  // Start the IR receiver
  //

  IrReceive.enableIRIn();

  //
  // Prepare button pins - set them as input and enable pull ups
  //

  for (i = 0; i < ButtonsSize; i++) {
    pinMode(Buttons [i].Pin, INPUT_PULLUP);
  }

  //
  // Configure PWM pins
  //

  pinMode(Pwm12VPin, OUTPUT);
  analogWrite(Pwm12VPin, 0);

  pinMode(Pwm24VPin, OUTPUT);
  analogWrite(Pwm24VPin, 255);

  //
  // Configure 74HC595 pins as outputs
  //

  digitalWrite(SER_Pin, LOW);
  digitalWrite(RCLK_Pin, LOW);
  digitalWrite(SRCLK_Pin, LOW);

  pinMode(SER_Pin, OUTPUT);    // data
  pinMode(RCLK_Pin, OUTPUT);  // latch
  pinMode(SRCLK_Pin, OUTPUT);  // clock

  ShiftOutByte (0x00);
}

// ======================================================
// Main loop routine
//

void loop() {
  int i;
  static unsigned long int last = 0;

  //
  // Update the status of debounced buttons
  //

  for (i = 0; i < ButtonsSize; i++) {  
    Buttons[i].Button.update ();
  }

  //
  // Process button press events - we only care about the rising edge event
  //

  if (BUTTON_A.fallingEdge ()) {
    last = millis ();
  }

  if (BUTTON_A.risingEdge ()) {
    if ((millis () - last) < PRESS_SHORT) {  // just a short button press, toggle the light
      ProcessLightEvent (LIGHT_UNDER_DESK, LIGHT_TOGGLE);
    }
  }
  
  if (!BUTTON_A.read ()) {    // Button still pressed
    if ((millis () - last) >= PRESS_SHORT) {
      if ((millis () - last) >= (PRESS_SHORT + PRESS_INC)) {
        ProcessLightEvent (LIGHT_UNDER_DESK, LIGHT_PWM_UP);
        last += PRESS_INC;
      }
    }
  }
  
  if (BUTTON_B.risingEdge ()) {
    ProcessLightEvent (LIGHT_ROOM, LIGHT_TOGGLE);
  }
  
  //
  // Process IR remote input
  //
  
  if (!IrReceive.decode(&IrResult)) {        // No IR code received?
    return;
  }
  
  IrReceive.resume ();

  //
  // Search the whole event list and execute all the events corresponding to the IR code
  //
  
  for (i = 0; i < IrEventsSize; i++) {
    if ((IrResult.decode_type == IrEvents[i].DecodeType) &&
        (IrResult.value == IrEvents[i].Value)) {
         ProcessLightEvent (IrEvents[i].LightIndex, IrEvents[i].Event);
    }
  }
}

