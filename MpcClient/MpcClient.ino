/*********************************************************************
 *
 * Basic MPC client communicating via serial terminal to real mpc
 *
 *********************************************************************
 * FileName:   MpcClient.ino     
 * Processor:  Arduino Mega1280      
 *
 * Author   Date       Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Kubik    2.6.2013   Added file headers and basic T6963 support
 * Kubik    4.6.2013   LCD replaced to regular 2x40
 ********************************************************************/
 
//
// LCD support
//

#include <LiquidCrystal.h>

#define   CONTRAST_PIN   6
#define   BACKLIGHT_PIN  8
#define   CONTRAST       0
LiquidCrystal lcd(7, 14, 5, 4, 3, 2, BACKLIGHT_PIN, POSITIVE );

//
// IR Remote support
//

#include <IRremote.h>
#include <IRremoteInt.h>

const byte RECV_PIN = 22;
IRrecv irrecv(RECV_PIN);
decode_results IrResult;

//
// Defines IR events - what remote and button corresponds to what command
//

typedef struct {
  byte DecodeType;      // NEC, SHARP etc
  unsigned long Value;  // 32bit IR value
  char *Command;        // Command to send
  byte Flag;            // Flags describing details of the specific command
} IrEvent_t; 

#define FE_STOP 0x01
#define FE_PLAY 0x02
#define FE_VOLUME 0x04

IrEvent_t IrEvents[] = {
  // Black multimedia
  {NEC, 0x0687CBCA, "mpc play 1", FE_PLAY},
  {NEC, 0x0687CBF4, "mpc play 2", FE_PLAY},
  {NEC, 0x0687CBCC, "mpc play 3", FE_PLAY},
  {NEC, 0x0687CBEC, "mpc stop", FE_STOP},
  {NEC, 0x0687CBFA, "amixer -c 0 sset Speaker 10%-", FE_VOLUME},
  {NEC, 0x0687CBFE, "amixer -c 0 sset Speaker 10%+", FE_VOLUME},
  {NEC, 0x0687CBFC, "amixer -c 0 sset Speaker 0", FE_VOLUME},
  {NEC, 0x0687CBC2, "amixer -c 0 sset Speaker 50%", FE_VOLUME},
};

#define IrEventsSize (sizeof (IrEvents) / sizeof (IrEvent_t)) 

unsigned int Flags = 0;
#define F_PLAYING 0x01
#define F_MILLIS 0x02
#define F_BUFFER_READY 0x04

#define FlagSet(F) {Flags |= F;}
#define FlagClear(F) {Flags &= ~F;}
#define FlagIsSet(F) (Flags & F)

#define BUFFER_SIZE 64


//
////---------------------------------------------------------------------------
//// Circular buffer support - used to detect the command line prompt
////
//
//#define BUFFER_SIZE 64
//#define R_COMMAND_PROMPT ":/# "
//
//int BufferStart = 0;
//int BufferEnd = 0;
//int BufferActive = 0;
//
//byte CircularBuffer [BUFFER_SIZE];
//
//void PushToQueue(byte p)
//{
//    CircularBuffer[BufferEnd] = p;
//    BufferEnd = (BufferEnd + 1) % BUFFER_SIZE;
//
//    if (BufferActive < BUFFER_SIZE)
//    {
//        BufferActive++;
//    } else {
//        /* Overwriting the oldest. Move start to next-oldest */
//        BufferStart = (BufferStart + 1) % BUFFER_SIZE;
//    }
//}
//
//int SearchString (char *s) {
//    char *p;
//    int i = (BufferEnd + BUFFER_SIZE - 1) % BUFFER_SIZE;
//    int len = strlen(s);
//
//    if (BufferActive < len) {
//        return 0;
//    }
//    p = s + len - 1;
//
//    while (p != s) {
//        if (*p != CircularBuffer[i]) {
//            break;
//        }
//        p--;
//        i = (i + BUFFER_SIZE - 1) % BUFFER_SIZE;
//    }
//
//    return *p == CircularBuffer[i];
//}

//---------------------------------------------------------------------------
// Line read support
//

char SerialBuffer [BUFFER_SIZE];

void MpcGetLine (void) {
  char c;
  byte i;
  
  i = 0;
  while (1) {
    if (!Serial1.available()) {
      continue;
    }
    
    c = Serial1.read();
    if (c == 0x0A) {
      break;
    }
    if (i < (sizeof (SerialBuffer) - 1)) {
      SerialBuffer [i] = c;
      i++;
    }
  }
  
  SerialBuffer [i] = '\0';
}

char* strtrim(char* input) {
    char* start = input;
    while (isspace(*start)) { //trim left
        start++;
    }

    char* ptr = start;
    char* end = start;
    while (*ptr++ != '\0') { //trim right
        if (!isspace(*ptr)) { //only move end pointer if char isn't a space
            end = ptr;
        }
    }

    *end = '\0'; //terminate the trimmed string with a null
    return start;
}

//---------------------------------------------------------------------------
// Setup
//

void setup() {
  char *str;
  
  //
  // Open serial communications and display program name
  //
  
  Serial.begin(115200);
  Serial.println("[MPC client]");
 
  //
  // Enable IR remote receiver.
  //
  
  irrecv.enableIRIn(); 
  
  //
  // Set the data rate for the Serial1 port
  //
  
  Serial1.begin(115200);

  //
  // Enable LCD, set contrast etc. and initialize the display
  //
  
  pinMode(CONTRAST_PIN, OUTPUT);
  analogWrite ( CONTRAST_PIN, CONTRAST );
  
  lcd.backlight();
    
  lcd.begin(40,2);               // initialize the lcd 

  lcd.home ();                   // go home
  lcd.print("MPC client");  
  lcd.setCursor ( 0, 1 );        // go to the next line
  lcd.print ("(c) 2013 Kubik");      
}

//
// Enum describing the states of the MPC communication
//

typedef enum {
  S_BOOTED, 
  S_WAITING,    
  S_WAITING_TIMEOUT,
  S_COMMAND,
  S_PARSE_SONG,
} RouterStatus_t;

void loop() {
  static RouterStatus_t Status = S_BOOTED;
  static char *Command; 
  byte c, i;
  static unsigned long int LastCommand = 0;
  char *TrimmedString;
  static unsigned long int LastMillis = 0;
  unsigned long int Millis = millis ();
  static unsigned int Timeout = 0;
  static byte BufIndex = 0;

  //
  // Detect if millis changed - if yes, set flag for this loop iteration
  //
  
  if (Millis != LastMillis) {
    FlagSet (F_MILLIS);
    LastMillis = Millis;
  }

  //
  // Process the IR input. If any IR code was received,
  // search the whole event list and execute all the events corresponding to the IR code
  //

  if (irrecv.decode(&IrResult)) {
    irrecv.resume(); // Receive the next value
    for (i = 0; i < IrEventsSize; i++) {
      if ((IrResult.decode_type == IrEvents[i].DecodeType) &&
          (IrResult.value == IrEvents[i].Value)) {
           Command = IrEvents[i].Command;
           if (IrEvents[i].Flag & FE_PLAY) {
             FlagSet (F_PLAYING);
           } else if (IrEvents[i].Flag & FE_STOP) {
             FlagClear (F_PLAYING);
           }
      }
    } 
  }




  if (Serial1.available()) {
    c = Serial1.read();
    if (c == 0x0A) {
      SerialBuffer [BufIndex] = '\0';
      FlagSet (F_BUFFER_READY);
      BufIndex = 0;
    } else if (BufIndex < (sizeof (SerialBuffer) - 1)) {
      SerialBuffer [BufIndex] = c;
      BufIndex++;
      FlagClear (F_BUFFER_READY);
    }
  }





  
//  //
//  // If any serial byte is available, push it into the input queue
//  //
//
//  if (Serial1.available ()) {
//    c = Serial1.read ();
//    PushToQueue (c);
//    Serial.write(c);
//  }
//  
  switch (Status) {
    
    //
    // We have just booted, and we better wait for the AVR command prompt after
    //
    
    case S_BOOTED:      
      if (FlagIsSet (F_BUFFER_READY)) {
        if (strstr (SerialBuffer, "AVR Start!")) {
        Status = S_WAITING;
        }
      }
      break;
      
    //
    // Prepare timeout of 5s
    //
  
    case S_WAITING:    // Waiting for any of the messages from 
      Timeout = 5000;
      Status = S_WAITING_TIMEOUT;
      break;
      
    //
    // Wait for either command prompt appearing, or timeout of 5 seconds
    // Use F_MILLIS as this flag is set once per millisecond
    //
    
    case S_WAITING_TIMEOUT:    // Waiting for the command prompt
      if (SearchString (R_COMMAND_PROMPT)) {
        Status = S_COMMAND;
      } else if (FlagIsSet (F_MILLIS)) {
        if (Timeout) {
          Timeout--;
        } else {
          Status = S_WAITING;
          Serial1.write(0x0A);
//          for (i = 0; i < BUFFER_SIZE; i++) {
//            Serial.write(CircularBuffer[i]);
//          }
//          Serial.write('!');
//          Serial.print(BufferStart);
//          Serial.write('!');
//          Serial.print(BufferEnd);
//          Serial.println("!");
        }
      }
      break;
      
    case S_COMMAND:    // System waiting at command prompt, send the command
      if (Command != NULL) {
        Serial1.print(Command);
        Serial1.write(0x0A);
        Command = NULL;
        LastCommand = millis ();
        Status = S_WAITING;
      } else {      // No command pending, get the status every ten seconds
        if ((FlagIsSet (F_PLAYING)) && (millis () > (LastCommand + 10000))) {
          Serial1.print("mpc -f '%title%\\n%name%'");
          Serial1.write(0x0A);
          Status = S_PARSE_SONG;
          LastCommand = millis ();
        } else {
          Status = S_WAITING;
        }
      }
          
      break;
      
    case S_PARSE_SONG:  // Parse the song name
      
      //
      // The first line is just the copy of the command, ignore it.
      // The second line is supposed to be the song name, rest can be skipped for now.
      //
      
      MpcGetLine ();
      MpcGetLine ();
      TrimmedString = strtrim (SerialBuffer);
      if (TrimmedString[0] == '\0') {    // No song name - use station name instead
        MpcGetLine ();
      }

//      Serial.println(SerialBuffer);
      SerialBuffer[40] = 0;
      SerialBuffer[strlen (SerialBuffer) - 1] = 0;
      lcd.clear ();
      lcd.print (SerialBuffer);  
//      lcd.setCursor ( 0, 1 );        // go to the next line
//      lcd.print (LcdSecondLine);  
      Status = S_WAITING;
      break;
  }

  FlagClear (F_MILLIS);
}

