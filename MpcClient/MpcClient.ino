#define BUFFER_SIZE 64

//
// Serial buffer support
//

int BufferStart = 0;
int BufferEnd = 0;
int BufferActive = 0;

byte CircularBuffer [BUFFER_SIZE];

void PushToQueue(byte p)
{
    CircularBuffer[BufferEnd] = p;
    BufferEnd = (BufferEnd + 1) % BUFFER_SIZE;

    if (BufferActive < BUFFER_SIZE)
    {
        BufferActive++;
    } else {
        /* Overwriting the oldest. Move start to next-oldest */
        BufferStart = (BufferStart + 1) % BUFFER_SIZE;
    }
}

int SearchString (char *s) {
    char *p;
    int i = (BufferEnd + BUFFER_SIZE - 1) % BUFFER_SIZE;
    int len = strlen(s);

    if (BufferActive < len) {
        return 0;
    }
    p = s + len - 1;

    while (p != s) {
        if (*p != CircularBuffer[i]) {
            break;
        }
        p--;
        i = (i + BUFFER_SIZE - 1) % BUFFER_SIZE;
    }

    return *p == CircularBuffer[i];
}

//
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


void setup() {
  char *str;
  
  // Open serial communications and wait for port to open:
  Serial.begin(115200);

  Serial.println("MPC client");

  // set the data rate for the SoftwareSerial port
  Serial1.begin(115200);
  
  //
  // Wait for the last message, that is enabling swap, and send Enter to activate the console
  //
  
  while (1) {
    MpcGetLine ();
    str = strstr(SerialBuffer, "swap on ");
    if (str != NULL) {
      break;
    }
  }
  
  Serial1.write(0x0A);    // Activate the console
}

typedef enum {
  S_BOOTED, 
  S_WAITING,    // For command prompt
  S_WAITING1,    // For command prompt
  S_COMMAND,
  S_PARSE_SONG,
} RouterStatus_t;

#define R_COMMAND_PROMPT ":/# "

void loop() {
  static RouterStatus_t Status = S_BOOTED;
  static char *Command = "mpc play 1";
  byte c;
  static unsigned long int LastCommand = 0;
  
  if (Serial1.available ()) {
    PushToQueue (Serial1.read ());
  }
  
  switch (Status) {
    case S_BOOTED:      // We have just booted, and we better wait for the command prompt
//      Serial.write('b');
      Status = S_WAITING;
      break;
      
    case S_WAITING:
////      Serial.write('w');
//      Status = S_WAITING1;
//      break;
    
    case S_WAITING1:
      if (SearchString (R_COMMAND_PROMPT)) {
        Status = S_COMMAND;
      }
      break;
      
    case S_COMMAND:    // System waiting at command prompt, send the command
//      Serial.write('c');
      if (Command != NULL) {
        Serial1.print(Command);
        Serial1.write(0x0A);
        Command = NULL;
        LastCommand = millis ();
//        Serial.write(' ');
//        Serial.println(LastCommand);
        Status = S_WAITING;
      } else {      // No command pending, get the status every five seconds
        if (millis () > (LastCommand + 10000)) {
//          Serial.write('t');
          Serial1.print("mpc -f \"%title%\"");
          Serial1.write(0x0A);
          Status = S_PARSE_SONG;
          LastCommand = millis ();
//          Serial.write(' ');
//          Serial.println(LastCommand);
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
      Serial.println(SerialBuffer);
      Status = S_WAITING;
      break;
  }
}

