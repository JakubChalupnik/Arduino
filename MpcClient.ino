
char SerialBuffer [100];

void MpcGetLine (void) {
  char c;
  byte i;
  
  i = 0;
  while (1) {
    if (!Serial1.available()) {
      continue;
    }
    
    c = Serial1.read();
//    Serial.print(c);
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
  
  Serial1.write(0x0A);
//  Serial.println("Found entry");  
}

typedef enum {
  S_BOOTED, 
  S_WAITING,    // For command prompt
  S_COMMAND,
} RouterStatus_t;

#define R_COMMAND_PROMPT "root@OpenWrt:/#"

void loop() {
  static RouterStatus_t Status = S_BOOTED;
  static char *Command = "mpc play 1";
  byte c;
  static unsigned long int LastCommand = 0;
  
  if (Serial1.available ()) {
    c = Serial1.read ();
  }
  
  switch (Status) {
    case S_BOOTED:      // We have just booted, and we better wait for the command prompt
      Status = S_WAITING;
      break;
      
    case S_WAITING:
      if (c == '#') {
        Status = S_COMMAND;
      }
      break;
      
    case S_COMMAND:    // System waiting at command prompt, send the command
      if (Command != NULL) {
        Serial1.print(Command);
        Serial1.write(0x0A);
        Command = NULL;
        LastCommand = millis ();
      } else {      // No command pending, get the status every five seconds
        if (millis () > (LastCommand + 5000)) {
          Serial1.print("mpc -f \"%title%\"");
          Serial1.write(0x0A);
          LastCommand = millis ();
        }
      }
          
      Status = S_WAITING;
      break;
  }
      
}

