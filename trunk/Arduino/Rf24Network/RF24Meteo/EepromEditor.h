//*******************************************************************************
//*                               printf support                                *
//*******************************************************************************

int serial_putc (char c, FILE *) {

  Serial.write (c);
  return c;
} 

void PrintfInit (void) {

  fdevopen (&serial_putc, 0);
} 

//*******************************************************************************
//*                               VT100 support                                 *
//*******************************************************************************

#define VT100_ATTR_OFF	        0
#define VT100_BOLD	        1
#define VT100_USCORE	        4
#define VT100_BLINK	        5
#define VT100_REVERSE	        7
#define VT100_BOLD_OFF	        21
#define VT100_USCORE_OFF	24
#define VT100_BLINK_OFF	        25
#define VT100_REVERSE_OFF	27
 
void Vt100Init (void) {

  printf_P (PSTR("\x1B\x63"));
}

void Vt100Cls (void) {

  printf_P (PSTR("\x1B[2J"));
}

void Vt100SetAttr (uint8_t Attr) {

  printf_P (PSTR("\x1B[%dm"), Attr);
}

void Vt100GotoXY (uint8_t X, uint8_t Y) {

  printf_P (PSTR("\x1B[%d;%dH"), Y, X);
}

void Vt100ShowCursow (bool Visible) {

  if (Visible) {
    printf_P (PSTR("\x1B[?25h"));
  } else {
    printf_P (PSTR("\x1B[?25l"));
  }
}

//*******************************************************************************
//*                            Keyboard input support                           *
//*******************************************************************************

void PutChar (char c) {
  Serial.write (c);
}

bool IsChar (void) {
  
  return Serial.available ();
}

char GetChar (void) {
  int c;

  while (!Serial.available ());
  c = Serial.read ();
  if (c == -1) {
    printf_P (PSTR("Panic! Nothing read from Serial"));
    while (1);
  }
  return (char) c;
}

char PeekChar (void) {
  int c;

  while (!Serial.available ());
  c = Serial.peek ();
  if (c == -1) {
    printf_P (PSTR("Panic! Nothing can be peeked at in Serial"));
    while (1);
  }
  return (char) c;
}

//*******************************************************************************
//*                             Editor support                                  *
//*******************************************************************************

void DisplayFrame (byte x, byte y, byte width, byte height) {
  byte i;

  width--;
  height--;
  
  //
  // Upper line
  //
  
  Vt100GotoXY (x, y);
  putchar ('+');
  for (i = 1; i < width; i++) {
    putchar ('-');
  }
  putchar ('+');
  
  //
  // Side lines
  //
  
  for (i = 1; i < height; i++) {
    Vt100GotoXY (x, y + i);
    putchar ('|');
    Vt100GotoXY (x + width, y + i);
    putchar ('|');
  }
  
  //
  // Lower line
  //
  
  Vt100GotoXY (x, y + height);
  putchar ('+');
  for (i = 1; i < width; i++) {
    putchar ('-');
  }
  putchar ('+');
}  
  
void DisplayEeprom (void) {
  char Buffer[NODE_ID_SIZE + 1];

  Vt100GotoXY (3, 2);
  printf_P (PSTR("Id: %c%c"), (char) (Eeprom.SensorId >> 8), (char) (Eeprom.SensorId & 0xFF));
  
  Vt100GotoXY (3, 3);
  switch (Eeprom.Flags & F_BATTERY_MASK) {
    case F_BATTERY_NONE:
      printf_P (PSTR("No batt"));
      break;
    case F_BATTERY_CR2032:
      printf_P (PSTR("CR2032 "));
      break;
    case F_BATTERY_LIION:
      printf_P (PSTR("LiIon  "));
      break;
    case F_BATTERY_SOLAR:
      printf_P (PSTR("Solar  "));
      break;
    default:
      printf_P (PSTR("Unknown"));
      break;
  }

  printf_P (PSTR("  %d"), Eeprom.BatteryCoefficient);

  Vt100GotoXY (3, 4);
  printf_P (PSTR("MeasurePer %2d"), Eeprom.MeasurementPeriod);
}

bool GetString (char *Buffer, int Size) {
  byte p;
  char c;
  
  if ((Size <= 0) || (Buffer == NULL)) {
    return false;
  }
  
  memset (Buffer, ' ', Size);  
  
  p = 0;
  while (1) {
    c = GetChar ();
    if ((c >= ' ') && (c < 127)) {      // ASCII character
      if (p < (Size)) {
        Buffer [p] = c;
        PutChar (c);
        p++;
      }
    } else if ((c == 127) || (c == 8)) {              // Backspace
      if (p > 0) {
        p--;
        Buffer [p] = ' ';
        PutChar (c);
      }        
    } else if (c == 0x1B) {              // Esc
      return false;
    } else if (c == 0x0D) {              // Enter
      return true;
    }
  }   
}

void DisplayMenu (void) {
  
  Vt100GotoXY (1, 6);
  printf_P (PSTR("(I)dentification (B)attery (C)oef"));
  Vt100GotoXY (1, 7);
  printf_P (PSTR("(M)easurePeriod"));
  Vt100GotoXY (1, 8);
  printf_P (PSTR("(W)rite (R)evert (Q)uit"));
} 

//*******************************************************************************
//*                             Editor main function                            *
//*******************************************************************************

void EepromEditor (void) {
  char c;
  char Buffer[32];
  uint16_t Word;
  byte i;

  while (1) {
    Vt100ShowCursow (false);
    Vt100Cls ();  
    DisplayFrame (1, 1, 25, 5);
    DisplayEeprom ();
    DisplayMenu ();
    
    c = GetChar ();
    
    switch (c) {
      case 'b':
      case 'B':
        if ((Eeprom.Flags & F_BATTERY_MASK) == F_BATTERY_NONE) {
          Eeprom.Flags &= ~F_BATTERY_MASK;
          Eeprom.Flags |= F_BATTERY_CR2032;
        } else if ((Eeprom.Flags & F_BATTERY_MASK) == F_BATTERY_CR2032) {
          Eeprom.Flags &= ~F_BATTERY_MASK;
          Eeprom.Flags |= F_BATTERY_LIION;
        } else if ((Eeprom.Flags & F_BATTERY_MASK) == F_BATTERY_LIION) {
          Eeprom.Flags &= ~F_BATTERY_MASK;
          Eeprom.Flags |= F_BATTERY_SOLAR;
        } else if ((Eeprom.Flags & F_BATTERY_MASK) == F_BATTERY_SOLAR) {
          Eeprom.Flags &= ~F_BATTERY_MASK;
          Eeprom.Flags |= F_BATTERY_NONE;
        }
        break;
          
      case 'i':
      case 'I':
        Vt100GotoXY (1, 9);
        printf_P (PSTR("New ID (2 chars): "));
        Vt100ShowCursow (true);
        if (GetString (Buffer, 2)) { // Successfull entry finished by <Enter>
          Eeprom.SensorId = Buffer[0] << 8 | Buffer [1];
        }
        Vt100ShowCursow (false);
        break;
  
      case 'c':
      case 'C':
        Vt100GotoXY (1, 9);
        printf_P (PSTR("New coefficient (decimal): "));
        Vt100ShowCursow (true);
        if (GetString (Buffer, 4)) { // Successfull entry finished by <Enter>
          Buffer [NODE_ID_SIZE] = 0;
          sscanf (Buffer, "%d", &Word);
          Eeprom.BatteryCoefficient = Word;
        }
        Vt100ShowCursow (false);
        break;
  
      case 'm':
      case 'M':
        Vt100GotoXY (1, 9);
        printf_P (PSTR("New measurement period (in units of 8 seconds): "));
        Vt100ShowCursow (true);
        if (GetString (Buffer, 4)) { // Successfull entry finished by <Enter>
          Buffer [NODE_ID_SIZE] = 0;
          sscanf (Buffer, "%d", &Word);
          Eeprom.MeasurementPeriod = Word;
        }
        Vt100ShowCursow (false);
        break;
  
      case 'w':
      case 'W':
        EepromWrite ();
        break;
  
      case 'r':
      case 'R':
        EepromRead ();
        break;
  
      case 'q':
      case 'Q':
        Vt100Cls ();
        return;
  
      default:
        break;
    }    
  }
}

