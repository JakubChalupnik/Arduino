//*******************************************************************************
//*                           Includes and defines                              *
//*******************************************************************************  

#include <EEPROM.h>
//#include "printf.h"

typedef struct {
  uint32_t Crc;
  uint16_t Header;
  char Id[8];
  uint16_t NodeAddress;
  uint16_t Flags;
} Eeprom_t;

#define F_DEFAULTS 0x8000 

//*******************************************************************************
//*                               Static variables                              *
//*******************************************************************************
 
//
// EEPROM variables
//

Eeprom_t EepromContent;

int serial_putc( char c, FILE * ) 
{
  Serial.write( c );

  return c;
} 

void printf_begin(void)
{
  fdevopen( &serial_putc, 0 );
}

//*******************************************************************************
//*                                  CRC32 support                              *
//*******************************************************************************

#include <avr/pgmspace.h>

static PROGMEM prog_uint32_t CrcTable[16] = {
    0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
    0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
    0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
    0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
};

uint32_t CrcUpdate (uint32_t Crc, uint8_t Data) {
    byte TblIdx;
    TblIdx = Crc ^ (Data >> (0 * 4));
    Crc = pgm_read_dword_near(CrcTable + (TblIdx & 0x0f)) ^ (Crc >> 4);
    TblIdx = Crc ^ (Data >> (1 * 4));
    Crc = pgm_read_dword_near(CrcTable + (TblIdx & 0x0f)) ^ (Crc >> 4);
    return Crc;
}

uint32_t CrcBuffer (byte *Buffer, byte Size) {

  uint32_t Crc = ~0L;
  while (Size > 0) {
    Crc = CrcUpdate (Crc, *Buffer++);
    Size--;
  }
  Crc = ~Crc;
  return Crc;
}

//*******************************************************************************
//*                                  EEPROM support                             *
//*******************************************************************************

//
// Read EEPROM from address 0 to EepromContent. 
// Check the CRC, compare it with stored CRC.
// If it does not match, load EepromContent with defaults.
//

bool EepromRead (void) {
  byte i;
  byte *EepromPtr;
  uint32_t Crc;

  EepromPtr = (byte *) &EepromContent;
  for (i = 0; i < sizeof (Eeprom_t); i++) {
    *EepromPtr++ = EEPROM.read (i);
  }
  
  EepromPtr = (byte *) &EepromContent;
  Crc = CrcBuffer (EepromPtr + 4, sizeof (Eeprom_t) - 4);    // Do not include stored CRC into CRC calculation
  if (Crc != EepromContent.Crc) {
    EepromContent.Flags = F_DEFAULTS;
    EepromContent.Header = 'ID';
    EepromContent.NodeAddress = 05;
    memcpy (EepromContent.Id, "TempNode", sizeof (EepromContent.Id));
    return false;
  }
  return true;
}
  
//*******************************************************************************
//*                            Arduino setup method                             *
//******************************************************************************* 

void setup () {
  byte i;
  
  Serial.begin (57600);
  Serial.println (F("[RF24NodeEepromWrite]"));

  printf_begin ();
  
  //
  // Read EEPROM to get the node address, ID and other stuff
  //


//  uint32_t Crc;
//  uint16_t Header;
//  char Id[8];
//  uint16_t NodeAddress;
//  uint16_t Flags;
  
  if (EepromRead ()) {
    printf_P (PSTR("EEPROM read OK\n"));
  } else {  
    printf_P (PSTR("EEPROM read failed, default address 005 assigned\n"));
  }    
}



int getch (void) {
  
  while (!Serial.available ());
  return Serial.read ();
}

//void StringRead (char *String, byte Size) {
//  byte Index = 0;
//  int inChar=-1;
//  
//  while (Serial.available()) {
//    if(Index < (Size - 1)) { // One less than the size of the array
//      inChar = Serial.read(); // Read a character
//      String[Index] = inChar; // Store it
//      Index++; // Increment where to write next
//      String[Index] = '\0'; // Null terminate the string
//    }
//  }
//}


void StringRead (char *String, byte Size) {
  byte Index = 0;
  int inChar=-1;
  
  while (Serial.available()) {
    if(Index < (Size - 1)) { // One less than the size of the array
      inChar = Serial.read(); // Read a character
      String[Index] = inChar; // Store it
      Index++; // Increment where to write next
      String[Index] = '\0'; // Null terminate the string
    }
  }
}


int readSerial(char result[])
{
  int i = 0;
  while(1)
  {
    while (Serial.available() > 0)
    {
      char inChar = Serial.read();
      if (inChar == '\n')
      {
        result[i] = '\0';
        Serial.flush();
        return 0;
      }
      if(inChar!='\r')
      {
        result[i] = inChar;
        i++;
      }
    }
  }
} 

//*******************************************************************************
//*                              Main program loop                              *
//******************************************************************************* 

void loop () {
  char c;
  char Buffer [32];
  byte i;
  byte *EepromPtr;

  printf_P (PSTR("CRC     0x%8.8lx\n"), EepromContent.Crc);
  printf_P (PSTR("Header  %c%c\n"), EepromContent.Header >> 8, EepromContent.Header & 0xFF);
  printf_P (PSTR("ID      %8.8s\n"), EepromContent.Id);
  printf_P (PSTR("Address %3.3o\n"), EepromContent.NodeAddress);
  
  printf_P (PSTR("Select (I) or (A) to modify ID or Address, or (W) to write EEPROM\n"));
//  while (!Serial.available ());
  c = getch ();
  if ((c == 'a') || (c == 'A')) {
    printf_P (PSTR("Enter new node address: "));
    c = getch ();
    readSerial (Buffer);
    printf_P (PSTR("\nNew address: %s\n"), Buffer);
    sscanf (Buffer, "%o", &EepromContent.NodeAddress);
  } else if ((c == 'i') || (c == 'I')) {
    memset (Buffer, ' ', sizeof Buffer);
    printf_P (PSTR("Enter new node ID (8 characters): "));
    c = getch ();
    readSerial (Buffer);
    memcpy (EepromContent.Id, Buffer, 8);
    printf_P (PSTR("\nNew node ID: %8.8s\n"), EepromContent.Id);
  } else if ((c == 'w') || (c == 'W')) {
    EepromPtr = (byte *) &EepromContent;
    EepromContent.Crc = CrcBuffer (EepromPtr + 4, sizeof (Eeprom_t) - 4);    // Do not include stored CRC into CRC calculation
    for (i = 0; i < sizeof (Eeprom_t); i++) {
      EEPROM.write (i, *EepromPtr++);
    }
  }    

  delay (1000);


}

