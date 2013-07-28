
#define CLK 10    // PB4
#define SDI  9    // PH6
#define OE  12    // PB6
#define LE  11    // PB5

//#define MatrixClkPulse()       { digitalWrite (CLK, HIGH); digitalWrite (CLK, LOW); }
//#define MatrixStrobePulse()    { digitalWrite (LE, HIGH); digitalWrite (LE, LOW); }
//#define MatrixDataHigh()       { digitalWrite (SDI, HIGH); }
//#define MatrixDataLow()        { digitalWrite (SDI, LOW); }

#define MatrixClkPulse()       { PORTB |= (1 << 4); PORTB &= ~(1 << 4); }
#define MatrixStrobePulse()    { PORTB |= (1 << 5); PORTB &= ~(1 << 5); }
#define MatrixDataHigh()       { PORTH |= (1 << 6); }
#define MatrixDataLow()        { PORTH &= ~(1 << 6); }
#define MatrixOeHigh()         { PORTB |= (1 << 6); }
#define MatrixOeLow()          { PORTB &= ~(1 << 6); }
  
void ShiftSendByte (byte b) {
  byte i;
  
  for (i = 0; i < 8; i++) {
    if (b & 0x01) {
      MatrixDataHigh ();
    } else {
      MatrixDataLow ();
    }
    MatrixClkPulse ();
    b = b >> 1;
  }
}

#define C_MASK_4 0x80  
#define C_MASK_2 0x40  
#define C_MASK_1 0x20  
#define C_MASK_7 0x10  
#define C_MASK_6 0x08  
#define C_MASK_0 0x04  
#define C_MASK_5 0x02  
#define C_MASK_3 0x01  

byte MbiConvertColumns (byte c) {
  byte Columns = 0;
  
  if (c & 0x80) Columns |= C_MASK_0;
  if (c & 0x40) Columns |= C_MASK_1;
  if (c & 0x20) Columns |= C_MASK_2;
  if (c & 0x10) Columns |= C_MASK_3;
  if (c & 0x08) Columns |= C_MASK_4;
  if (c & 0x04) Columns |= C_MASK_5;
  if (c & 0x02) Columns |= C_MASK_6;
  if (c & 0x01) Columns |= C_MASK_7;
  
  return Columns;
}

void MbiSendByte (byte b) {
  ShiftSendByte (MbiConvertColumns (b));
}
 

void MatrixSend (unsigned long Columns, byte Row) {
  ShiftSendByte (~(1 << (Row & 0x07)));
  MbiSendByte ((Columns >> 24) & 0xFF);
  MbiSendByte ((Columns >> 16) & 0xFF);
  MbiSendByte ((Columns >> 8) & 0xFF);
  MbiSendByte ((Columns >> 0) & 0xFF);
  MatrixStrobePulse ();
}


// the setup routine runs once when you press reset:
void setup() {                
  // initialize the digital pin as an output.
  
  digitalWrite (CLK, LOW);
  digitalWrite (LE, LOW);
  digitalWrite (SDI, LOW);
  digitalWrite (OE, HIGH);
  
  pinMode (CLK, OUTPUT);
  pinMode (SDI, OUTPUT);
  pinMode (OE, OUTPUT);
  pinMode (LE, OUTPUT);

  digitalWrite (OE, LOW);
}

 
// the loop routine runs over and over again forever:
void loop() {
  static byte i = 0;

  delay(100);               // wait for a second
  MatrixSend (1L << (i + 24), i);
  
  i++;
  if (i > 7) i = 0;
  
}
