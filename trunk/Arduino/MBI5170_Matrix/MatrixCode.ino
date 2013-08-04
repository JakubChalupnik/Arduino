//*******************************************************************************
//*                             LED matrix routines                             *
//*******************************************************************************

//
// The routine updates the screen matrix. If page flip is required (FLAGS_FLIP_PAGE),
// we change the page as soon as the whole display is finished to avoid flickering
//

void MatrixInterrupt (void) {
    static byte Row = 0;
    static byte Pwm_Counter = 0;
    static byte Fade = 0;
    static byte LineBuffer [SX];
    byte *ActivePage;           // Points to the row that shall be displayed this time
    byte *InactivePage;         // Points to the very same row of the inactive page
    byte *LineBufferPtr = LineBuffer;
    byte i;


    //
    // Initialize ActivePage. Point it to the current row on a page that's being displayed.
    //

    ActivePage = Screen [ScreenPageDisplayed()] + Row * SX;
    InactivePage = Screen [ScreenPageInactive()] + Row * SX;

    //
    // If fading is active (Fade > 0), we display both pages to give smoother transition
    //

    for(i = 0; i < SX; i++) {           // Send all bytes of current row
        *LineBufferPtr = *ActivePage++;
        if(Fade > 0) {
            *LineBufferPtr |= *InactivePage++;
        }
        LineBufferPtr++;
    }

    MatrixSend (LineBuffer, Row);
    Row = (Row + 1) & 0x07;

    //
    // Handle page fliping - if the flag is set, wait until the whole buffer
    // is displayed (i.e. Row is 0 again) and then toggle the FLAGS_PAGE_DISPLAYED
    //

    if((Row == 0) && (Flags & FLAGS_FLIP_PAGE)) {       // If someone requested fliping page
        Flags ^= FLAGS_PAGE_DISPLAYED;  // Flip the pages - displayed becomes inactive and vice versa
        Flags &= ~FLAGS_FLIP_PAGE;      // and clear the flag to signal the page was flipped
    }
    //
    // Handle fading - if the flag is set, wait until the whole buffer is displayed
    // and then set Fade variable to the amount of frames needed for transition
    //

    if((Row == 0) && (Flags & FLAGS_FADE_PAGE)) {       // If someone requested fading page
        Flags ^= FLAGS_PAGE_DISPLAYED;  // Flip the pages - displayed becomes inactive and vice versa
        Flags &= ~FLAGS_FADE_PAGE;      // and clear the flag to signal the page is fading
        Fade = CONFIG_FADE_DELAY;
    }
    //
    // If we are fading (Fade > 0), decrease the counter. The fading itself is done above
    //

    if((Row == 0) && (Fade > 0)) {
        Fade--;
    }
    //
    // Based on the light intensity, prepare the value of PWM counter.
    // The idea is as follow: after displaying all the rows, keep the display blank for Pwm_Counter frames,
    // thus emulating PWM dimming of the display.
    //

//     if(Row == 0) {
//         Pwm_Counter = (31 - LightIntensity) << 3;
//     }

}


//
// Puts bitmap line located at address BitmapLine to screen line ScreenLine starting from offset StartBit.
// The bitmap line is BitmapWidthIn bits wide and is located in program memory.
// StartBit cannot be < -7 !
// Due to use of signed char, this only works for small displays and bitmaps, stay under cca 100 bits
//

void PutBitmapLine(volatile byte * ScreenLine, PGM_P BitmapLine, signed char StartBit, byte BitmapWidthIn, byte ScreenWidth) {
    byte Tail;                  // Remainder of the previous bitmap byte that needs to be added to the next screen byte
    byte Shift;                 // How many bits has Tail
    byte Mask;                  // Mask corresponding to Shift (5 --> 00011111
    byte sx;                    // Screen byte index
    byte x;
    byte BitmapByte;            // Current bitmap byte read from program memory
    byte BitmapByteMask;        //  and its mask in case we're at the last byte and bitmap width is not divisable by 8
    signed char BitmapWidth;        // Internal copy of BitmapWidthIn that can go negative

    BitmapByteMask = ~((1 << (8 - (BitmapWidthIn & 0x07))) - 1);
    BitmapWidth = BitmapWidthIn;

    //
    // If we are starting with negative StartBit, set Tail to the visible part of the first bitmap byte, and skip it
    //

    if(StartBit < 0) {
        StartBit += 8;
        BitmapWidth -= 8;               // Width is in bits, not bytes, thus -8!
        Tail = pgm_read_byte(BitmapLine) << (8 - StartBit);     // Lower bitmap bits will become upper bits of next screen byte
        BitmapLine++;
    } else {
        Tail = 0;                       // and no tail
    }

    //
    // Prepare some constants used later in the main loop
    //

    Shift = StartBit & 0x07;            // The same as "modulo 8"
    Mask = (1 << (8 - Shift)) - 1;      // "Shift" number of ones starting from bit 0 (Shift == 5 --> 00011111)
    sx = StartBit / 8;                  // This is an index of the processed byte in the ScreenLine

    //
    // Process all bytes in the bitmap line.
    // Update the recent screen byte with combination of the tail from previous pass (or init value computed above),
    // and content of the recent bitmap byte.
    //

    for(x = 0; (x < BitmapWidth) && (sx < ScreenWidth); x += 8) {       // x is the bit position in the bitmap line
        BitmapByte = pgm_read_byte(BitmapLine);
        if((BitmapWidth - x) < 8) {     // Last byte of the bitmap line is not aligned (mod 8) so mask out extra bits at the end
            BitmapByte &= BitmapByteMask;
        }
        *(ScreenLine + sx) |= Tail | (BitmapByte >> Shift);     // Update the screen line, ORing the new value with original content
        sx++;                           // Next screen byte
        Tail = BitmapByte << (8 - Shift);       // Calculate tail - that's the part of the current bitmap byte we did not use yet
        BitmapLine++;                   // Advance BitmapLine pointer to next byte
    }

    //
    // All bytes of the bitmap processed, now update the next screen byte with whatever left.
    //

    if(sx < ScreenWidth) {              // Only when we're not reaching behind the screen end
        if(x == 0) {                    // This happens when tail has to be cut - LS bits need to be masked out as bitmap width is not mod 8
            Tail &= BitmapByteMask << (8 - StartBit);
        }
        *(ScreenLine + sx) |= Tail;     // And update the screen with whatever left
    }
}

//
// Put bitmap of size bw x bh to screen location bx,by.
//

void PutBitmap(signed char bx, signed char by, byte bw, byte bh, PGM_P Bitmap) {
    byte BitmapLineSize;
    byte BitmapOffset, y;

    BitmapLineSize = (bw + 7) / 8;

    //
    // If bx < -7, then we can skip bytes at the beginning, and adjust the bitmap line beginning and length.
    // BitmapOffset says how many bytes at the beginning of every bitmap line we have to skip.
    //

    if(bx < -7) {
        BitmapOffset = (-bx) / 8;
        bw -= BitmapOffset * 8;
        bx = BitmapOffset * 8 + bx;

    } else {
        BitmapOffset = 0;
    }

    for(y = 0; y < bh; y++) {           // For all rows of the bitmap
        if(((y + by) < 0) || ((y + by) >= SY)) {        // If the row is outside the screen
            Bitmap += BitmapLineSize;   // Advance to next bitmap line and continue
            continue;

        }
        //
        // Current line inside the screen, call helper to map it
        //

        PutBitmapLine (Screen [ScreenPageInactive()] + (y + by) * SX, Bitmap + BitmapOffset, bx, bw, SX);
        Bitmap += BitmapLineSize;       // Advance to next bitmap line and continue
    }

}

//
// Displays one character at specified location on the screen
//

extern const byte Font8x6[] PROGMEM;

void PutChar(signed char x, signed char y, byte c) {

    PutBitmap(x, y, 6, 8, (PGM_P) (Font8x6 + 8 * (c - '0')));
}
