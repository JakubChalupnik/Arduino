/*********************************************************************
 *
 *         Basic support for HBUS for Arduino central unit
 *         Supports sending only, no receiving!
 *
 *********************************************************************
 * FileName:        hbus.pde
 * Dependencies:
 * Date:            12.3.2013
 * Processor:       Arduino ATMega1284p
 *
 * Author   Date       Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Kubik    12.3.2013  Initial version, based on eb_6010 project
 ********************************************************************/

#include "typedefs.h"
#include "hbus.h"
#include "hbus_cfg.h"

//---------------------------------------------------------------------------
// Local variables
uint8_t send_buff[MAX_MESSAGE_LEN];

void Rs485SendByte(byte c) {
//  Serial.write (c);
  loop_until_bit_is_set(UCSR1A, UDRE1); /* Wait until data register empty. */
  UDR1 = c;
//  loop_until_bit_is_set(UCSR1A, TXC1);
} 

//---------------------------------------------------------------------------
// Helper functions
static void send_ascii(uint8_t b) {

    b &= 0x0F;
    b = (b > 9) ? b + 'A' - 10 : b + '0';
    Rs485SendByte(b);
}

//---------------------------------------------------------------------------
// Send message - calculates CRC and encodes properly to Modbus-like ASCII frame
//

void send_message(uint8_t len) {
    uint8_t cksum;
    uint8_t *ptr;

    UCSR1B &= 0x1F;
    HOOK_BEFORE_TX();
    Rs485EnableDriver();
    Rs485SendByte(':');
    cksum = 0;
    ptr = send_buff;

    while(len > 0) {
        send_ascii(*ptr >> 4);
        send_ascii(*ptr);
        cksum += *ptr;
        ptr++;
        len--;
    }

    cksum = ~cksum + 1;
    send_ascii(cksum >> 4);
    send_ascii(cksum);
    Rs485SendByte(0x0D);                             // CR
    Rs485SendByte(0x0A);                             // LF
//    while(Rs485SenderBusy());
//    Rs485SenderBusyClear ();
//    Rs485DisableDriver();
    HOOK_AFTER_TX();
}

