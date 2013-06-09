#ifndef _HBUS_H
#define _HBUS_H

#include "Arduino.h"

#define MAX_MESSAGE_LEN 64

#define MSG_BROADCAST	0xFF

#define MSG_GET_ID		0x00
#define MSG_SET_CLOCK	0x01
#define MSG_SEND_MSG    0x02
#define MSG_SEND_TEXT   0x03
#define MSG_GET_BUTTONS 0x04
#define MSG_GET_TEXT    0x05
#define MSG_GET_IR      0x06
#define MSG_SEND_IR     0x07
#define MSG_SET_LEDS    0x08
#define MSG_SET_7SEG    0x09
#define MSG_ATTENTION   0x10
#define MSG_SET_PARM    0x11
#define MSG_SYSTEM      0x1F

#define SF_CAN_SHOW_MESSAGES        0x01
#define SF_CAN_SHOW_TEXT            0x02
#define SF_HAS_BUTTONS              0x04
#define SF_HAS_TEXT_INPUT           0x08
#define SF_HAS_INFRARED_RECEIVER    0x10
#define SF_HAS_INFRARED_SENDER      0x20
#define SF_CAN_SHOW_LEDS            0x40
#define SF_CAN_SHOW_7SEG            0x80

// --- Define frequency of the timeout (100Hz -> 10ms)
#define F_TURNAROUND_TIMEOUT 100UL

// --- Define escape sequences for text display
#define T_POS       0x01

#define F_MESSAGE_RECEIVED		0x0001
#define F_MESSAGE_FAILED		0x0002
#define F_MESSAGE_SENDING		0x0004
#define F_RECEIVING_MESSAGE		0x0008

#define hex_bin(c) ((c > '9') ? (c - 'A' + 10) : (c - '0'))

//boolean receive_message_handler(void);

void send_message(uint8_t len);
//boolean get_answer(uint8_t slave, uint8_t msg);

extern uint8_t send_buff[MAX_MESSAGE_LEN];
extern uint8_t rec_buff[MAX_MESSAGE_LEN];
extern uint8_t rec_len;

#endif

