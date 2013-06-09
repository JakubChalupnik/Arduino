#ifndef _HBUS_CFG_H
#define _HBUS_CFG_H

//================================================
// HBUS configuration
//

#define	RS485_BAUD 		9600

#define Rs485DirectionPin      23
//#define Rs485RxLedPin          0
//#define Rs485TxLedPin          1

//================================================
// HBUS functions
//

#define Rs485SendByte(c)       {Serial1.write (c); Serial.write (c);}
#define Rs485SenderBusy()      (!(UCSR1A & (1 << TXC1)))
#define Rs485SenderBusyClear() {UCSR1A |= (1 << TXC1);}
#define Rs485GetByte()         (Serial1.read ())
#define Rs485ByteAvailable()   (Serial1.available () > 0)

#define Rs485EnableDriver()    {digitalWrite (Rs485DirectionPin, HIGH);}
#define Rs485DisableDriver()   {digitalWrite (Rs485DirectionPin, LOW);}

//#define Rs485RxLedOn()           {}
//#define Rs485RxLedOff()          {}

#define Rs485TxLedOn()           {ShiftPWM.SetOne (2, 31);}
//#define Rs485TxLedOff()          {}

//================================================
// HBUS hooks
//

#define HOOK_BEFORE_RECEIVE_HANDLER()   {}
#define HOOK_AFTER_RECEIVE_HANDLER()    {}
//#define HOOK_BEFORE_RX()                {Rs485RxLedOn ();}
//#define HOOK_AFTER_RX()                 {Rs485RxLedOff ();}
//#define HOOK_BEFORE_TX()                {Rs485TxLedOn ();}
//#define HOOK_AFTER_TX()                 {Rs485TxLedOff ();;}

#define HOOK_BEFORE_RX()               
#define HOOK_AFTER_RX()                
#define HOOK_BEFORE_TX()                {Rs485TxLedOn ();}
#define HOOK_AFTER_TX()                

#endif

