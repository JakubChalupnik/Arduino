ATMega1284p

Konektor RS-485:
 A B | A B 
 - + | - +
full | TX only

Pripojene na UART1, rizeni smeru D23
Napajeni spolecne bez jakehokoliv oddeleni

Adafruit_PCD8544 display = Adafruit_PCD8544(18, 19, 20, 22, 21);
PWM na LED na D15

ENC28J60 ma CS na D4 (PB4), pouzivam Jeelib knihovnu pro Ethercard. INT neni zapotrebi

RFM12B: 
 - CS na D31 (PA7)
 - IRQ na PB2
 - https://github.com/LowPowerLab/RFM12B

Nutno upravit pouzitou knihovnu:
 - pouzit attachInterrupt 2 misto 0
#elif defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__)
  #define RFM_IRQ     10 
  #define SS_DDR      DDRA
  #define SS_PORT     PORTA
  #define SS_BIT      7 


Pouzite piny:

OneWire 	27
Nokia LCD	18, 19, 20, 22, 21 + 15 (b/l)
ShiftPWM	24 26 25
Rs485Direction	23
RFM12		2 (IRQ) 31 (CS)
Ethernet	4 (CS)
SPI		5 6 7
UART1		8 9
UART2		10 11

Volne piny:	0 1 3 12 13 14 16 17 28 29 30

Alternativni funkce:
I2C		16 17
PWM		3 12 13 14
Analog		28 29 30
