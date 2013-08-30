//*******************************************************************************
//*
//* Arduino based matrix clock with Ethernet and NRF24L01
//*
//*******************************************************************************
//* Processor:  Arduino Pro Mini
//* Library:    http://code.google.com/p/ht1632c
//*
//* Author      Date       Comment
//*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//* Kubik       30.8.2013 First release, testing the HW
//*******************************************************************************

//*******************************************************************************
//*                            HW details                                       *
//*******************************************************************************
// Using 32x16 bicolor display from SeeedStudio (?)
// CS = 4, CLK = 5, WR = 6, Data = 7
//
// Note - the ht1632c library includes various fonts by default.
// To reduce the code size, some of the fonts can be disabled in ht1632c.h
//

//*******************************************************************************
//*                           Includes and defines                              *
//*******************************************************************************

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <ht1632c.h>

//*******************************************************************************
//*                               Static variables                              *
//*******************************************************************************

ht1632c Display = ht1632c (&PORTD, 7, 6, 5, 4, GEOM_32x16, 2);

//*******************************************************************************
//*                            Arduino setup method                             *
//*******************************************************************************

void setup () {

  Serial.begin (57600);
  Display.clear ();
  Display.pwm (1);

  Display.setfont (FONT_7x13);
  Display.putchar (1,  -2, '1', RED);
  Display.putchar (8,  -2, '2', RED);
  Display.setfont (FONT_5x7W);
  Display.putchar (14,  1, ':', RED);
  Display.setfont (FONT_7x13);
  Display.putchar (18,  -2, '3', RED);
  Display.putchar (25,  -2, '4', RED);

  Display.setfont (FONT_4x6);
  Display.putchar ( 0, 11, 'V', RED);
  Display.putchar ( 4, 11, 'o', RED);
  Display.putchar ( 8, 11, 'n', RED);
  Display.putchar (12, 11, ' ', RED);
  Display.putchar (16, 11, '1', RED);
  Display.putchar (20, 11, '8', RED);
  Display.putchar (24, 11, '~', RED);
  Display.putchar (28, 11, 'C', RED);

  Display.sendframe ();
}

//*******************************************************************************
//*                              Main program loop                              *
//******************************************************************************* 

void loop () {
}

