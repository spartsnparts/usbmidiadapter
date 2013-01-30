/** \file
 * ATMega8u2Serial.h
 * Written by Justin Walbeck
 * Copyright (c) 2013
 *
 * Implement a very rudimentary double ring-buffered serial line.
 */

#ifndef ATMega8u2Serial_h
#define ATMega8u2Serial_h

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>

void Serial_init(void);
uint8_t Serial_getChar(uint8_t*);
void Serial_sendChar(uint8_t);

#endif
