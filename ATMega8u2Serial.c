/** \file
 * ATMega8u2Serial.c
 * Written by Justin Walbeck
 * Copyright (c) 2013
 *
 * Implement a very rudimentary double ring-buffered serial line.
 */

#include "ATMega8u2Serial.h"

/**
 * Define a set of rudimentary ring buffers. Set RING_BUFFER_SIZE as
 * large as possible for the desired microcontroller to minimize dropped
 * packets.
 */
#define RING_BUFFER_SIZE 127
/** Current location of the write point for the transmit buffer*/
volatile int8_t write_ptr_tx;
/** Current location of the read point for the transmit buffer*/
volatile int8_t read_ptr_tx;
/** SRAM storage space for outgoing character data */
volatile uint8_t tx_ring_buffer[RING_BUFFER_SIZE];
/** Current location of the write point for the receive buffer*/
volatile int8_t write_ptr_rx;
/** Current location of the read point for the receive buffer*/
volatile int8_t read_ptr_rx;
/** SRAM storage space for incoming character data */
volatile uint8_t rx_ring_buffer[RING_BUFFER_SIZE];

/**
 * Initialize the on-chip hardware USART. Configures both lines for
 * 31250 baud communication, the standard comm rate for MIDI devices.
 */
void Serial_init() {
  UCSR1B |= (1 << RXEN1) | (1 << TXEN1); //Activate both directions
  UCSR1B |= (1 << RXCIE1);
  UCSR1C |= (3 << UCSZ10) | (1 << USBS1); //8 bits, one stop, no parity
  //31250 baud for MIDI
  UBRR1H = 0;
  UBRR1L = 31;

  write_ptr_tx = read_ptr_tx = 0;
  write_ptr_rx = read_ptr_rx = 0;
}

/**
 * Checks for a character being received from the peripheral controller.
 * The function is non-blocking, meaning if no character is waiting it
 * returns immediately.
 */
uint8_t Serial_getChar(uint8_t* c) {
  if (read_ptr_rx == write_ptr_rx)
    return 0;
  *c = rx_ring_buffer[read_ptr_rx++];
  if (read_ptr_rx == RING_BUFFER_SIZE)
    read_ptr_rx = 0;
  return 1;
}

/**
 * Queue a character to be transmitted to the controller from a host
 * computer.
 */
inline void Serial_sendChar(uint8_t c) {
  tx_ring_buffer[write_ptr_tx++] = c;
  //Let the hardware UART know we have something for it to send.
  UCSR1B |= (1 << UDRIE1);
}

/**
 * Interrupt routine for non-blocking character transmission. Called when
 * the hardware USART detects an empty UDR register.
 */
ISR(USART1_UDRE_vect)
{
  if (write_ptr_tx != read_ptr_tx)
    UDR1 = tx_ring_buffer[read_ptr_tx++];
  else {
    //Disable interrupt generation as we've sent everything we need to.
    UCSR1B &= ~(1 << UDRIE1);
    return;
  }
  if (read_ptr_tx == RING_BUFFER_SIZE)
    read_ptr_tx = 0;
}

/** Interrupt routine for non-blocking character reception. */
ISR(USART1_RX_vect)
{
  rx_ring_buffer[write_ptr_rx++] = UDR1;
  if (write_ptr_rx == RING_BUFFER_SIZE)
    write_ptr_rx = 0;
}