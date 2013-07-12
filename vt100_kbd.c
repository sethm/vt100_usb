/* 
 * VT100 to USB Converter.
 *
 * Copyright (c) 2013 Seth Morabito
 *
 */

#include <avr/io.h>
// #include <avr/pgmspace.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "usb_debug_only.h"
#include "print.h"

#define CPU_PRESCALE(n)	(CLKPR = 0x80, CLKPR = (n))

/* 
 * TODO: On the breadboard, D4 & D6 are tied together. In
 * the final board, they'll be separate.
 */
/* #define KBD_RD_L    (1<<6) */
#define KBD_RD_L    ((1<<6) | (1<<4))
#define KBD_WR_L    (1<<7)
#define KBD_RESET_H (1<<4)
#define KBD_TBMT_H  (1<<6)

#define CLOCK_PERIOD 8  /* microseconds */

#define TOGGLE_READ  PORTD ^= KBD_RD_L ; _delay_us(10) ; PORTD ^= KBD_RD_L

/* Pulse the KBD_RESET_H line high for 10 microseconds */
#define TOGGLE_RESET PORTB ^= KBD_RESET_H ; _delay_us(10) ; PORTB ^= KBD_RESET_H ; _delay_us(25 * CLOCK_PERIOD)

/*
 * Function definitions
 */
void setup(void);
void service_loop(void);
uint8_t kbd_read(void);
void kbd_write(uint8_t val);

/*
 * Main Entry Point
 */
int main(void) {
  CPU_PRESCALE(0);
  setup();
  service_loop();
}

#define STEPS_BETWEEN_SCAN 64

/*
 * The main service loop.
 */
void service_loop(void) {
  uint8_t kstatus;
  volatile int scan_counter;

  scan_counter = STEPS_BETWEEN_SCAN;

  while (1) {
    kstatus = 1;


    // Just do a binary count.
    kstatus = 0;
    while (kstatus < (1<<4)) {
      for (int k = 0; k < 512; k++) {

	if (--scan_counter == 0) {
	  kstatus |= 0x50; // bit 6 is 'START_SCAN'
	} else {
	  kstatus &= ~0x50;
	}

	kbd_write(kstatus);

	if (scan_counter == 0) {
	  scan_counter = STEPS_BETWEEN_SCAN;
	}

      }
      kstatus++;
    }

  }
}

/*
 * The unfortuate arrangement of IO pins, and my absurd desire to make
 * the hardware wiring easier, lead to a fairly complex implementation
 * of a simple write.
 *
 * The keyboard output is wired to B0, B1, B2, B3, B7, D0, D1, D2.
 *
 * Meanwhile, D6 and D7 are also outputs (wired to the KBD_RD_L and
 * KBD_WR_L lines, respectively) and their state must not change on
 * data writes!
 */

void kbd_write(uint8_t val) {
  PORTB = ((val & 0x10) << 3) | (val & 0x0F);
  PORTD = ((val & 0xE0) >> 5) | (1 << 7) | KBD_RD_L | KBD_WR_L;

  // 1. Wait for write buffer to be empty.
  while (!(PINC & KBD_TBMT_H)) {
    ; // Do nothing
  }

  // 2. OK, buffer is empty. Let's write by pulsing
  //    KBD_WR_L low for 10 uS.
  PORTD ^= KBD_WR_L;
  _delay_us(10);
  PORTD ^= KBD_WR_L;
}

/*
 * Read the contents of the receive buffer.
 */
uint8_t kbd_read(void) {
  int f, b;

  PORTD ^= KBD_RD_L;

  _delay_us(1);

  f = PINF;
  b = PINB;

  PORTD ^= KBD_RD_L;

  /* Expected result byte is read from:
   *
   *    [B5|B6|F7|F6|F5|F4|F1|F0]
   *     7  6  5  4  3  2  1  0
   */
  return (uint8_t)((f & 0x03) |                 // F0..F1
		   ((f & 0xf0) >> 2) |          // F4..F5
		   (b & 0x40) |                 // B6
		   ((b & 0x20) << 2));          // B5
}

/*
 * Interrupt service routine for keyboard input.
 *
 * When the UART receives a byte of data, Port D3
 * will transition from LOW to HIGH (this is INT3 on
 * the Teensy 2.0)
 *
 * When that happens, we read off the byte, and the
 * interrupt is considered to have been served.
 *
 */
ISR(INT3_vect) {
  int read_byte;
  read_byte = kbd_read();
  // Transmit USB.
}

/*
 * Configure inputs and outputs.
 */
void setup(void) {
  /*
   * B0 - OUT - TBR0
   * B1 - OUT - TBR1
   * B2 - OUT - TBR2
   * B3 - OUT - TBR3
   * B4 - OUT - UART_RESET
   * B5 - IN  - RBR7
   * B6 - IN  - RBR6
   * B7 - OUT - TBR4
   */
  DDRB = 0b10011111;

  /*
   * C0 - OUT - NA
   * C1 - OUT - NA
   * C2 - OUT - NA
   * C3 - OUT - NA
   * C4 - OUT - NA
   * C5 - OUT - NA
   * C6 - IN  - KBD_TBMT_H
   * C7 - OUT - NA
   */
  DDRC = 0b10111111;

  /*
   * D0 - OUT - TBR5
   * D1 - OUT - TBR6
   * D2 - OUT - TBR7
   * D3 - IN  - KBD_DATA_AVAIL_H  // also, INT3
   * D4 - OUT - LINKED TO D6
   * D5 - OUT - NA
   * D6 - OUT - KBD_WR_L
   * D7 - OUT - KBD_RD_L
   */
  DDRD = 0b11110111;

  /*
   * F0 - IN  - RBR0
   * F1 - IN  - RBR1
   * F2 - OUT - NA
   * F3 - OUT - NA
   * F4 - IN  - RBR2
   * F5 - IN  - RBR3
   * F6 - IN  - RBR4
   * F7 - IN  - RBR5
   */
  DDRF = 0b00001100;

  // Initialize RESET to LOW (it's active high)
  PORTD &= ~KBD_RESET_H;

  // Initialize READ and WRITE to HIGH (they're active low)
  PORTB = KBD_RD_L | KBD_WR_L;

  // Enable interrupts on INT3 (PD3)
  EIMSK = (1 << INT3);

  // Configure for RISING EDGE trigger.
  EICRA = (1 << ISC31) | (1 << ISC30);

  // Now turn on interrupts.
  sei();

  // Do a reset.
  TOGGLE_RESET;
}
