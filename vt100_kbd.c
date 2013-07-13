/* 
 * VT100 to USB Converter.
 *
 * Copyright (c) 2013 Seth Morabito
 *
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>
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

/* Pulse the KBD_RESET_H line high for 10 microseconds */
#define TOGGLE_RESET PORTB ^= KBD_RESET_H ; _delay_us(10) ; PORTB ^= KBD_RESET_H ; _delay_us(25 * CLOCK_PERIOD)

/*
 * Number of status updates between requests for keyboard scans.
 */
#define STEPS_BETWEEN_SCAN 16

/*
 * Function definitions
 */
void setup(void);
void kbd_status_loop(void);
uint8_t kbd_read(void);
void kbd_write(uint8_t val);

/*
 * Globals
 */

/* The current iteration through the status loop. */
volatile int scan_counter = 0;

/* The keyboard status word. */
volatile uint8_t kbd_status = 0;

/* If set to '1', a scan is needed at the next interval. */
volatile int scan_done = 1;

/* The most recently read character. */
volatile uint8_t last_char = 0;

static const char unshifted_char_codes[32] = {
  0x00, 0x00, 0x00, 0x7f, 0x0d, 0x70, 0x6f, 0x79, 0x74, 0x77, 0x71, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x5d, 0x5b, 0x69, 0x75, 0x72, 0x65, 0x31, 0x00, 0x00, 0x00, 0x00, 0x00
};

static const char *char_names[32] = {
  "NUL", "NUL", "NUL", "DEL", "RET", "p", "o", "y", "t", "w", "q", "NUL", "NUL", "NUL", "NUL", "NUL",
  "NUL", "NUL", "NUL", "NUL", "]",   "[", "i", "u", "r", "e", "1", "NUL", "NUL", "NUL", "NUL", "NUL"
};

/*
 * Main Entry Point
 */
int main(void) {
  CPU_PRESCALE(0);
  setup();
  usb_init();

  /* Keep pushing status to the keyboard, forever. */

  kbd_status_loop();
}

/* DEBUG: Watchdog timeout on waiting for 'scan done' */

volatile int scan_done_watchdog = 2048;

/*
 * The main service loop.
 */
void kbd_status_loop(void) {
  
  scan_counter = STEPS_BETWEEN_SCAN;
  
  while (1) {

    // Watchdog check.
    if (--scan_done_watchdog == 0) {

      if (!scan_done) {
	print("Scan Watchdog Timeout exceeded. Resetting.\r\n");
	scan_done = 1;
      }
      
      scan_done_watchdog = 2048;
    }

    if (--scan_counter == 0 && scan_done) {
      kbd_status |= 0x50; // bit 6 is 'START_SCAN'
    } else {
      kbd_status &= ~0x50;
    }
    
    kbd_write(kbd_status);
    
    if (scan_counter == 0) {
      scan_counter = STEPS_BETWEEN_SCAN;
    }
    
  }
}

/*
 * The unfortuate arrangement of IO pins and my absurd desire to make
 * the hardware wiring easier lead to a fairly complex implementation
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

  // PORTD ^= KBD_RD_L;
  PORTD &= ~KBD_RD_L; // Pull low

  _delay_us(1);

  f = PINF;
  b = PINB;

  // PORTD ^= KBD_RD_L;
  PORTD |= KBD_RD_L;  // Make high again

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

// TODO:
// I think the algorithm is:
//   1. Read into global keycode variable.
//   2. If keycode was not '7F', disable 'start next scan' variable.
//   3. If keycode is '7F', enable 'start next scan' variable.
//
// New global state needed:
//   1. Last scan code read.
//   2. Toggle for 'do a scan at your next opportunity'

ISR(INT3_vect) {
  uint8_t data_in = kbd_read();
  
  if (data_in == 0x7f) {
    scan_done = 1;
  } else {
    /* If the data is not 0x7f, we need to hold off on the scan requests until it appears. */
    // TODO: We'll need to watch for key repeat here.
    if (last_char != data_in) {
      print("KBD Read interrupt: data=");
      phex(data_in);
      print("\r\n");

      last_char = data_in;
    }
    scan_done = 0;
  }
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

  // Initialize READ and WRITE to HIGH (they're active low),
  // the rest of PORTD to low.
  PORTD = KBD_RD_L | KBD_WR_L;

  // Enable interrupts on INT3 (PD3)
  EIMSK = (1 << INT3);

  // Configure for RISING EDGE trigger.
  EICRA = (1 << ISC31) | (1 << ISC30);

  // Now turn on interrupts.
  sei();

  // Do a reset.
  TOGGLE_RESET;
}
