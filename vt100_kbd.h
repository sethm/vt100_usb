/*
 * VT100 to USB Converter.
 *
 * ------------------------------------------------------------------------
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2013 Seth Morabito.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * ------------------------------------------------------------------------
 */

#ifndef __VT100_KBD_H
#define __VT100_KBD_H

#define CPU_PRESCALE(n)	(CLKPR = 0x80, CLKPR = (n))

#define KBD_RD_L    (1<<6)
#define KBD_WR_L    (1<<7)
#define KBD_RESET_H (1<<4)
#define KBD_TBMT_H  (1<<6)
#define START_SCAN  (1<<6)

#define MODIFIER_ALT      0x31 // PF3
#define MODIFIER_GUI      0x41 // PF4
#define MODIFIER_SETUP    0x7b
#define MODIFIER_CTRL     0x7c
#define MODIFIER_SHIFT    0x7d
#define MODIFIER_CAPSLOCK 0x7e
#define END_OF_SCAN       0x7f

// Keyboard Status bits.
#define BELL_BIT          0x01
#define KEYCLICK_BIT      0x02
#define SETUP_BIT         0x04
#define SPEAKER_LED_BIT   0x08
#define CAPSLOCK_BIT      0x10
#define SPEAKER_BIT       0x80

#define CLOCK_PERIOD 8  /* microseconds */

/*
 * Number of status updates between requests for keyboard scans.
 */
#define UPDATES_BETWEEN_SCANS 16

/*
 * Number of trips through the status loop to take before deciding
 * that a 0x7f "scan done" isn't going to appear. 16 is extremely
 * generous, it really should never be > 4.
 */
#define SCAN_DONE_TIMEOUT 16

#define KEY_BUFFER_SIZE 3

#define LONG_REPEAT_TIMEOUT  400
#define SHORT_REPEAT_TIMEOUT 30

#define TRUE 1
#define FALSE 0

/*
 * Typedefs
 */

struct key_struct {
  unsigned int address : 7;
  unsigned int sent : 1;
};

typedef struct key_struct key;

typedef unsigned int bool_t;

/*
 * Function definitions
 */
static uint8_t kbd_read(void);

static void kbd_write(uint8_t val);

static inline void io_setup(void);

static inline void reset(void);

static inline void clear_buf(key *buf, size_t size);

static inline void copy_buf(key *source_buffer, key *dest_buffer, size_t size);

#endif
