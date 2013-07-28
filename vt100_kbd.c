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
 *
 * Most of this implementation is based on DEC Document
 * EK-VT100-TM-003, "VT100 Series Video Terminal Technical Manual".
 * Sections 4.4.8 and 4.4.9 document the VT100 firmware's
 * key handling procedures, which this program attempts to emulate.
 *
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdint.h>
#include "usb_keyboard.h"
#include "vt100_kbd.h"

/*
 * Globals
 */

// I have made some pretty arbitrary choices when deciding how to map
// various DEC keys to USB. Letters, numbers, and punctuation all
// behave just as you'd expect, but some other keys may need to be
// called out for special attention:
//
// DEC         USB
// ---         ---
// BREAK       Pause / Break
// PF1         F1
// PF2         F2
// PF3         ALT (PC) / Option (Mac)
// PF4         Windows Key (PC) / Command (Mac)
// LINE FEED   Enter / Return
// NO SCROLL   Scroll Lock
//
// SETUP       Keyboard Setup (internal use only)


/* Lookup table that converts key addresses to USB key codes */
static const uint8_t usb_key_codes[128] = {
  0,              0,               0,               KEY_DELETE,
  KEY_ENTER,      KEY_P,           KEY_O,           KEY_Y,
  KEY_T,          KEY_W,           KEY_Q,           0,
  0,              0,               0,               0,
  KEY_RIGHT,      0,               0,               0,
  KEY_LEFT_BRACE, KEY_RIGHT_BRACE, KEY_I,           KEY_U,
  KEY_R,          KEY_E,           KEY_1,           0,
  0,              0,               0,               0,
  KEY_LEFT,       0,               KEY_DOWN,        KEY_PAUSE,
  KEY_TILDE,      KEY_MINUS,       KEY_9,           KEY_7,
  KEY_4,          KEY_3,           KEY_ESC,         0,
  0,              0,               0,               0,
  KEY_UP,         KEY_F3,          KEY_F1,          KEY_BACKSPACE,
  KEY_EQUAL,      KEY_0,           KEY_8,           KEY_6,
  KEY_5,          KEY_2,           KEY_TAB,         0,
  0,              0,               0,               0,
  KEYPAD_7,       KEY_F4,          KEY_F2,          KEYPAD_0,
  KEY_ENTER,      KEY_BACKSLASH,   KEY_L,           KEY_K,
  KEY_G,          KEY_F,           KEY_A,           0,
  0,              0,               0,               0,
  KEYPAD_8,       KEYPAD_ENTER,    KEYPAD_2,        KEYPAD_1,
  0,              KEY_QUOTE,       KEY_SEMICOLON,   KEY_J,
  KEY_H,          KEY_D,           KEY_S,           0,
  0,              0,               0,               0,
  KEYPAD_PERIOD,  KEY_COMMA,       KEYPAD_5,        KEYPAD_4,
  KEY_ENTER,      KEY_PERIOD,      KEY_COMMA,       KEY_N,
  KEY_B,          KEY_X,           KEY_SCROLL_LOCK, 0,
  0,              0,               0,               0,
  KEYPAD_9,       KEYPAD_3,        KEYPAD_6,        KEYPAD_MINUS,
  0,              KEY_SLASH,       KEY_M,           KEY_SPACE,
  KEY_V,          KEY_C,           KEY_Z,           0,
  0,              0,               0,               0
  };

// Decremented each time through the main loop. When this value
// reaches '0', a scan is requested.
int scan_needed_timeout = UPDATES_BETWEEN_SCANS;

// The keyboard status word.
volatile uint8_t kbd_status = 0;

// If set to '1', a scan is needed at the next interval.
volatile bool_t scan_needed = TRUE;

// The timeout (in trips through the status update loop) before we
// will auto-repeat the key(s) being held down.
volatile unsigned int repeat_timeout = LONG_REPEAT_TIMEOUT;

bool_t capslock_seen = FALSE;
bool_t setup_seen = FALSE;

// The bitmask of the modifier keys to send with the USB key code.
uint8_t modifier_keys = 0;

// The count of (non-modifier) key addresses found in the current
// scan.
size_t scan_count = 0;

// If set to a value > 0, will cause the speaker to click when the status
// byte is sent to the keyboard. Each time through the status update,
// this will be decremented until it reaches 0 again. A value of ~'200'
// will generate a beep. A value of '2' will generate a key-click.
volatile uint8_t speaker_counter = 0;

// Buffers to hold (non-modifier) keys seen in the current and the
// previous scan.
key old_key_buffer[KEY_BUFFER_SIZE];
key new_key_buffer[KEY_BUFFER_SIZE];
size_t new_key_buffer_idx = 0;

/*
 * Main Entry Point
 */
int main(void) {
  CPU_PRESCALE(0);
  io_setup();
  usb_init();

  // While a very rough estimate, each trip through this loop is going
  // to be on the order of 1.28 ms, since each keyboard status write
  // takes 160 clock cycles at 8 us each. This does not account for
  // overhead.
  while (1) {

    // Decrement the repeat timeout, but only if it
    // is not yet 0.
    if (repeat_timeout > 0) {
      repeat_timeout--;
    }

    // If it's time to request a scan, do so.
    if (--scan_needed_timeout == 0 && scan_needed) {
      kbd_status |= START_SCAN;
    } else {
      kbd_status &= ~START_SCAN;
    }

    if (speaker_counter > 0) {
      speaker_counter--;
      kbd_status |= (SPEAKER_BIT | SPEAKER_LED_BIT);
    } else {
      kbd_status &= ~(SPEAKER_BIT | SPEAKER_LED_BIT);
    }

    kbd_write(kbd_status);

    if (scan_needed_timeout == 0) {
      scan_needed_timeout = UPDATES_BETWEEN_SCANS;
    }

  }
}

/*
 * The keyboard output is wired to B0, B1, B2, B3, B7, D0, D1, D2.
 *
 * Meanwhile, D6 and D7 are also outputs (wired to the KBD_RD_L and
 * KBD_WR_L lines, respectively) and their state must not change on
 * data writes!
 */

static void kbd_write(uint8_t val) {
  // Data is written to:
  //
  //    [D2|D1|D0|B7|B3|B2|B1|B0]
  //     7  6  5  4  3  2  1  0
  //
  PORTB = ((val & 0x10) << 3) | (val & 0x0F);
  // KBD_RD_L and KBD_WR_L should always be left high during a write.
  PORTD = ((val & 0xE0) >> 5) | (1 << 7) | KBD_RD_L | KBD_WR_L;

  // Poll the KBD_TBMT_H input, waiting for write buffer to be empty.
  while (!(PINC & KBD_TBMT_H)) {
    ; // Do nothing
  }

  // Once the buffer is empty, write by pulsing KBD_WR_L for 10 uS
  PORTD &= ~KBD_WR_L;
  _delay_us(10);
  PORTD |= KBD_WR_L;
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

  // Data is read from:
  //
  //    [B5|B6|F7|F6|F5|F4|F1|F0]
  //     7  6  5  4  3  2  1  0
  //
  return (uint8_t)((f & 0x03) |                 // F0..F1
                   ((f & 0xf0) >> 2) |          // F4..F5
                   (b & 0x40) |                 // B6
                   ((b & 0x20) << 2));          // B5
}

/***********************************************************************/
/* Routines used by the INT3 Interrupt Handler                         */
/***********************************************************************/

/*
 * Toggle the CAPSLOCK state, if needed.
 */
static inline void update_modifier_state(void) {
  if ((kbd_status & CAPSLOCK_BIT) && !capslock_seen) {
    kbd_status &= ~CAPSLOCK_BIT;
    usb_keyboard_press(KEY_CAPS_LOCK, 0);
  } else if (!(kbd_status & CAPSLOCK_BIT) && capslock_seen) {
    kbd_status |= CAPSLOCK_BIT;
    usb_keyboard_press(KEY_CAPS_LOCK, 0);
  }

  if ((kbd_status & SETUP_BIT) && !setup_seen) {
    kbd_status &= ~SETUP_BIT;
  } else if (!(kbd_status & SETUP_BIT) && setup_seen) {
    kbd_status |= SETUP_BIT;
  }
}

static inline void handle_setup(uint8_t key_code, uint8_t modifier_keys) {
  switch (key_code) {
  case KEY_K:
    kbd_status ^= KEYCLICK_BIT;
    break;
  case KEY_B:
    kbd_status ^= BELL_BIT;
    break;
  }
}

static inline void possibly_transmit_keys(void) {
  key *old_key, *new_key;

  // Do a two-pass comparison of the old_key_buffer and the
  // new_key_buffer.
  //
  // 1. Remove any keys from the old_key_buffer that do not appear
  //    in the new_key_buffer
  for (int i = 0; i < KEY_BUFFER_SIZE; i++) {
    bool_t match_seen = FALSE;

    old_key = &old_key_buffer[i];

    if (old_key->address == 0) {
      continue;
    }

    for (int j = 0; j < KEY_BUFFER_SIZE; j++) {
      new_key = &new_key_buffer[j];

      if (new_key->address == 0) {
        continue;
      }

      if (new_key->address == old_key->address) {
        match_seen = TRUE;
      }
    }

    // If the old key does not exist in the new key buffer,
    // we clear it out.
    if (!match_seen) {
      old_key->address = 0;
      old_key->sent = 0;
    }
  }

  // 2. Any remaining keys in the old_key_buffer are compared
  //    to the new_key_buffer to see if they should be transmitted.
  for (int j = 0; j < KEY_BUFFER_SIZE; j++) {
    new_key = &new_key_buffer[j];

    if (new_key->address == 0) {
      continue;
    }

    for (int i = 0; i < KEY_BUFFER_SIZE; i++) {
      old_key = &old_key_buffer[i];

      if (old_key->address == 0) {
        continue;
      }

      if (old_key->address == new_key->address) {
        if (!old_key->sent || repeat_timeout == 0) {
          uint8_t key_code = usb_key_codes[new_key->address & 0x7f];

          if (key_code) {
            if ((kbd_status & KEYCLICK_BIT) &&
                repeat_timeout > SHORT_REPEAT_TIMEOUT) {
              speaker_counter = 2; // Short key-click
            }

            if ((kbd_status & BELL_BIT) &&
                (modifier_keys & KEY_CTRL) && key_code == KEY_G) {
              speaker_counter = 200; // ~ 1 second bell
            }

            if (kbd_status & SETUP_BIT) {
              handle_setup(key_code, modifier_keys);
            } else {
              usb_keyboard_press(key_code, modifier_keys);
            }
          }

          // Mark both to ensure propagation when new is copied to old.
          new_key->sent = 1;
          old_key->sent = 1;

          // If we're re-setting the repeat timeout, reset to the shorter
          // delay
          if (repeat_timeout == 0) {
            repeat_timeout = SHORT_REPEAT_TIMEOUT;
          }

        } else {
          // Keep the state and propagate it.
          new_key->sent = 1;
        }
      }
    }
  }
}

/*
 * Interrupt service routine for keyboard input.
 *
 * When the UART receives a byte of data from the keyboard, Port D3
 * will transition from LOW to HIGH, triggering the INT3 interrupt.
 * We read off a byte and act on it.
 */
ISR(INT3_vect) {
  uint8_t data_in = kbd_read();

  // Will be set true if the scan is over.
  scan_needed = FALSE;

  // A scan is finished when 0x7f is received.
  switch (data_in) {
  case END_OF_SCAN:
    // If the scan_count was > 3, we pretend this scan never happened,
    // as per the technical manual.
    if (scan_count <= 3) {
      // We immediately want to check to see if the old and new
      // buffers are identical. If not, we reset the key repeat
      // timeout.
      for (int i = 0; i < KEY_BUFFER_SIZE; i++) {
        if (old_key_buffer[i].address != new_key_buffer[i].address) {
          repeat_timeout = LONG_REPEAT_TIMEOUT;
          break;
        }
      }

      update_modifier_state();
      possibly_transmit_keys();
    }

    // Copy the new buffer to the old buffer
    copy_buf(new_key_buffer, old_key_buffer, KEY_BUFFER_SIZE);

    // Reset state.
    clear_buf(new_key_buffer, KEY_BUFFER_SIZE);
    scan_needed = TRUE;
    scan_count = 0;
    new_key_buffer_idx = 0;
    modifier_keys = 0;

    capslock_seen = FALSE;
    setup_seen = FALSE;

    break;
  case MODIFIER_CAPSLOCK:
    // DEC considers CAPSLOCK to be a modifier key, but the USB
    // standard says it's a toggle key. We have to watch for it and
    // maintain the state internally so we can send it again when it
    // turns off.
    capslock_seen = TRUE;
    break;
  case MODIFIER_SETUP:
    setup_seen = TRUE;
    break;
  case MODIFIER_ALT:
    modifier_keys |= KEY_ALT;
    break;
  case MODIFIER_GUI:
    modifier_keys |= KEY_GUI;
    break;
  case MODIFIER_CTRL:
    modifier_keys |= KEY_CTRL;
    break;
  case MODIFIER_SHIFT:
    modifier_keys |= KEY_SHIFT;
    break;
  default:
    // This isn't an end-of-scan, capslock, or a modifier, so treat
    // it as a normal key. Just append this to the cur_key_code_buffer
    // if there's room.
    if (new_key_buffer_idx < KEY_BUFFER_SIZE) {
      int idx = new_key_buffer_idx++;
      new_key_buffer[idx].address = data_in;
      new_key_buffer[idx].sent = 0;
    }

    // Increment the count of non-modifier keys found
    scan_count++;
  }
}

/*
 * Configure inputs and outputs.
 */
static inline void io_setup(void) {
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

  // Configure for RISING EDGE trigger, since
  // the UART data available line is active high.
  EICRA = (1 << ISC31) | (1 << ISC30);

  // Now turn on interrupts.
  sei();

  // Do a reset.
  reset();
}

/*
 * Request a reset of the UART
 */
static inline void reset(void) {
  // Bring the KBD_RESET_H line high for 10 uS, then wait at least 18
  // UART clock cycles before continuing (as specified by the
  // datasheet)
  PORTB |= KBD_RESET_H;
  _delay_us(10);
  PORTB &= ~KBD_RESET_H;
  _delay_us(18 * CLOCK_PERIOD);
}

/*
 * Clear the specified keyboard buffer.
 */
static inline void clear_buf(key *buf, size_t size) {
  int i;

  for (i = 0; i < size; i++) {
    buf[i].address = 0;
    buf[i].sent = 0;
  }
}

/*
 * Copy from the specified source keyboard buffer to the destination
 * keyboard buffer.
 */
static inline void copy_buf(key *source_buffer, key *dest_buffer, size_t size) {
  int i;

  for (i = 0; i < size; i++) {
    dest_buffer[i].address = source_buffer[i].address;
    dest_buffer[i].sent = source_buffer[i].sent;
  }
}
