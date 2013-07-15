#ifndef __VT100_KBD_H
#define __VT100_KBD_H

#define CPU_PRESCALE(n)	(CLKPR = 0x80, CLKPR = (n))
 
// TODO: On the breadboard, D4 & D6 are tied together. In
// the final board, they'll be separate.

/* #define KBD_RD_L    (1<<6) */
#define KBD_RD_L    ((1<<6) | (1<<4))
#define KBD_WR_L    (1<<7)
#define KBD_RESET_H (1<<4)
#define KBD_TBMT_H  (1<<6)
#define START_SCAN  (1<<6)

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

/*
 * Typedefs
 */

struct key_struct {
  unsigned int address : 7;
  unsigned int sent : 1;
};

typedef struct key_struct key;

/*
 * Function definitions
 */
static void kbd_status_loop(void);

static uint8_t kbd_read(void);

static void kbd_write(uint8_t val);

static inline void io_setup(void);

static inline void reset(void);

static inline void clear_buf(key *buf, size_t size);

static inline void copy_buf(key *source_buffer, key *dest_buffer, size_t size);

#endif
