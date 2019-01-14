# VT100 to USB Converter Firmware

**Version:** 1.0
**Last Updated:** 28 July, 2013

## 1.0 About

This project contains source code and schematics for a DEC VT100
terminal keyboard to USB converter.

The purpose of the converter is to allow a DEV VT100 terminal keyboard
to be used as a USB input device on a modern computer. This is, of
course, almost completely useless, but that doesn't mean it's not fun.

## 2.0 Description

The converter uses a [Teensy 2.0](http://www.pjrc.com/teensy/)
development board powered by an ATmega32U4 microcontroller, which this
firmware targets. In addition to the microcontroller, the converter
uses an external HD6402 UART, an external clock generator, and a
voltage comparator for signalling with the VT100 keyboard.

The firmware is written in AVR C, and can be compiled with the avr-gcc
tool chain. The resulting firmware should be loaded onto the Teensy
with the Teensy loader.

## 3.0 Key Mapping

Most keys should be self-explanatory. All alphanumerics and standard
punctuation characters map to what you'd expect. Several keys, though,
need special attention.

| VT100 Key | USB Keyboard Key          |
|:---------:|:-------------------------:|
| NO SCROLL | Scroll Lock               |
| BREAK     | Pause / Break             |
| LINE FEED | Enter / Return            |
| PF1       | F1                        |
| PF2       | F2                        |
| SET-UP    | Keyboard Config (see 4.0) |

The VT100 keyboard is notably deficient in modifier keys compared to a
modern USB keyboard, so some sacrifices had to be made. **PF3** and
**PF4** do not map to **F3** and **F4**. Instead, they map as follows

| VT100 Key | USB Keyboard Key             |
|:---------:|:----------------------------:|
| PF3       | Alt (PC) / Option (Mac)      |
| PF4       | Windows (PC) / Command (Mac) |


## 4.0 Keyboard Configuration

The VT100 keyboard's built-in speaker can be used to generate
key-clicks and an audible bell if the options are set in the USB
converter's firmware. The options are persisted in the AVR's
non-volatile EEPROM between restarts.

### 4.1 Bell (SET-UP + B)

To toggle an audible bell when the **^G** (control-G, ASCII BEL)
character is sent, hold down **SET-UP** and press **B**. The first LED
(labelled **L1** on the keyboard) will illuminate when the audible
bell is enabled.

### 4.2 Key Clicks (SET-UP + K)

To toggle key clicks, hold down **SET-UP** and press **K**. The second
LED (labelled **INSERT** on the keyboard) will illuminate to show that
key clicks are enabled.

## 5.0 License and Credits

This project is Copyright &copy; 2013 by Seth Morabito, and is
distributed under the MIT license. See the file LICENSE.txt for
details.

This code makes use of the Teensy USB Keyboard Example Code
(`usb_keyboard.c` and `usb_keyboard.h`). This code is Copyright &copy;
2009 by PJRC.COM, LLC, and is also distributed by PJRC under the MIT
license.

