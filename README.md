VT100 to USB Converter Firmware
===============================

**Version:** 0.1
**Last Updated:** 15-July-2013

## 1.0 About

This project contains source code and schematics for a DEC VT100 terminal keyboard to USB converter.

The purpose of the converter is to allow a DEV VT100 terminal keyboard to be used as a USB input device on a modern computer. This is, of course, almost completely useless, but that doesn't mean it's not fun.

## 2.0 Description

The converter uses a [Teensy 2.0] (http://www.pjrc.com/teensy/) development board powered by an ATmega32U4 microcontroller, which this firmware targets. In addition to the microcontroller, the converter uses an external HD6402 UART, an external clock generator, and a voltage comparator for signalling with the VT100 keyboard.

The firmware is written in AVR C, and can be compiled with the avr-gcc tool chain. The resulting firmware should be loaded onto the Teensy with the Teensy loader.