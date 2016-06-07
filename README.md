# beacon-keyer
Keyer software for a radio beacon, to run on an ATtiny45 microcontroller.

The make file is for WinAVR.

The initial message is defined in `fsm.c` in the `vMsg[]` array and stored in EEPROM via the `keyer.eep` file.
The message can be changed via the serial port.
