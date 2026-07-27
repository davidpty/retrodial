# RetroDial

Bring a vintage rotary phone back to life on a modern telephone exchange. RetroDial converts pulse dialing to DTMF tones in real time, running on a single ATtiny85 hidden inside the phone with no visible modifications.

Built around the Siemens W48 with full support for the `Earth key` (German: Erdtaste), it goes beyond basic pulse conversion with last-number redial, speed-dial, Direct Dial Mode, auto-dial on pickup, `*` and `#` hold gestures, pause support, and adjustable DTMF timing. Everything is configured directly from the rotary dial - no computer needed after flashing.

![Siemens W48 rotary phone](w48-phone.png)

This project is a fork of the rotarydial firmware, extended with Earth key support, auto-dial, speed-dial, Direct Dial Mode, and configurable DTMF and menu timing.

## Hardware

Needed build parts:

* 1x ATTINY85V
* 1x 78L05 5V regulator
* 1x 4 MHz crystal
* 1x 680 ohm resistor
* 1x 27 V 1 W zener diode
* 2x 22 pF ceramic capacitors
* 4x 0.1 uF (100 nF) ceramic capacitors
* 1x 4 position terminal
* 1x 1N4007 diode

## Circuit

A Fritzing layout is included with a compact footprint designed to fit inside the W48 housing without modifications. See `pcb.fzz`.

## Wiring

### Rotary dial

* Disconnect the rotary dial from the phone circuitry. Do not connect it in parallel with the original phone circuit.
* Connect it to the converter:

  * Pulse contact, normally closed and opening for each pulse, to `PULSE`
  * Dial contact, closed while the dial is moving, to `DIAL`

### Earth key

* Disconnect the `Earth key` from the phone circuitry.
* Connect it to the same `DIAL` line used by the rotary dial:

  * One side to `DIAL`
  * Other side to `GND`
* Pressing the `Earth key` must pull `DIAL` to `GND`.

### Power

* Connect power after the hookswitch:

  * Line `+` to `VCC`
  * Line `-` to `GND`
* The device must be off when the phone is on-hook.
* Optional: add a diode, for example 1N4007, between line `+` and `VCC` for protection:

  * Anode to line `+`
  * Cathode to `VCC`

## Build

Install tools:

```bash
sudo apt update && sudo apt install gcc-avr avr-libc binutils-avr avrdude make
```

Build, flash and set fuses:

```bash
make install              # default 4 MHz external crystal
make CLOCK_MODE=8 install # 8 MHz internal oscillator, for testing only
```

Erase EEPROM, clearing all stored numbers and settings:

```bash
make erase
```

## Operation

### Normal dialing

Dial a number and release. The firmware sends the matching DTMF tone.

If you pause for more than about 3 seconds between digits, a pause is automatically inserted into the redial memory at that point. If you wait more than about 6 seconds, two pauses are inserted. This allows redial to correctly replay numbers that require waiting for an automated phone menu or office switchboard to respond.

### Earth key

The `Earth key` is a quick redial and speed-dial shortcut.

* Brief press and release: redial the last number
* Hold until the first beep: dial the stored Earth key speed-dial number
* Hold until the second beep: program the Earth key speed-dial number
* Press during startup: cancel auto-dial for that cycle

### Rotary hold menu

Use the rotary dial for special functions.

Dial a digit and hold until the first beep, then release:

* `1` = send `*`
* `2` = send `#`
* `3` = redial last number
* `4` to `9`, `0` = dial the stored speed-dial number

Keep holding until the second beep:

* `4` to `9`, `0` = enter programming mode for that speed-dial slot
* `1` = cycle DTMF tone duration, 100 ms or 200 ms
* `2` = cycle menu hold time, 1 s or 2 s
* `3` = enter auto-dial setup

### Speed-dial programming

1. Dial the slot digit.
2. Keep holding until the second beep, then release.
3. Enter the number one digit at a time.
4. Release after each digit.
5. Hang up when done.

Notes:

* Each digit is written to EEPROM immediately.
* `*`, `#`, and pause can be stored by dialing `1`, `2`, or `3`, holding until the first beep, then releasing.
* The Earth key has its own dedicated speed-dial slot, programmed by holding the Earth key until the second beep.
* During programming, you can copy into the current slot:

  * Press the `Earth key` to load the last dialed number.
  * Dial a source speed-dial slot, `4` to `9` or `0`, and hold until the first beep.
* Hold `0` until the second beep to clear the current slot. This also works for the Earth key slot.

### Auto-dial setup

Auto-dial is stored in one of the speed-dial slots and dials automatically after power-up if enabled.

Set auto-dial:

1. Dial `3`.
2. Hold until the second beep, then release.
3. Dial the speed-dial slot, or press the `Earth key` to use the Earth key speed-dial number as the auto-dial number.
4. The auto-dial is saved with the default 5 second delay.
5. Optional: dial a delay digit from `0` to `9` to override the delay.

Clear auto-dial:

1. Dial `3`.
2. Hold until the second beep, then release.
3. Hold `0` until the second beep, then release.

Pressing the `Earth key` or starting to dial during startup cancels auto-dial for that cycle.

## Service Codes

Service codes are entered during a normal call using the rotary hold menu for `*` and `#`.

### Direct Dial Mode

Direct Dial Mode turns digits `4` to `9` and `0` into instant speed-dial keys. Pressing any of them immediately dials the stored number without holding. The Earth key also dials its assigned speed-dial slot directly.

Set it up with a 3-digit PIN `PPP`, using digits `0` to `9` only:

* `**PPP` enables Direct Dial with Programming Lock:

  * Normal dialing still works.
  * Speed-dial programming, settings changes, and factory reset are blocked.
* `*#PPP` enables Direct Dial Only:

  * Normal dialing is blocked.
  * Digits `4` to `9` and `0` directly dial their assigned speed-dial slots.
  * The Earth key directly dials the Earth key speed-dial slot.
  * Speed-dial programming, settings changes, and factory reset are blocked.

Disable the active lock mode:

* `**PPP` or `*#PPP` with the correct PIN permanently disables the active lock mode and clears the PIN.
* `##PPP` with the correct PIN temporarily disables the active lock mode for the current call only.

To enter a PIN when Direct Dial Only is active:

1. Dial `*` or `#` twice via first-level hold as the first two actions after pickup.
2. Any combination works: `**`, `##`, `*#`, or `#*`.
3. This arms PIN entry mode.
4. Dial the three PIN digits directly.
5. These digits feed the PIN sequence without triggering any speed-dial slot.

Use `**PPP` or `*#PPP` to permanently disable the active lock mode, or `##PPP` to disable it for the current call only.

### Factory reset

Dial `*#0#*` to erase all stored numbers, speed-dial slots, auto-dial configuration, and reset DTMF duration and menu hold time to defaults.

Factory reset is blocked when a lock mode is active.

## Credits

RetroDial builds on the work of [Boris Cherkasskiy](http://boris0.blogspot.ca/2013/09/rotary-dial-for-digital-age.html), who created the original firmware in 2011, [Arnie Weber](https://bitbucket.org/310weber/rotary_dial/), who reworked the hardware in 2015, and [Matthew Millman](http://tech.mattmillman.com/), who cleaned up the implementation in 2018.

