# automat firmware

Firmware for the [dadamachines automat](https://dadamachines.com/product/automat-controller/) MIDI-to-solenoid controller.

Built with [PlatformIO](https://platformio.org/) using the [Adafruit Arduino SAMD Core](https://github.com/adafruit/ArduinoCore-samd).

## Hardware

- MCU: SAMD21G18A (Cortex-M0+, 48 MHz)
- 256 KB Flash / 32 KB RAM
- 12 solenoid outputs via SPI shift registers
- USB MIDI + DIN MIDI input
- UF2 bootloader for drag-and-drop firmware updates

## USB Power & iOS Compatibility

The firmware declares a USB bus-current draw of 100 mA (Adafruit SAMD core default). The original dadamachines BSP declared 20 mA. Both values are well within the USB 2.0 limit of 500 mA, so the automat enumerates normally on iPads and other iOS devices. Solenoid power is supplied externally and is not drawn from USB.

## Configuration

Use the [automat configurator](https://automat-configurator.dadamachines.com/) (WebMIDI) to set per-output velocity programs, min/max timing, and gate durations. Source: [dadamachines/automat-configurator](https://github.com/dadamachines/automat-configurator)

## Build

### Prerequisites

- [PlatformIO CLI](https://docs.platformio.org/en/latest/core/installation.html) or [PlatformIO IDE (VS Code extension)](https://platformio.org/platformio-ide)

### Compile

```bash
pio run
```

After a successful build the firmware files are in `.pio/build/automat/`:
- `firmware.bin` — raw binary
- `automat-sw_<version>.automat.uf2` — UF2 file for drag-and-drop flashing

### Upload via SAM-BA (serial)

```bash
pio run -t upload
```

### Flash via UF2

1. Double-tap the reset button on the automat to enter the UF2 bootloader.
2. A USB drive named `AUTOMAT` will appear.
3. Drag `automat-sw_<version>.automat.uf2` onto that drive.
4. The board resets automatically and runs the new firmware.

> **Note:** UF2 flashing requires the [dadamachines UF2 bootloader](https://github.com/dadamachines/uf2-samdx1/tree/dadamachines). If your automat still has the original SAM-BA-only bootloader, use `pio run -t upload` instead, or flash the UF2 bootloader once via SWD/JTAG.

## Dependencies

Managed automatically by PlatformIO (declared in `platformio.ini`):

| Library | Source |
|---|---|
| FlashStorage | [cmaglie/FlashStorage](https://github.com/cmaglie/FlashStorage) |
| Arduino MIDI Library | [FortySevenEffects/arduino_midi_library](https://github.com/FortySevenEffects/arduino_midi_library) |
| MIDIUSB | [arduino-libraries/MIDIUSB](https://github.com/arduino-libraries/MIDIUSB) |
| OneButton | [mathertel/OneButton](https://github.com/mathertel/OneButton) |

## Project Structure

```
├── platformio.ini              # PlatformIO build configuration
├── boards/
│   └── dadamachines_automat.json   # Custom board definition
├── variants/
│   └── automat/
│       ├── variant.h           # Pin mapping
│       └── variant.cpp         # Pin descriptions
├── src/
│   ├── main.cpp                # Main firmware
│   ├── automatConstants.h      # Board constants
│   ├── solenoidSPI.*           # Shift-register solenoid driver
│   ├── dadaMidiLearn.h         # MIDI learn mode
│   ├── dadaStatusLED.h         # Status LED controller
│   └── dadaSysEx.*             # SysEx configuration protocol
└── extra_scripts/
    └── post_build_uf2.py       # Automatic .bin → .uf2 conversion
```

## Change log

- V3.11 — Remove all PWM/motor code and files (PWMManager, humTiming); remove pitch bend and mod wheel handlers; clean up unused constants
- V3.10 — Simplify velocity programs: only Default (velocity-mapped timing) and NoteOn/NoteOff (always-on) remain active; SysEx/WebUI compatibility preserved
- V3.02 — Fix I2C message validation, pin bounds checks, PWMManager array bug
- V3.01 — Version bump, named UF2 output, README updates
- V3.00 — PlatformIO migration, Adafruit SAMD core, UF2 output
- V1.10 2018.04.15 — Velocity support added
- V1.00 2017.10.19 — Initial release

## Related Repositories

- [dadamachines/automat-configurator](https://github.com/dadamachines/automat-configurator) — WebMIDI configurator for velocity/gate settings
- [dadamachines/uf2-samdx1](https://github.com/dadamachines/uf2-samdx1/tree/dadamachines) — UF2 bootloader (fork of microsoft/uf2-samdx1)
- [dadamachines/arduino-board-index](https://github.com/dadamachines/arduino-board-index) — Legacy Arduino Board Support Package (superseded by this PlatformIO project)

## Contact

To report a bug, contribute, discuss on usage, or simply request support, please [create an issue here](https://github.com/dadamachines/automat-software/issues/new) or reach out via [dadamachines.com/contact](https://dadamachines.com/contact/).

## License

[GPLv3](https://github.com/dadamachines/automat-software/blob/master/LICENSE) Copyright © 2014-2026, Johannes Elias Lohbihler for [dadamachines](https://dadamachines.com)
