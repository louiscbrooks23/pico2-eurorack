# pico2-eurorack

Firmware projects developed for the **pico2-eurorack** development board: an RP2350 (Raspberry Pi Pico 2 class) based Eurorack-style audio and CV platform.

This repository has two kinds of folder: the **hardware** design files for the board itself, and self-contained **firmware projects** that run on it, each with its own `README.md`, build files and tests.

## Hardware

| Folder | Description |
|---|---|
| [`hardware/`](hardware/) | Schematic, PCB and assembly files for the two-board development board (DSP board and control board, revision V0.2), with pinouts, power, analog signal paths and a bring-up checklist. Start with [`hardware/README.md`](hardware/README.md). |

## Projects

| Folder | Description |
|---|---|
| [`multi-dsp/`](multi-dsp/) | Eleven-module audio DSP and CV firmware (resonator, granular, quantizer, oscillators, wavefolder, function generator, multi-effect and more) with an OLED scope/spectrum UI. C++ on the Pico SDK. |

## Board features used by the current projects

* RP2350 running at 144 MHz, dual core (audio on core 0, UI and control scanning on core 1)
* TI PCM3060 stereo audio codec, driven by a PIO I²S master
* MCP4728 4-channel 12-bit DAC for CV outputs
* MCP3208 8-channel ADC for CV inputs and pots, plus the RP2350's own ADC channels
* SSD1306-style 128×64 OLED, rotary encoder with push switch, and two buttons

The full hardware description (schematic, pin map, analog stages, power) is in [`hardware/README.md`](hardware/README.md). Each firmware project's README documents the clocking and pin usage its code relies on (see [`multi-dsp/README.md`](multi-dsp/README.md), section 3).

## Getting started

Open the folder of the project you want and follow its `README.md`. Projects are built with CMake and the [Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk); host-side tests (where provided) need only a C++17 compiler.

## Licence

Copyright (C) 2026 Louis Brooks.

This repository is free software, released under the **GNU General Public License, version 3 or (at your option) any later version** (`GPL-3.0-or-later`). See [`LICENSE`](LICENSE) for the full text; every source file also carries an SPDX header. You may use, study, modify and redistribute it, including built firmware images, provided you pass on the same freedoms and make the corresponding source available.

Third-party components used at build time (the Raspberry Pi Pico SDK and what it pulls in) keep their own licences; see [`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md).

Contributions are welcome. By submitting a contribution you agree that it is licensed under the same terms (`GPL-3.0-or-later`).
