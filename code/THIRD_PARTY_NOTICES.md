# Third-party notices

This repository's own code is `GPL-3.0-or-later` (see `LICENSE`). It is built against, and the resulting firmware images contain code from, the following components. They are **not** copied into this repository; you obtain them when you set up the build.

| Component | Licence | Where it comes in |
|---|---|---|
| [Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk) | BSD-3-Clause | Build dependency (`pico_sdk_import.cmake`, `pico_stdlib`, `hardware_*`, ...). Linked into every firmware image |
| [TinyUSB](https://github.com/hathach/tinyusb) | MIT | Pulled in by the SDK because `CMakeLists.txt` enables USB stdio (`pico_enable_stdio_usb`) |
| Compiler runtime / C library from your ARM toolchain (for example newlib, libgcc) | Various, with their own runtime exceptions | Linked into the firmware by the toolchain |

## What to do when you distribute binaries

* Ship the full text of the licences above with the binary (the Pico SDK repository contains its `LICENSE.TXT`, and TinyUSB its own licence file). BSD-3-Clause and MIT require that the copyright notice and licence text accompany redistributed binaries.
* Offer the corresponding source for this repository under `GPL-3.0-or-later`, for example by linking the repository at the matching tag in your release notes.
* BSD-3-Clause and MIT are compatible with the GPL, so combining them in a GPLv3-or-later firmware is permitted.

## Other code

If you add code from anywhere else (a font table, a display driver, a PIO example, ...), keep its original licence header and add a row here.
