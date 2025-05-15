# fabX Keypad Firmare

fabX Keypad Firmare acts as second factor interface for the [fabX](https://github.com/fabXlabs/fabX) access system.
It is intended for running on an ATSAMD21G18A on [fabX_doorKeyPCB](https://github.com/fabXlabs/fabX_doorKeyPCB)
acting as an SPI peripheral.

## Overview
The fabX access system consists of a backend server ([fabX](https://github.com/fabXlabs/fabX))
and two main hardware components:
* an M5Stack device, also known as the **fabX device**, running [device-firmware](https://github.com/fabXlabs/device-firmware)
* an optional [fabX_doorKeyPCB](https://github.com/fabXlabs/fabX_doorKeyPCB).

The fabX device interfaces with an RFID-RC522 board via an interface board, such as the
[fabX_compactPCB](https://github.com/fabXlabs/fabX_compactPCB) or [fabX_basePCB](https://github.com/fabXlabs/fabX_basePCB)
to read RFID cards for user authentication.

If the backend requires a second factor, the fabX device communicates with the
[fabX_doorKeyPCB](https://github.com/fabXlabs/fabX_doorKeyPCB), which features a keypad and OLED display,
to prompt the user for a PIN code.

This PIN is sent to the backend for verification, and access is granted upon successful authentication.

## Build & Flash

### Build the firmware
Building is done via the PlatformIO ecosystem.
```sh
pio run
```

### Build and upload

Once a generic Arduino zero bootloader build with the CRYSTALLESS option is flashed to the board building & uploading can be performed via
```sh
pio run -b upload
```
