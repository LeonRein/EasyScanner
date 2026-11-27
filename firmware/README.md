# Firmware

## `laser-switch/`: the laser switch

An ESP32-S3 (the scanner uses a Waveshare ESP32-S3-Zero) sits in the
scanner's electronics box. The phone powers it over USB-C (OTG), and it
switches the line laser through a small logic-level MOSFET module:

- the module's signal input (TRIG/PWM) to **GPIO 1**, its signal ground to **GND**
- the module's supply input (DC+ / DC−) to the board's **5V** and **GND**
  (5V is the phone's USB supply)
- the laser to the module's output (OUT+ red, OUT− black)

The module's gate pull-down keeps the laser off while the ESP32 boots.

### Behaviour

The firmware is written in Rust (`esp-hal`) and speaks
[`laser-link`](../protocol/laser-link/src/lib.rs) over the chip's built-in
USB Serial/JTAG port, which the phone sees as a CDC-ACM serial port:
typed postcard messages with a CRC-16 in COBS frames, a version handshake,
request ids and typed errors. The behaviour is `laser_link::Device`, tested
on the host:

| State | Laser | Leaves it |
|---|---|---|
| *Standalone* (after power-up) | **on**: a scan needs nothing but the cable | `Hello` with the same protocol version |
| *Session* | as the app sets it | no request for 1 s (watchdog) |
| *Lost* | **off**: an app that crashed leaves no laser shining | a new `Hello` |

The app does not open a session yet, so for now the laser is simply on
whenever the scanner is plugged in.

### Flashing

The CI builds the firmware on every change: Actions → *Firmware build* → the
run → Artifacts → `laser-switch-esp32s3` (an ELF file). Flash it with
[espflash](https://github.com/esp-rs/espflash):

```sh
cargo install espflash
espflash flash --chip esp32s3 laser-switch          # the unzipped artifact
```

Building it yourself needs Espressif's Xtensa Rust toolchain:

```sh
cargo install espup && espup install                 # then source ~/export-esp.sh
cd firmware/laser-switch
cargo run --release                                  # builds, flashes and opens the monitor
```

If the board is not found, hold BOOT while plugging it in, flash, then
unplug and replug it. Check it before it goes into the scanner: the laser
lights up a second after plugging in.

Note for the app side: the USB Serial/JTAG port resets the chip on the same
DTR/RTS sequence esptool uses; a host should leave both lines unchanged.
