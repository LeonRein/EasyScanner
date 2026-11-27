# Firmware

## `laser-switch/`: the laser switch

An ESP32-S3 (the scanner uses a Waveshare ESP32-S3-Zero) sits in the
scanner's electronics box. The phone powers it over USB-C (OTG) and it
switches the line laser through a small logic-level MOSFET module.

- the module's signal input (TRIG/PWM) to **GPIO 1**, its signal ground to **GND**
- the module's supply input (DC+ / DC−) to the board's **5V** and **GND**
  (5V is the phone's USB supply)
- the laser to the module's output (OUT+ red, OUT− black)

The module carries the gate resistor and the gate pull-down, so the laser
stays off while the ESP32 boots; the firmware switches it **on as soon as it
runs**. A scan therefore needs nothing but the cable. The phone can switch
the laser over the same USB connection (CDC serial, 115200 baud, one command
per line): `on`, `off`, `?`; each is answered with `laser on` or
`laser off`. The app does not use this yet.

The GPIO is `LASER_PIN` at the top of the sketch; change it if the module is
wired to another pin.

### Flashing

Arduino IDE 2:

1. File → Preferences → Additional boards manager URLs:
   `https://espressif.github.io/arduino-esp32/package_esp32_index.json`,
   then Boards Manager → install **esp32** by Espressif.
2. Open `firmware/laser-switch/laser-switch.ino`.
3. Tools → Board → **ESP32S3 Dev Module**, Tools → **USB CDC On Boot: Enabled**.
4. Connect the board to the computer and Upload. If the port does not show
   up, hold BOOT while plugging it in, upload, then unplug and replug it.

Or with arduino-cli:

```sh
arduino-cli core install esp32:esp32 --additional-urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli compile --fqbn esp32:esp32:esp32s3:CDCOnBoot=cdc firmware/laser-switch
arduino-cli upload  --fqbn esp32:esp32:esp32s3:CDCOnBoot=cdc -p <port> firmware/laser-switch
```

Check it from the computer before it goes into the scanner: the laser lights
up a second after plugging in, and a serial terminal at 115200 baud answers
`?` with `laser on`.
