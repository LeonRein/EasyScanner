// EasyScanner laser switch (ESP32-S3, e.g. Waveshare ESP32-S3-Zero).
//
// The board is powered by the phone over USB-C (OTG) and drives the signal
// input of a logic-level MOSFET module that switches the laser. The laser is on as
// soon as the scanner is plugged in, so a scan needs nothing but the cable;
// the phone can switch it over the same USB connection (CDC serial, one
// command per line):
//
//   on     laser on
//   off    laser off
//   ?      report the state
//
// Every command is answered with "laser on" or "laser off", anything else
// with "error: ...". Build: Arduino IDE or arduino-cli with the esp32 core,
// board "ESP32S3 Dev Module", "USB CDC On Boot: Enabled" (see README.md).

#include <Arduino.h>

// GPIO wired to the MOSFET module's signal input.
constexpr uint8_t LASER_PIN = 1;
// Longest command accepted; longer lines are discarded.
constexpr size_t MAX_LINE = 16;

namespace {

bool laserOn = false;
String line;
bool lineTooLong = false;

void setLaser(bool on) {
  laserOn = on;
  digitalWrite(LASER_PIN, on ? HIGH : LOW);
}

void reportState() { Serial.println(laserOn ? "laser on" : "laser off"); }

void handleCommand(const String& cmd) {
  if (cmd == "on") {
    setLaser(true);
  } else if (cmd == "off") {
    setLaser(false);
  } else if (cmd != "?") {
    Serial.println("error: unknown command (on, off, ?)");
    return;
  }
  reportState();
}

}  // namespace

void setup() {
  pinMode(LASER_PIN, OUTPUT);
  setLaser(true);
  Serial.begin(115200);
}

void loop() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\n' || c == '\r') {
      line.trim();
      if (lineTooLong) {
        Serial.println("error: line too long");
      } else if (line.length() > 0) {
        handleCommand(line);
      }
      line = "";
      lineTooLong = false;
    } else if (line.length() < MAX_LINE) {
      line += c;
    } else {
      lineTooLong = true;
    }
  }
}
