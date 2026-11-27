//! EasyScanner's laser switch on an ESP32-S3 (Waveshare ESP32-S3-Zero).
//!
//! The phone powers the board over USB-C (OTG); one GPIO drives a
//! logic-level MOSFET module that switches the line laser. The board talks
//! `laser-link` (`protocol/laser-link`) over its built-in USB Serial/JTAG
//! port, a CDC-ACM serial port to the phone. What the switch does lives in
//! [`laser_link::Device`]; this file only moves bytes, keeps the time and
//! sets the pin. Nothing else writes to the port: no logging, so the host
//! only ever reads frames.

#![no_std]
#![no_main]

use core::panic::PanicInfo;

use esp_hal::Blocking;
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::peripherals::GPIO1;
use esp_hal::time::Instant;
use esp_hal::usb::usb_serial_jtag::{UsbSerialJtag, UsbSerialJtagTx};
use laser_link::device::Millis;
use laser_link::frame::MAX_FRAME;
use laser_link::{Device, DeviceConfig, Event, FirmwareVersion, FrameDecoder, FromDevice, Laser, encode};

esp_bootloader_esp_idf::esp_app_desc!();

/// A session ends, and the laser goes off, after this long without a request.
const WATCHDOG_MS: u32 = 1000;
/// A reply nobody reads (the phone powers the board but has no session
/// open) is dropped after this long instead of stalling the loop.
const TX_TIMEOUT_MS: Millis = 20;

#[esp_hal::main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let mut device = Device::new(DeviceConfig { firmware: FIRMWARE_VERSION, watchdog_ms: WATCHDOG_MS });
    // the MOSFET module's signal input; starts in the device's state (on)
    let mut laser = Output::new(peripherals.GPIO1, level(device.laser()), OutputConfig::default());
    let (mut rx, mut tx) = UsbSerialJtag::new(peripherals.USB_DEVICE).split();
    let mut decoder = FrameDecoder::new();

    loop {
        if let Ok(byte) = rx.read_byte() {
            let reply = match decoder.push_payload(byte) {
                None => None,
                Some(Ok(payload)) => Some(device.receive(payload, now_ms())),
                Some(Err(e)) => Some(FromDevice::Event(Event::FrameRejected(e))),
            };
            if let Some(reply) = reply {
                laser.set_level(level(device.laser()));
                send(&mut tx, &reply);
            }
        }
        // after every byte, so a stream of bytes cannot hold off the watchdog
        if let Some(event) = device.poll(now_ms()) {
            laser.set_level(level(device.laser()));
            send(&mut tx, &FromDevice::Event(event));
        }
    }
}

/// A bug in the firmware: switch the laser off and stop. The board stays
/// halted until it is unplugged, rather than restarting into
/// [`laser_link::Link::Standalone`] with the laser on.
#[panic_handler]
fn panic(_: &PanicInfo) -> ! {
    // SAFETY: `main` never runs again, so this is the only user of the pin from here on.
    let pin = unsafe { GPIO1::steal() };
    let _off = Output::new(pin, Level::Low, OutputConfig::default());
    loop {
        core::hint::spin_loop();
    }
}

fn level(laser: Laser) -> Level {
    match laser {
        Laser::On => Level::High,
        Laser::Off => Level::Low,
    }
}

fn now_ms() -> Millis {
    Instant::now().duration_since_epoch().as_millis()
}

/// Send one message; give up on it when the host does not read for
/// [`TX_TIMEOUT_MS`]. A frame cut short is dropped by the host's decoder at
/// the next delimiter.
fn send(tx: &mut UsbSerialJtagTx<'_, Blocking>, msg: &FromDevice) {
    let mut buf = [0u8; MAX_FRAME];
    // every message fits a frame (laser-link's `every_message_fits_a_frame`)
    let Ok(frame) = encode(msg, &mut buf) else { return };
    let deadline = now_ms() + TX_TIMEOUT_MS;
    for &byte in frame {
        if !retry_until(deadline, || tx.write_byte_nb(byte)) {
            return;
        }
    }
    retry_until(deadline, || tx.flush_tx_nb());
}

/// Poll a non-blocking operation until it completes (true), fails or the
/// deadline passes (false).
fn retry_until<E>(deadline: Millis, mut op: impl FnMut() -> nb::Result<(), E>) -> bool {
    loop {
        match op() {
            Ok(()) => return true,
            Err(nb::Error::WouldBlock) if now_ms() < deadline => {}
            Err(_) => return false,
        }
    }
}

const FIRMWARE_VERSION: FirmwareVersion = FirmwareVersion {
    major: parse_u8(env!("CARGO_PKG_VERSION_MAJOR")),
    minor: parse_u8(env!("CARGO_PKG_VERSION_MINOR")),
    patch: parse_u8(env!("CARGO_PKG_VERSION_PATCH")),
};

/// A version component at compile time; one that is not a `u8` fails the build.
const fn parse_u8(s: &str) -> u8 {
    let bytes = s.as_bytes();
    assert!(!bytes.is_empty(), "empty version component");
    let mut value: u16 = 0;
    let mut i = 0;
    while i < bytes.len() {
        assert!(bytes[i].is_ascii_digit(), "version component is not a number");
        value = value * 10 + (bytes[i] - b'0') as u16;
        assert!(value <= u8::MAX as u16, "version component does not fit a u8");
        i += 1;
    }
    value as u8
}
