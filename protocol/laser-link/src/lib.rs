//! The link between the EasyScanner app and the laser switch (an ESP32-S3
//! on the scanner, powered by the phone over USB-C).
//!
//! - [`message`]: what the two sides say to each other, as typed enums. A
//!   session starts with [`Command::Hello`], which checks
//!   [`PROTOCOL_VERSION`]; every request carries an id its reply echoes, and
//!   errors are typed.
//! - [`frame`]: how a message travels over the byte stream: postcard, a
//!   CRC-16 over it, COBS, and a `0x00` delimiter, so a receiver can
//!   resynchronise on any delimiter and never acts on a damaged message.
//! - [`device`]: the switch's behaviour as a pure state machine, so it is
//!   tested on the host and the firmware only moves bytes and sets a pin.
//!   Without a session the laser is on (a scan needs nothing but the
//!   cable); inside one the app must send a request at least every
//!   [`DeviceInfo::watchdog_ms`], or the laser goes off.
//!
//! `no_std`, no allocation: the same crate runs in the firmware and in the
//! app's Rust core.

#![no_std]

pub mod device;
pub mod frame;
pub mod message;

pub use device::{Device, DeviceConfig};
pub use frame::{FrameDecoder, FrameError, encode};
pub use message::*;

/// Bumped on every change to the messages' encoding. A device answers a
/// [`Command::Hello`] of another version with [`Error::UnsupportedVersion`].
pub const PROTOCOL_VERSION: u16 = 1;
