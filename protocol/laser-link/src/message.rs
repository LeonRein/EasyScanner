//! The messages. Their encoding is postcard's: enum variants by index,
//! integers as varints. Append new variants and fields only at the end, and
//! bump [`crate::PROTOCOL_VERSION`] whenever the encoding changes.

use serde::{Deserialize, Serialize};

/// Host → device.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Request {
    /// Chosen by the host; the reply carries it back.
    pub id: u16,
    pub command: Command,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Command {
    /// Open a session: the device checks the version and arms its watchdog.
    Hello { protocol_version: u16 },
    /// Switch the laser (session only).
    SetLaser(Laser),
    /// The device's state; allowed without a session.
    GetStatus,
    /// Keep the session alive (session only). Every request does that; this
    /// one does nothing else.
    Heartbeat,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Laser {
    Off,
    On,
}

/// Device → host.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum FromDevice {
    /// The answer to the request with this id.
    Reply { id: u16, result: Result<Reply, Error> },
    /// Something the device reports on its own.
    Event(Event),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Reply {
    /// The session is open.
    Hello(DeviceInfo),
    /// The state after the command.
    Status(Status),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct DeviceInfo {
    pub protocol_version: u16,
    pub firmware: FirmwareVersion,
    /// A session ends, and the laser goes off, when no request arrives for
    /// this long.
    pub watchdog_ms: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct FirmwareVersion {
    pub major: u8,
    pub minor: u8,
    pub patch: u8,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Status {
    pub laser: Laser,
    pub link: Link,
    /// Time since the device started (ms, wraps after 49 days).
    pub uptime_ms: u32,
}

/// Who controls the laser.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Link {
    /// No session yet: the laser is on, for use without the app.
    Standalone,
    /// A host controls the laser.
    Session,
    /// The host went silent: the laser is off until a new
    /// [`Command::Hello`].
    Lost,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Error {
    /// The device speaks another protocol version.
    UnsupportedVersion { device: u16 },
    /// The command needs a session: send [`Command::Hello`] first.
    NoSession,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Event {
    /// A frame arrived damaged and was dropped; its request, if any, gets
    /// no reply.
    FrameRejected(crate::frame::FrameError),
    /// The watchdog ran out: the laser is off.
    SessionLost,
}
