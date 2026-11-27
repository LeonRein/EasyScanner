//! The messages. Their encoding is postcard's: enum variants by index,
//! integers as varints.
//!
//! Evolving them: a new [`Command`] or [`Event`], or a new variant at the
//! end of any other enum, raises the minor [`crate::PROTOCOL_VERSION`]; a
//! device that does not know a command answers it with
//! [`Error::UnknownCommand`]. Anything else that changes the encoding
//! (a field added, removed or reordered) raises the major version, which a
//! device refuses at [`Command::Hello`].

use serde::{Deserialize, Serialize};

use crate::frame::FrameError;

/// The version of these messages.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ProtocolVersion {
    /// Both sides must speak the same major version.
    pub major: u16,
    /// Commands and events added within a major version.
    pub minor: u16,
}

impl ProtocolVersion {
    /// Whether a side speaking `self` can talk to one speaking `other`.
    pub fn compatible_with(self, other: ProtocolVersion) -> bool {
        self.major == other.major
    }
}

/// Host → device.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Request {
    /// Chosen by the host; the reply carries it back.
    pub id: u16,
    pub command: Command,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Command {
    /// Open a session: the device checks the major version and arms its
    /// watchdog. The reply says which minor version, so which commands, the
    /// device knows.
    Hello { protocol_version: ProtocolVersion },
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
    pub protocol_version: ProtocolVersion,
    pub firmware: FirmwareVersion,
    /// A session ends, and the laser goes off, when no request arrives for
    /// this long. A host sends one at least every third of it, so a single
    /// lost frame does not end the session.
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
    /// Time since the device started (ms). A smaller value than before
    /// means the device restarted.
    pub uptime_ms: u64,
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
    /// The device speaks another major protocol version.
    UnsupportedVersion { device: ProtocolVersion },
    /// The command needs a session: send [`Command::Hello`] first.
    NoSession,
    /// A command of a later minor version than the device's.
    UnknownCommand,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Event {
    /// A frame arrived damaged and was dropped; its request, if any, gets
    /// no reply.
    FrameRejected(FrameError),
    /// The watchdog ran out: the laser is off.
    SessionLost,
}
