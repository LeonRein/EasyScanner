//! The laser switch's behaviour, independent of the hardware: the firmware
//! feeds it frames and the time, and sets the laser pin to [`Device::laser`].
//!
//! - Without a session ([`Link::Standalone`]) the laser is on, so the
//!   scanner works with nothing but the cable.
//! - [`Command::Hello`] with a compatible [`PROTOCOL_VERSION`] opens a
//!   session; from then on only the host switches the laser.
//! - In a session every request resets the watchdog. When it runs out the
//!   laser goes off ([`Link::Lost`], [`Event::SessionLost`]) and stays off
//!   until a new Hello: an app that crashed must not leave a laser shining.
//!   A request that arrives after the deadline finds the session already
//!   lost, however late [`Device::poll`] runs.

use crate::PROTOCOL_VERSION;
use crate::frame::FrameError;
use crate::message::{Command, DeviceInfo, Error, Event, FirmwareVersion, FromDevice, Laser, Link, Reply, Request, Status};

/// Milliseconds on the device's monotonic clock.
pub type Millis = u64;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DeviceConfig {
    pub firmware: FirmwareVersion,
    /// A session without a request for this long ends.
    pub watchdog_ms: u32,
}

#[derive(Debug)]
pub struct Device {
    cfg: DeviceConfig,
    laser: Laser,
    link: Link,
    /// When the session ends without another request.
    deadline: Millis,
    /// The session ended and [`Device::poll`] has not reported it yet.
    lost_unreported: bool,
}

impl Device {
    pub const fn new(cfg: DeviceConfig) -> Self {
        Device { cfg, laser: Laser::On, link: Link::Standalone, deadline: 0, lost_unreported: false }
    }

    /// The state the laser pin must have.
    pub fn laser(&self) -> Laser {
        self.laser
    }

    pub fn link(&self) -> Link {
        self.link
    }

    /// Answer one checked frame payload received at `now`. The request id
    /// is read before the command, so a command of a later protocol
    /// version is still answered, with [`Error::UnknownCommand`].
    pub fn receive(&mut self, payload: &[u8], now: Millis) -> FromDevice {
        let Ok((id, command)) = postcard::take_from_bytes::<u16>(payload) else {
            return FromDevice::Event(Event::FrameRejected(FrameError::Message));
        };
        let result = match postcard::from_bytes::<Command>(command) {
            Ok(command) => self.execute(command, now),
            Err(_) => {
                self.begin_request(now);
                Err(Error::UnknownCommand)
            }
        };
        FromDevice::Reply { id, result }
    }

    /// Answer one request received at `now`.
    pub fn handle(&mut self, req: Request, now: Millis) -> FromDevice {
        FromDevice::Reply { id: req.id, result: self.execute(req.command, now) }
    }

    fn execute(&mut self, command: Command, now: Millis) -> Result<Reply, Error> {
        self.begin_request(now);
        match command {
            Command::Hello { protocol_version } => {
                if !PROTOCOL_VERSION.compatible_with(protocol_version) {
                    return Err(Error::UnsupportedVersion { device: PROTOCOL_VERSION });
                }
                self.link = Link::Session;
                self.feed_watchdog(now);
                return Ok(Reply::Hello(self.info()));
            }
            Command::GetStatus => {}
            Command::SetLaser(laser) => {
                self.require_session()?;
                self.laser = laser;
            }
            Command::Heartbeat => self.require_session()?,
        }
        Ok(Reply::Status(self.status(now)))
    }

    /// Check the watchdog; call it regularly. Returns the event to send
    /// when the session has ended since the last call.
    pub fn poll(&mut self, now: Millis) -> Option<Event> {
        self.expire(now);
        core::mem::take(&mut self.lost_unreported).then_some(Event::SessionLost)
    }

    pub fn status(&self, now: Millis) -> Status {
        Status { laser: self.laser, link: self.link, uptime_ms: now }
    }

    pub fn info(&self) -> DeviceInfo {
        DeviceInfo { protocol_version: PROTOCOL_VERSION, firmware: self.cfg.firmware, watchdog_ms: self.cfg.watchdog_ms }
    }

    /// What every request does first: end a session whose deadline passed
    /// before it arrived, and keep a live one alive.
    fn begin_request(&mut self, now: Millis) {
        self.expire(now);
        if self.link == Link::Session {
            self.feed_watchdog(now);
        }
    }

    fn require_session(&self) -> Result<(), Error> {
        match self.link {
            Link::Session => Ok(()),
            Link::Standalone | Link::Lost => Err(Error::NoSession),
        }
    }

    fn expire(&mut self, now: Millis) {
        if self.link == Link::Session && now >= self.deadline {
            self.link = Link::Lost;
            self.laser = Laser::Off;
            self.lost_unreported = true;
        }
    }

    fn feed_watchdog(&mut self, now: Millis) {
        self.deadline = now + Millis::from(self.cfg.watchdog_ms);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::message::ProtocolVersion;

    const CFG: DeviceConfig = DeviceConfig { firmware: FirmwareVersion { major: 0, minor: 1, patch: 0 }, watchdog_ms: 1000 };

    fn req(id: u16, command: Command) -> Request {
        Request { id, command }
    }

    fn hello() -> Command {
        Command::Hello { protocol_version: PROTOCOL_VERSION }
    }

    #[test]
    fn standalone_the_laser_is_on_and_only_status_is_answered() {
        let mut d = Device::new(CFG);
        assert_eq!(d.laser(), Laser::On);
        assert_eq!(d.handle(req(1, Command::SetLaser(Laser::Off)), 0), FromDevice::Reply { id: 1, result: Err(Error::NoSession) });
        assert_eq!(d.laser(), Laser::On);
        let status = Status { laser: Laser::On, link: Link::Standalone, uptime_ms: 5 };
        assert_eq!(d.handle(req(2, Command::GetStatus), 5), FromDevice::Reply { id: 2, result: Ok(Reply::Status(status)) });
        assert_eq!(d.poll(1_000_000), None, "no watchdog without a session");
    }

    #[test]
    fn hello_checks_the_version() {
        let mut d = Device::new(CFG);
        let next_major = ProtocolVersion { major: PROTOCOL_VERSION.major + 1, minor: 0 };
        let old = d.handle(req(1, Command::Hello { protocol_version: next_major }), 0);
        assert_eq!(old, FromDevice::Reply { id: 1, result: Err(Error::UnsupportedVersion { device: PROTOCOL_VERSION }) });
        assert_eq!(d.link(), Link::Standalone);
        // a later minor version is welcome: the reply tells it which commands the device knows
        let next_minor = ProtocolVersion { minor: PROTOCOL_VERSION.minor + 1, ..PROTOCOL_VERSION };
        let reply = d.handle(req(2, Command::Hello { protocol_version: next_minor }), 0);
        assert_eq!(reply, FromDevice::Reply { id: 2, result: Ok(Reply::Hello(d.info())) });
        assert_eq!(d.link(), Link::Session);
    }

    #[test]
    fn a_request_after_the_deadline_finds_the_session_lost() {
        // the firmware was busy and polled late: the late request must not revive the session
        let mut d = Device::new(CFG);
        d.handle(req(1, hello()), 0);
        assert_eq!(d.handle(req(2, Command::Heartbeat), 1500), FromDevice::Reply { id: 2, result: Err(Error::NoSession) });
        assert_eq!((d.laser(), d.link()), (Laser::Off, Link::Lost));
        assert_eq!(d.poll(1500), Some(Event::SessionLost), "still reported");
    }

    #[test]
    fn an_unknown_command_is_answered_and_keeps_the_session() {
        let mut d = Device::new(CFG);
        d.handle(req(1, hello()), 0);
        // id 0x0105 as a varint, then a command variant no version-1.0 device knows
        let reply = d.receive(&[0x85, 0x02, 0x7F], 900);
        assert_eq!(reply, FromDevice::Reply { id: 0x0105, result: Err(Error::UnknownCommand) });
        assert_eq!(d.poll(1800), None, "the request fed the watchdog");
        assert_eq!(d.receive(&[], 1800), FromDevice::Event(Event::FrameRejected(FrameError::Message)));
        let mut payload = [0u8; 8];
        let known = postcard::to_slice(&req(3, Command::SetLaser(Laser::Off)), &mut payload).unwrap();
        assert!(matches!(d.receive(known, 1850), FromDevice::Reply { id: 3, result: Ok(_) }));
        assert_eq!(d.laser(), Laser::Off);
    }

    #[test]
    fn the_encoding_is_pinned() {
        // a change here breaks every deployed device or app: raise the major version
        // (the frame: CRC-16/IBM-SDLC of [1, 1] is 0x0716, then COBS)
        let mut buf = [0u8; 32];
        let hello = req(0x0105, Command::Hello { protocol_version: ProtocolVersion { major: 1, minor: 2 } });
        assert_eq!(postcard::to_slice(&hello, &mut buf).unwrap(), &[0x85, 0x02, 0, 1, 2]);
        assert_eq!(postcard::to_slice(&req(7, Command::SetLaser(Laser::On)), &mut buf).unwrap(), &[7, 1, 1]);
        let status =
            FromDevice::Reply { id: 7, result: Ok(Reply::Status(Status { laser: Laser::Off, link: Link::Session, uptime_ms: 300 })) };
        assert_eq!(postcard::to_slice(&status, &mut buf).unwrap(), &[0, 7, 0, 1, 0, 1, 0xAC, 0x02]);
        let lost = FromDevice::Event(Event::SessionLost);
        assert_eq!(postcard::to_slice(&lost, &mut buf).unwrap(), &[1, 1]);
        let mut frame = [0u8; crate::frame::MAX_FRAME];
        assert_eq!(crate::frame::encode(&lost, &mut frame).unwrap(), &[0, 5, 1, 1, 0x16, 0x07, 0]);
    }

    #[test]
    fn in_a_session_the_host_switches_and_must_keep_talking() {
        let mut d = Device::new(CFG);
        d.handle(req(1, hello()), 0);
        d.handle(req(2, Command::SetLaser(Laser::Off)), 100);
        assert_eq!(d.laser(), Laser::Off);
        d.handle(req(3, Command::SetLaser(Laser::On)), 900);
        // every request moves the deadline
        assert_eq!(d.poll(1800), None);
        d.handle(req(4, Command::Heartbeat), 1800);
        assert_eq!(d.poll(2799), None);
        assert_eq!(d.poll(2800), Some(Event::SessionLost));
        assert_eq!((d.laser(), d.link()), (Laser::Off, Link::Lost));
        assert_eq!(d.poll(5000), None, "reported once");
    }

    #[test]
    fn a_lost_session_keeps_the_laser_off_until_a_new_hello() {
        let mut d = Device::new(CFG);
        d.handle(req(1, hello()), 0);
        d.poll(1000);
        assert_eq!(d.handle(req(2, Command::SetLaser(Laser::On)), 1100), FromDevice::Reply { id: 2, result: Err(Error::NoSession) });
        assert_eq!(d.laser(), Laser::Off);
        d.handle(req(3, hello()), 1200);
        d.handle(req(4, Command::SetLaser(Laser::On)), 1300);
        assert_eq!((d.laser(), d.link()), (Laser::On, Link::Session));
    }
}
