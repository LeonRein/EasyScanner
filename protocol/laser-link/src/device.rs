//! The laser switch's behaviour, independent of the hardware: the firmware
//! feeds it requests and the time, and sets the laser pin to [`Device::laser`].
//!
//! - Without a session ([`Link::Standalone`]) the laser is on, so the
//!   scanner works with nothing but the cable.
//! - [`Command::Hello`] with this [`PROTOCOL_VERSION`] opens a session; from
//!   then on only the host switches the laser.
//! - In a session every request resets the watchdog. When it runs out the
//!   laser goes off ([`Link::Lost`], [`Event::SessionLost`]) and stays off
//!   until a new Hello: an app that crashed must not leave a laser shining.

use crate::PROTOCOL_VERSION;
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
}

impl Device {
    pub const fn new(cfg: DeviceConfig) -> Self {
        Device { cfg, laser: Laser::On, link: Link::Standalone, deadline: 0 }
    }

    /// The state the laser pin must have.
    pub fn laser(&self) -> Laser {
        self.laser
    }

    pub fn link(&self) -> Link {
        self.link
    }

    /// Answer one request received at `now`.
    pub fn handle(&mut self, req: Request, now: Millis) -> FromDevice {
        FromDevice::Reply { id: req.id, result: self.execute(req.command, now) }
    }

    fn execute(&mut self, command: Command, now: Millis) -> Result<Reply, Error> {
        if let Command::Hello { protocol_version } = command {
            if protocol_version != PROTOCOL_VERSION {
                return Err(Error::UnsupportedVersion { device: PROTOCOL_VERSION });
            }
            self.link = Link::Session;
            self.feed_watchdog(now);
            return Ok(Reply::Hello(self.info()));
        }
        match (command, self.link) {
            (Command::GetStatus, Link::Session) => self.feed_watchdog(now),
            (Command::GetStatus, _) => {}
            (_, Link::Standalone | Link::Lost) => return Err(Error::NoSession),
            (Command::SetLaser(laser), Link::Session) => {
                self.laser = laser;
                self.feed_watchdog(now);
            }
            (Command::Heartbeat, Link::Session) => self.feed_watchdog(now),
            (Command::Hello { .. }, Link::Session) => unreachable!("handled above"),
        }
        Ok(Reply::Status(self.status(now)))
    }

    /// Check the watchdog; call it regularly. Returns the event to send
    /// when the session just ended.
    pub fn poll(&mut self, now: Millis) -> Option<Event> {
        if self.link == Link::Session && now >= self.deadline {
            self.link = Link::Lost;
            self.laser = Laser::Off;
            return Some(Event::SessionLost);
        }
        None
    }

    pub fn status(&self, now: Millis) -> Status {
        // wraps after 49 days, as documented on the field
        Status { laser: self.laser, link: self.link, uptime_ms: now as u32 }
    }

    pub fn info(&self) -> DeviceInfo {
        DeviceInfo { protocol_version: PROTOCOL_VERSION, firmware: self.cfg.firmware, watchdog_ms: self.cfg.watchdog_ms }
    }

    fn feed_watchdog(&mut self, now: Millis) {
        self.deadline = now + Millis::from(self.cfg.watchdog_ms);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
        let old = d.handle(req(1, Command::Hello { protocol_version: PROTOCOL_VERSION + 1 }), 0);
        assert_eq!(old, FromDevice::Reply { id: 1, result: Err(Error::UnsupportedVersion { device: PROTOCOL_VERSION }) });
        assert_eq!(d.link(), Link::Standalone);
        assert_eq!(d.handle(req(2, hello()), 0), FromDevice::Reply { id: 2, result: Ok(Reply::Hello(d.info())) });
        assert_eq!(d.link(), Link::Session);
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
