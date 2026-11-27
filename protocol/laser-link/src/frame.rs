//! Framing over a byte stream: `0x00 ‖ COBS(postcard(message) ‖ CRC-16) ‖ 0x00`.
//!
//! COBS removes every `0x00` from the frame, so `0x00` only ever delimits:
//! a receiver that joins mid-stream or loses a byte resynchronises at the
//! next delimiter. The leading one ends whatever the receiver collected
//! before (boot messages, a frame cut short), so each frame is read even
//! after garbage; the empty frames this makes are skipped. The CRC (CRC-16/IBM-SDLC, little endian) makes sure a
//! damaged frame is dropped rather than acted on.

use crc::{CRC_16_IBM_SDLC, Crc};
use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};

const CRC: Crc<u16> = Crc::<u16>::new(&CRC_16_IBM_SDLC);
const CRC_LEN: usize = 2;

/// The largest encoded message (postcard bytes, before the CRC). Today's
/// messages take at most 20; the rest is room for later ones.
pub const MAX_PAYLOAD: usize = 64;
/// The largest frame on the wire, both delimiters included.
pub const MAX_FRAME: usize = 1 + cobs::max_encoding_length(MAX_PAYLOAD + CRC_LEN) + 1;

/// Why a received frame was not accepted (it travels in
/// [`crate::Event::FrameRejected`]).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum FrameError {
    /// Longer than [`MAX_FRAME`]: its bytes were dropped up to the delimiter.
    TooLong,
    /// Not valid COBS.
    Cobs,
    /// The checksum does not match: damaged in transit.
    Crc,
    /// Intact, but not a message of this protocol version.
    Message,
}

/// Why a message could not be framed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EncodeError {
    /// Its encoding is longer than [`MAX_PAYLOAD`].
    TooLarge,
    /// The output buffer is shorter than the frame; [`MAX_FRAME`] always fits.
    BufferTooSmall,
}

/// Encode `msg` as one frame into `out`; returns the frame's bytes.
pub fn encode<'a, T: Serialize + ?Sized>(msg: &T, out: &'a mut [u8]) -> Result<&'a [u8], EncodeError> {
    let mut raw = [0u8; MAX_PAYLOAD + CRC_LEN];
    let n = postcard::to_slice(msg, &mut raw[..MAX_PAYLOAD]).map_err(|_| EncodeError::TooLarge)?.len();
    let crc = CRC.checksum(&raw[..n]).to_le_bytes();
    raw[n..n + CRC_LEN].copy_from_slice(&crc);
    let (start, body) = out.split_first_mut().ok_or(EncodeError::BufferTooSmall)?;
    *start = 0;
    let len = cobs::try_encode(&raw[..n + CRC_LEN], body).map_err(|_| EncodeError::BufferTooSmall)?;
    *body.get_mut(len).ok_or(EncodeError::BufferTooSmall)? = 0;
    Ok(&out[..len + 2])
}

/// Collects bytes from the stream and yields one payload, or message, per
/// frame.
pub struct FrameDecoder {
    buf: [u8; MAX_FRAME],
    len: usize,
    /// The current frame outgrew the buffer; skip to its delimiter.
    overflowed: bool,
}

impl Default for FrameDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl FrameDecoder {
    pub const fn new() -> Self {
        FrameDecoder { buf: [0; MAX_FRAME], len: 0, overflowed: false }
    }

    /// Feed one received byte. At a delimiter, returns the frame's message
    /// or why it was dropped; empty frames (consecutive delimiters) are
    /// skipped.
    pub fn push<T: DeserializeOwned>(&mut self, byte: u8) -> Option<Result<T, FrameError>> {
        let payload = self.push_payload(byte)?;
        Some(payload.and_then(|p| postcard::from_bytes(p).map_err(|_| FrameError::Message)))
    }

    /// Like [`FrameDecoder::push`], but returns the checked payload
    /// undecoded, for a receiver that reads it piece by piece.
    pub fn push_payload(&mut self, byte: u8) -> Option<Result<&[u8], FrameError>> {
        if byte != 0 {
            match self.buf.get_mut(self.len) {
                Some(slot) if !self.overflowed => {
                    *slot = byte;
                    self.len += 1;
                }
                _ => self.overflowed = true,
            }
            return None;
        }
        let (len, overflowed) = (self.len, self.overflowed);
        self.len = 0;
        self.overflowed = false;
        match (len, overflowed) {
            (_, true) => Some(Err(FrameError::TooLong)),
            (0, false) => None,
            _ => Some(unframe(&mut self.buf[..len])),
        }
    }
}

/// One frame's bytes (without the delimiters) → its payload, decoded in
/// place and checked.
fn unframe(frame: &mut [u8]) -> Result<&[u8], FrameError> {
    let n = cobs::decode_in_place(frame).map_err(|_| FrameError::Cobs)?;
    let (payload, crc) = frame[..n].split_at(n.checked_sub(CRC_LEN).ok_or(FrameError::Crc)?);
    if CRC.checksum(payload).to_le_bytes() != crc {
        return Err(FrameError::Crc);
    }
    Ok(payload)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::message::*;

    fn frame_of<T: Serialize>(msg: &T) -> ([u8; MAX_FRAME], usize) {
        let mut out = [0u8; MAX_FRAME];
        let n = encode(msg, &mut out).unwrap().len();
        (out, n)
    }

    fn feed<T: DeserializeOwned>(dec: &mut FrameDecoder, bytes: &[u8]) -> Option<Result<T, FrameError>> {
        let mut last = None;
        for &b in bytes {
            if let Some(r) = dec.push(b) {
                assert!(last.is_none(), "one frame, one result");
                last = Some(r);
            }
        }
        last
    }

    fn largest_messages() -> [FromDevice; 2] {
        let info = DeviceInfo {
            protocol_version: ProtocolVersion { major: u16::MAX, minor: u16::MAX },
            firmware: FirmwareVersion { major: u8::MAX, minor: u8::MAX, patch: u8::MAX },
            watchdog_ms: u32::MAX,
        };
        let status = Status { laser: Laser::On, link: Link::Lost, uptime_ms: u64::MAX };
        [
            FromDevice::Reply { id: u16::MAX, result: Ok(Reply::Hello(info)) },
            FromDevice::Reply { id: u16::MAX, result: Ok(Reply::Status(status)) },
        ]
    }

    #[test]
    fn every_message_fits_a_frame() {
        for msg in largest_messages() {
            let (_, n) = frame_of(&msg);
            assert!(n <= MAX_FRAME);
        }
        let version = ProtocolVersion { major: u16::MAX, minor: u16::MAX };
        let req = Request { id: u16::MAX, command: Command::Hello { protocol_version: version } };
        assert!(frame_of(&req).1 <= MAX_FRAME);
        let rejected = FromDevice::Event(Event::FrameRejected(FrameError::Message));
        assert!(frame_of(&rejected).1 <= MAX_FRAME);
    }

    #[test]
    fn frames_round_trip_and_hold_no_zero_but_the_delimiters() {
        let req = Request { id: 0, command: Command::SetLaser(Laser::Off) };
        let (out, n) = frame_of(&req);
        assert!(out[1..n - 1].iter().all(|&b| b != 0));
        assert_eq!((out[0], out[n - 1]), (0, 0));
        let mut dec = FrameDecoder::new();
        assert_eq!(feed::<Request>(&mut dec, &out[..n]), Some(Ok(req)));
        for msg in largest_messages() {
            let (out, n) = frame_of(&msg);
            assert_eq!(feed::<FromDevice>(&mut dec, &out[..n]), Some(Ok(msg)));
        }
    }

    #[test]
    fn a_damaged_frame_is_dropped_and_the_next_one_is_read() {
        let req = Request { id: 7, command: Command::GetStatus };
        let (mut out, n) = frame_of(&req);
        let good = out;
        out[1] ^= 0x10;
        let mut dec = FrameDecoder::new();
        assert!(matches!(feed::<Request>(&mut dec, &out[..n]), Some(Err(FrameError::Crc | FrameError::Cobs))));
        assert_eq!(feed::<Request>(&mut dec, &good[..n]), Some(Ok(req)));
    }

    #[test]
    fn joining_mid_frame_resynchronises_at_the_delimiter() {
        let req = Request { id: 300, command: Command::Heartbeat };
        let (out, n) = frame_of(&req);
        let mut dec = FrameDecoder::new();
        // the tail of an earlier frame, then a clean one
        assert!(feed::<Request>(&mut dec, &out[3..n]).is_some_and(|r| r.is_err()));
        assert_eq!(feed::<Request>(&mut dec, &[0, 0]), None, "empty frames are skipped");
        assert_eq!(feed::<Request>(&mut dec, &out[..n]), Some(Ok(req)));
    }

    #[test]
    fn garbage_without_a_delimiter_costs_no_frame() {
        // e.g. a boot message on the same port: the frame's leading delimiter ends it
        let req = Request { id: 9, command: Command::GetStatus };
        let (out, n) = frame_of(&req);
        let mut dec = FrameDecoder::new();
        let mut results = b"boot log".iter().chain(&out[..n]).filter_map(|&b| dec.push::<Request>(b));
        assert!(results.next().is_some_and(|r| r.is_err()));
        assert_eq!(results.next(), Some(Ok(req)));
    }

    #[test]
    fn an_endless_frame_is_dropped_whole() {
        let mut dec = FrameDecoder::new();
        let junk = [0x55u8; 3 * MAX_FRAME];
        assert_eq!(feed::<Request>(&mut dec, &junk), None);
        assert_eq!(dec.push::<Request>(0), Some(Err(FrameError::TooLong)));
        assert_eq!(encode(&[0u8; MAX_PAYLOAD][..], &mut [0u8; 2 * MAX_FRAME]), Err(EncodeError::TooLarge));
        assert_eq!(encode(&Command::GetStatus, &mut [0u8; 3]), Err(EncodeError::BufferTooSmall));
        let req = Request { id: 1, command: Command::GetStatus };
        let (out, n) = frame_of(&req);
        assert_eq!(feed::<Request>(&mut dec, &out[..n]), Some(Ok(req)));
    }

    #[test]
    fn an_intact_frame_of_another_message_is_refused() {
        // valid framing and CRC around bytes that are no Request
        let (out, n) = frame_of(&[0xFFu8, 0xFF, 0xFF, 0xFF, 0xFF]);
        let mut dec = FrameDecoder::new();
        assert_eq!(feed::<Request>(&mut dec, &out[..n]), Some(Err(FrameError::Message)));
    }
}
