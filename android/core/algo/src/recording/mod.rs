//! Raw recordings of scans: what the device saw and decided per frame, so a
//! scan can be replayed offline (`bench::reconstruct`, `scan_bench eval`)
//! and the pipeline tuned on real data. The app streams every scan session
//! into `recording.jsonl` ([`session::SessionRecorder`]); the benchmark's
//! simulator writes the same format.
//!
//! # File format
//!
//! JSON lines, each an object tagged by `kind`:
//!
//! | `kind` | what | when |
//! |---|---|---|
//! | `header` | [`Header`] plus `version` | first line, once |
//! | `setup` | [`Setup`]: camera model and laser plane | before the first frame, and whenever the calibration is (re)loaded |
//! | `frame` | [`FrameRec`] | every analysis frame, in order |
//! | `gyro` | gyroscope samples | every batch, in arrival order |
//!
//! Compatibility: `version` changes only when an existing record changes
//! meaning; a reader refuses other versions. Adding a field (with a
//! default) or a new `kind` keeps the version: readers fill in the default
//! and skip kinds they do not know. A file cut off mid-line (the app was
//! killed) reads up to its last whole line.

pub mod format;
pub mod session;

use std::fs::File;
use std::io::{self, BufRead, BufReader, Write};
use std::path::Path;

use serde::{Deserialize, Serialize};

pub use format::*;

use crate::imu::GyroSample;

/// The version this build writes and reads.
pub const VERSION: u32 = 1;

/// A whole recording in memory.
#[derive(Clone, Debug, PartialEq)]
pub struct Recording {
    pub header: Header,
    /// Each setup with the index of the first frame it applies to, in order.
    pub setups: Vec<(usize, Setup)>,
    pub frames: Vec<FrameRec<'static>>,
    /// Gyroscope samples (gyro clock, IMU frame), oldest first.
    pub gyro: Vec<GyroSample>,
}

impl Recording {
    /// The setup frame `i` was processed with.
    pub fn setup_of(&self, i: usize) -> Option<&Setup> {
        let n = self.setups.partition_point(|(first, _)| *first <= i);
        n.checked_sub(1).map(|k| &self.setups[k].1)
    }

    pub fn load(path: &Path) -> io::Result<Recording> {
        Recording::read(BufReader::new(File::open(path)?))
    }

    /// Read a recording (see the module docs for what is accepted).
    pub fn read(mut r: impl BufRead) -> io::Result<Recording> {
        let invalid = |no: usize, msg: String| io::Error::new(io::ErrorKind::InvalidData, format!("line {no}: {msg}"));
        let mut rec: Option<Recording> = None;
        let mut buf = String::new();
        for no in 1.. {
            buf.clear();
            if r.read_line(&mut buf)? == 0 {
                break;
            }
            let complete = buf.ends_with('\n');
            if buf.trim().is_empty() {
                continue;
            }
            let line: LineIn = match serde_json::from_str(&buf) {
                Ok(l) => l,
                Err(_) if !complete => break,
                Err(e) => return Err(invalid(no, e.to_string())),
            };
            match (&mut rec, line) {
                (None, LineIn::Header { version, header }) => {
                    if version != VERSION {
                        return Err(invalid(no, format!("recording version {version} (this build reads version {VERSION})")));
                    }
                    rec = Some(Recording { header, setups: Vec::new(), frames: Vec::new(), gyro: Vec::new() });
                }
                (None, _) => return Err(invalid(no, "a recording starts with its header".into())),
                (Some(_), LineIn::Header { .. }) => return Err(invalid(no, "a second header".into())),
                (Some(rec), LineIn::Setup(s)) => rec.setups.push((rec.frames.len(), s)),
                (Some(rec), LineIn::Frame(f)) => rec.frames.push(f.into_owned()),
                (Some(rec), LineIn::Gyro { samples }) => rec.gyro.extend(samples),
                (Some(_), LineIn::Unknown) => {}
            }
        }
        rec.ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, "empty recording"))
    }

    /// Write it out: header, gyro, then each setup before its frames.
    pub fn write(&self, out: impl Write) -> io::Result<()> {
        let mut w = RecordingWriter::new(out, &self.header)?;
        w.gyro(&self.gyro)?;
        let mut setups = self.setups.iter().peekable();
        for (i, f) in self.frames.iter().enumerate() {
            while let Some((_, s)) = setups.next_if(|(first, _)| *first <= i) {
                w.setup(s)?;
            }
            w.frame(f)?;
        }
        w.into_inner().map(drop)
    }
}

/// One line, as written.
#[derive(Serialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
enum LineOut<'a> {
    Header {
        version: u32,
        #[serde(flatten)]
        header: &'a Header,
    },
    Setup(&'a Setup),
    Frame(&'a FrameRec<'a>),
    Gyro {
        samples: &'a [GyroSample],
    },
}

/// One line, as read.
#[derive(Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
enum LineIn {
    Header {
        version: u32,
        #[serde(flatten)]
        header: Header,
    },
    Setup(Setup),
    Frame(FrameRec<'static>),
    Gyro {
        samples: Vec<GyroSample>,
    },
    /// A kind a later version added.
    #[serde(other)]
    Unknown,
}

/// Writes a recording as it happens. Each record is one line, so a reader
/// can follow the file and a cut-off file loses at most its last line.
pub struct RecordingWriter<W: Write> {
    out: W,
}

impl<W: Write> RecordingWriter<W> {
    /// Start a recording: writes the header with [`VERSION`].
    pub fn new(out: W, header: &Header) -> io::Result<Self> {
        let mut w = RecordingWriter { out };
        w.line(&LineOut::Header { version: VERSION, header })?;
        Ok(w)
    }

    pub fn setup(&mut self, setup: &Setup) -> io::Result<()> {
        self.line(&LineOut::Setup(setup))
    }

    pub fn frame(&mut self, frame: &FrameRec<'_>) -> io::Result<()> {
        self.line(&LineOut::Frame(frame))
    }

    /// Gyro samples that arrived since the last batch; an empty batch writes nothing.
    pub fn gyro(&mut self, samples: &[GyroSample]) -> io::Result<()> {
        if samples.is_empty() {
            return Ok(());
        }
        self.line(&LineOut::Gyro { samples })
    }

    pub fn flush(&mut self) -> io::Result<()> {
        self.out.flush()
    }

    /// The output it writes to.
    pub fn get_ref(&self) -> &W {
        &self.out
    }

    pub fn into_inner(mut self) -> io::Result<W> {
        self.out.flush()?;
        Ok(self.out)
    }

    fn line(&mut self, line: &LineOut) -> io::Result<()> {
        serde_json::to_writer(&mut self.out, line).map_err(io::Error::other)?;
        self.out.write_all(b"\n")
    }
}

#[cfg(test)]
mod tests {
    use std::borrow::Cow;

    use super::*;

    pub(crate) fn header() -> Header {
        Header {
            source: "test".into(),
            board: BoardDims::default(),
            volume: VolumeDims::default(),
            cam_from_imu: None,
            fusion: Default::default(),
            scan_gate: Default::default(),
        }
    }

    pub(crate) fn setup(d: f64) -> Setup {
        Setup { intrinsics: CameraModel::default(), laser_plane: Some(PlaneRec { n: [0.0, -0.6, 0.8], d }), board_surface: None }
    }

    pub(crate) fn frame(ts_ns: i64) -> FrameRec<'static> {
        FrameRec {
            ts_ns,
            readout_ns: 20_000_000,
            exposure_ns: 5_000_000,
            width: 1280,
            height: 960,
            pose: Some(PoseObs {
                r: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
                t: [1.0, 2.0, 300.0],
                reproj_rms: 0.4,
                margin: None,
            }),
            scan: Some(ScanOutcome::Rejected(crate::gate::Reject::Moving)),
            corners: Cow::Owned(vec![CornerRec { id: 3, u: 100.5, v: 200.25 }]),
            laser_px: Cow::Owned(vec![[10.0, 20.0], [11.0, 21.0]]),
        }
    }

    fn recording() -> Recording {
        Recording {
            header: header(),
            setups: vec![(0, setup(120.0)), (2, setup(121.0))],
            frames: vec![frame(1), frame(2), frame(3)],
            gyro: vec![GyroSample { t_ns: 5, rate: [0.1, 0.2, 0.3] }],
        }
    }

    fn bytes(rec: &Recording) -> Vec<u8> {
        let mut out = Vec::new();
        rec.write(&mut out).unwrap();
        out
    }

    #[test]
    fn round_trips_with_setups_in_place() {
        let rec = recording();
        let back = Recording::read(bytes(&rec).as_slice()).unwrap();
        assert_eq!(back, rec);
        assert_eq!(back.setup_of(1).unwrap().laser_plane.unwrap().d, 120.0);
        assert_eq!(back.setup_of(2).unwrap().laser_plane.unwrap().d, 121.0);
    }

    #[test]
    fn a_cut_off_last_line_is_left_out_but_a_broken_middle_line_is_an_error() {
        let rec = recording();
        let mut file = bytes(&rec);
        file.truncate(file.len() - 10);
        assert_eq!(Recording::read(file.as_slice()).unwrap().frames.len(), 2);
        let mut broken = bytes(&rec);
        let at = broken.iter().position(|&b| b == b'\n').unwrap() + 3;
        broken[at] = b'#';
        assert!(Recording::read(broken.as_slice()).is_err());
    }

    #[test]
    fn another_version_is_refused_and_unknown_kinds_are_skipped() {
        let mut file = String::from_utf8(bytes(&recording())).unwrap();
        let later = file.replacen(r#""version":1"#, r#""version":2"#, 1);
        let err = Recording::read(later.as_bytes()).unwrap_err();
        assert!(err.to_string().contains("version 2"), "{err}");
        file.push_str("{\"kind\":\"annotation\",\"text\":\"from a later version\"}\n");
        assert_eq!(Recording::read(file.as_bytes()).unwrap().frames.len(), 3);
    }

    #[test]
    fn a_file_must_start_with_its_header() {
        let file = String::from_utf8(bytes(&recording())).unwrap();
        let headless: String = file.lines().skip(1).map(|l| format!("{l}\n")).collect();
        assert!(Recording::read(headless.as_bytes()).is_err());
    }
}
