//! Streams a scan session into a [`Recording`] (`recording.jsonl` in the
//! session directory), so that a real scan can be replayed offline through
//! the benchmark's reconstruction (`scan_bench eval`) while the pipeline is
//! tuned.
//!
//! The recording starts with the first scanning frame, because only then are
//! the frame size, the camera model and the laser plane it is triangulated
//! with known; the gyro samples of the last [`GYRO_PRE_ROLL_NS`] before it
//! go in first, so its rolling-shutter correction can be replayed too. From
//! then on every gyro batch is written, and every frame the engine scans. A
//! recording covers one set-up: when the camera model, the laser plane or the
//! board changes under it, or a write fails, it ends and the session carries
//! on without it.

use std::collections::VecDeque;
use std::fs::File;
use std::io::{self, BufWriter};
use std::path::{Path, PathBuf};

use crate::bench::recording::{FrameRec, Header, RecordingWriter};
use crate::imu::GyroSample;

/// The recording's file name in the session directory.
pub const FILE_NAME: &str = "recording.jsonl";

/// Gyro samples kept from before the first scanning frame.
pub const GYRO_PRE_ROLL_NS: i64 = 1_000_000_000;

/// Why a recording stopped before its session did.
#[derive(Debug)]
pub enum Ended {
    /// The camera model, laser plane, board or frame size changed.
    SetupChanged,
    Io(io::Error),
}

impl std::fmt::Display for Ended {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Ended::SetupChanged => f.write_str("the calibration or frame size changed"),
            Ended::Io(e) => write!(f, "write failed: {e}"),
        }
    }
}

enum State {
    /// No scanning frame yet: the latest gyro samples wait for it.
    Waiting {
        gyro: VecDeque<GyroSample>,
    },
    /// The header is boxed: one recorder per session, and the enum stays small.
    Recording {
        out: RecordingWriter<BufWriter<File>>,
        header: Box<Header>,
    },
    Ended(Ended),
}

pub struct SessionRecorder {
    path: PathBuf,
    state: State,
    frames: usize,
}

impl SessionRecorder {
    /// A recorder for the session in `dir`; the file is created with the
    /// first scanning frame.
    pub fn new(dir: &Path) -> Self {
        SessionRecorder { path: dir.join(FILE_NAME), state: State::Waiting { gyro: VecDeque::new() }, frames: 0 }
    }

    pub fn path(&self) -> &Path {
        &self.path
    }

    /// Frames written so far.
    pub fn frames(&self) -> usize {
        self.frames
    }

    /// Gyro samples that arrived since the last call, oldest first.
    pub fn push_gyro(&mut self, samples: &[GyroSample]) {
        match &mut self.state {
            State::Waiting { gyro } => {
                gyro.extend(samples);
                if let Some(newest) = gyro.back().map(|s| s.t_ns) {
                    while gyro.front().is_some_and(|s| s.t_ns < newest - GYRO_PRE_ROLL_NS) {
                        gyro.pop_front();
                    }
                }
            }
            State::Recording { out, .. } => {
                if let Err(e) = out.gyro(samples) {
                    self.state = State::Ended(Ended::Io(e));
                }
            }
            State::Ended(_) => {}
        }
    }

    /// A frame the engine scanned, with the set-up it was scanned in.
    pub fn frame(&mut self, header: &Header, frame: &FrameRec) {
        let result = match &mut self.state {
            State::Waiting { gyro } => {
                let gyro: Vec<GyroSample> = gyro.drain(..).collect();
                Self::start(&self.path, header, &gyro, frame).map(|out| {
                    self.state = State::Recording { out, header: Box::new(header.clone()) };
                })
            }
            State::Recording { header: started, .. } if **started != *header => {
                self.state = State::Ended(Ended::SetupChanged);
                return;
            }
            State::Recording { out, .. } => out.frame(frame),
            State::Ended(_) => return,
        };
        match result {
            Ok(()) => self.frames += 1,
            Err(e) => self.state = State::Ended(Ended::Io(e)),
        }
    }

    fn start(path: &Path, header: &Header, gyro: &[GyroSample], frame: &FrameRec) -> io::Result<RecordingWriter<BufWriter<File>>> {
        let mut out = RecordingWriter::create(path, header)?;
        out.gyro(gyro)?;
        out.frame(frame)?;
        Ok(out)
    }

    /// Push what is buffered to the file (the app is leaving the screen).
    pub fn flush(&mut self) {
        if let State::Recording { out, .. } = &mut self.state
            && let Err(e) = out.flush()
        {
            self.state = State::Ended(Ended::Io(e));
        }
    }

    /// Close the file; says how the recording went, for the status line.
    pub fn finish(mut self) -> String {
        self.flush();
        match self.state {
            State::Waiting { .. } => format!("no {FILE_NAME}: nothing was scanned"),
            State::Recording { .. } => format!("{FILE_NAME} {} frames", self.frames),
            State::Ended(why) => format!("{FILE_NAME} {} frames, then stopped: {why}", self.frames),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bench::recording::{CornerRec, PlaneRec, Recording, VolumeDims};
    use crate::bench::scene::BoardDims;
    use crate::bench::sim::CameraModel;

    fn header() -> Header {
        Header {
            version: Recording::VERSION,
            source: "test".into(),
            board: BoardDims::default(),
            volume: VolumeDims::default(),
            intrinsics: CameraModel::default(),
            laser_plane: PlaneRec { n: [0.0, -0.6, 0.8], d: 120.0 },
            cam_from_imu: None,
        }
    }

    fn frame(ts_ns: i64) -> FrameRec {
        FrameRec {
            ts_ns,
            readout_ns: 20_000_000,
            exposure_ns: 5_000_000,
            pose: None,
            corners: vec![CornerRec { id: 3, u: 100.5, v: 200.25 }],
            laser_px: vec![[10.0, 20.0], [11.0, 21.0]],
        }
    }

    fn gyro(t_ns: i64) -> GyroSample {
        GyroSample { t_ns, rate: [0.1, 0.0, -0.1] }
    }

    fn temp_dir(name: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!("easyscanner-recorder-{name}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }

    #[test]
    fn records_the_gyro_pre_roll_then_frames_and_gyro_as_they_come() {
        let dir = temp_dir("stream");
        let mut r = SessionRecorder::new(&dir);
        // 2 s of gyro before the scan starts: only the last second is kept
        let early: Vec<GyroSample> = (0..=40).map(|i| gyro(i * 50_000_000)).collect();
        r.push_gyro(&early);
        assert!(!r.path().exists(), "nothing is written before the first scanning frame");
        r.frame(&header(), &frame(2_000_000_000));
        r.push_gyro(&[gyro(2_050_000_000)]);
        r.frame(&header(), &frame(2_033_000_000));
        assert_eq!(r.finish(), "recording.jsonl 2 frames");

        let rec = Recording::load(&dir.join(FILE_NAME)).unwrap();
        assert_eq!(rec.header, header());
        assert_eq!(rec.frames, vec![frame(2_000_000_000), frame(2_033_000_000)]);
        let t: Vec<i64> = rec.gyro.iter().map(|s| s.t_ns).collect();
        assert_eq!(t.first(), Some(&1_000_000_000));
        assert_eq!(t.last(), Some(&2_050_000_000));
        assert_eq!(t.len(), 22);
    }

    #[test]
    fn a_changed_setup_ends_the_recording() {
        let dir = temp_dir("setup");
        let mut r = SessionRecorder::new(&dir);
        r.frame(&header(), &frame(1));
        let refitted = Header { laser_plane: PlaneRec { n: [0.0, -0.6, 0.8], d: 121.0 }, ..header() };
        r.frame(&refitted, &frame(2));
        r.frame(&header(), &frame(3));
        assert_eq!(r.finish(), "recording.jsonl 1 frames, then stopped: the calibration or frame size changed");
        assert_eq!(Recording::load(&dir.join(FILE_NAME)).unwrap().frames.len(), 1);
    }

    #[test]
    fn an_unscanned_session_leaves_no_file() {
        let dir = temp_dir("empty");
        let mut r = SessionRecorder::new(&dir);
        r.push_gyro(&[gyro(0)]);
        assert_eq!(r.finish(), "no recording.jsonl: nothing was scanned");
        assert!(!dir.join(FILE_NAME).exists());
    }
}
