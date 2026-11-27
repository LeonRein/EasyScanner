//! Streams a scan session into its `recording.jsonl`.
//!
//! The file is created with the first frame, so a session that never saw a
//! frame leaves none; the gyro samples of the last [`GYRO_PRE_ROLL_NS`]
//! before it go in first. From then on every frame and gyro batch is
//! written, and a [`Setup`] line whenever the device announces one, so a
//! re-fitted laser plane or a new frame size continues the recording. It
//! stops early only when a write fails or the file reaches its size limit;
//! the session carries on without it.

use std::collections::VecDeque;
use std::fmt;
use std::fs::File;
use std::io::{self, BufWriter, Write};
use std::path::{Path, PathBuf};

use super::{FrameRec, Header, RecordingWriter, Setup};
use crate::imu::GyroSample;

/// The recording's file name in the session directory.
pub const FILE_NAME: &str = "recording.jsonl";

/// Gyro samples kept from before the first frame.
pub const GYRO_PRE_ROLL_NS: i64 = 1_000_000_000;

/// Large enough that a frame's lines leave the processing thread in one write.
const BUFFER_BYTES: usize = 256 << 10;

/// Why a recording stopped before its session did.
#[derive(Debug)]
pub enum StopReason {
    /// The file reached the configured size limit.
    SizeLimit {
        bytes: u64,
    },
    Io(io::Error),
}

impl fmt::Display for StopReason {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            StopReason::SizeLimit { bytes } => write!(f, "size limit of {} MB reached", bytes >> 20),
            StopReason::Io(e) => write!(f, "write failed: {e}"),
        }
    }
}

/// How a recording went.
#[derive(Debug)]
pub struct Outcome {
    pub frames: usize,
    pub bytes: u64,
    pub stopped: Option<StopReason>,
}

impl fmt::Display for Outcome {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mb = self.bytes as f64 / (1u64 << 20) as f64;
        match (&self.stopped, self.frames) {
            (None, 0) => write!(f, "no {FILE_NAME}: no frames"),
            (None, n) => write!(f, "{FILE_NAME}: {n} frames, {mb:.0} MB"),
            (Some(why), n) => write!(f, "{FILE_NAME}: {n} frames, {mb:.0} MB, then stopped: {why}"),
        }
    }
}

/// Counts what passes through, for the size limit.
struct Counting<W> {
    inner: W,
    bytes: u64,
}

impl<W: Write> Write for Counting<W> {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        let n = self.inner.write(buf)?;
        self.bytes += n as u64;
        Ok(n)
    }

    fn flush(&mut self) -> io::Result<()> {
        self.inner.flush()
    }
}

type Writer = RecordingWriter<Counting<BufWriter<File>>>;

enum State {
    /// No frame yet: the latest gyro samples wait for it.
    Waiting {
        gyro: VecDeque<GyroSample>,
    },
    Recording(Writer),
    Stopped,
}

pub struct SessionRecorder {
    path: PathBuf,
    max_bytes: u64,
    state: State,
    /// The device's current setup; written before the next frame when `announce` is set.
    setup: Setup,
    announce: bool,
    frames: usize,
    bytes: u64,
    stopped: Option<StopReason>,
}

impl SessionRecorder {
    /// A recorder for the session in `dir` that stops at `max_bytes`, with
    /// the device's current `setup`.
    pub fn new(dir: &Path, max_bytes: u64, setup: Setup) -> Self {
        SessionRecorder {
            path: dir.join(FILE_NAME),
            max_bytes,
            state: State::Waiting { gyro: VecDeque::new() },
            setup,
            announce: true,
            frames: 0,
            bytes: 0,
            stopped: None,
        }
    }

    pub fn path(&self) -> &Path {
        &self.path
    }

    /// The device (re)loaded its calibration. A setup equal to the last one
    /// is written again all the same: it costs one line, and a calibration
    /// read back from its file may differ from the solved one in the last bits.
    pub fn set_setup(&mut self, setup: Setup) {
        self.setup = setup;
        self.announce = true;
    }

    /// Gyro samples that arrived since the last call, oldest first.
    pub fn push_gyro(&mut self, samples: &[GyroSample]) {
        let result = match &mut self.state {
            State::Waiting { gyro } => {
                gyro.extend(samples);
                if let Some(newest) = gyro.back().map(|s| s.t_ns) {
                    while gyro.front().is_some_and(|s| s.t_ns < newest - GYRO_PRE_ROLL_NS) {
                        gyro.pop_front();
                    }
                }
                Ok(())
            }
            State::Recording(out) => out.gyro(samples),
            State::Stopped => Ok(()),
        };
        self.after_write(result);
    }

    /// One analysis frame; `header` is called once, for the first.
    pub fn frame(&mut self, header: impl FnOnce() -> Header, frame: &FrameRec<'_>) {
        if let State::Waiting { gyro } = &mut self.state {
            let gyro: Vec<GyroSample> = gyro.drain(..).collect();
            match Self::create(&self.path, &header(), &gyro) {
                Ok(out) => self.state = State::Recording(out),
                Err(e) => return self.stop(StopReason::Io(e)),
            }
        }
        let State::Recording(out) = &mut self.state else { return };
        let announce = std::mem::take(&mut self.announce);
        let result = if announce { out.setup(&self.setup) } else { Ok(()) }.and_then(|()| out.frame(frame));
        if result.is_ok() {
            self.frames += 1;
        }
        self.after_write(result);
    }

    fn create(path: &Path, header: &Header, gyro: &[GyroSample]) -> io::Result<Writer> {
        let file = Counting { inner: BufWriter::with_capacity(BUFFER_BYTES, File::create(path)?), bytes: 0 };
        let mut out = RecordingWriter::new(file, header)?;
        out.gyro(gyro)?;
        Ok(out)
    }

    /// Push what is buffered to the file (the app is leaving the screen).
    pub fn flush(&mut self) {
        if let State::Recording(out) = &mut self.state {
            let result = out.flush();
            self.after_write(result);
        }
    }

    /// Close the file.
    pub fn finish(mut self) -> Outcome {
        self.flush();
        Outcome { frames: self.frames, bytes: self.bytes, stopped: self.stopped }
    }

    fn after_write(&mut self, result: io::Result<()>) {
        if let State::Recording(out) = &self.state {
            self.bytes = out.get_ref().bytes;
        }
        match result {
            Err(e) => self.stop(StopReason::Io(e)),
            Ok(()) if self.bytes >= self.max_bytes => self.stop(StopReason::SizeLimit { bytes: self.max_bytes }),
            Ok(()) => {}
        }
    }

    fn stop(&mut self, why: StopReason) {
        log::warn!("{}: recording stopped: {why}", self.path.display());
        if let State::Recording(out) = std::mem::replace(&mut self.state, State::Stopped) {
            // what made it into the buffer still goes to the file
            let _ = out.into_inner();
        }
        self.stopped = Some(why);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::recording::Recording;
    use crate::recording::tests::{frame, header, setup};

    fn gyro(t_ns: i64) -> GyroSample {
        GyroSample { t_ns, rate: [0.1, 0.0, -0.1] }
    }

    /// A fresh directory, removed again when dropped.
    struct TempDir(PathBuf);

    impl TempDir {
        fn new(name: &str) -> Self {
            let dir = std::env::temp_dir().join(format!("easyscanner-recorder-{name}-{}", std::process::id()));
            let _ = std::fs::remove_dir_all(&dir);
            std::fs::create_dir_all(&dir).unwrap();
            TempDir(dir)
        }
    }

    impl Drop for TempDir {
        fn drop(&mut self) {
            let _ = std::fs::remove_dir_all(&self.0);
        }
    }

    const LIMIT: u64 = 1 << 30;

    #[test]
    fn records_the_gyro_pre_roll_then_frames_and_gyro_as_they_come() {
        let dir = TempDir::new("stream");
        let mut r = SessionRecorder::new(&dir.0, LIMIT, setup(120.0));
        // 2 s of gyro in two batches before the first frame: only the last second is kept
        r.push_gyro(&(0..=20).map(|i| gyro(i * 50_000_000)).collect::<Vec<_>>());
        r.push_gyro(&(21..=40).map(|i| gyro(i * 50_000_000)).collect::<Vec<_>>());
        assert!(!r.path().exists(), "nothing is written before the first frame");
        r.frame(header, &frame(2_000_000_000));
        r.push_gyro(&[gyro(2_050_000_000)]);
        r.frame(|| unreachable!("the header is written once"), &frame(2_033_000_000));
        let out = r.finish();
        assert_eq!((out.frames, out.stopped.is_none()), (2, true));

        let rec = Recording::load(&dir.0.join(FILE_NAME)).unwrap();
        assert_eq!(rec.header, header());
        assert_eq!(rec.setups, vec![(0, setup(120.0))]);
        assert_eq!(rec.frames, vec![frame(2_000_000_000), frame(2_033_000_000)]);
        let t: Vec<i64> = rec.gyro.iter().map(|s| s.t_ns).collect();
        assert_eq!((t.first(), t.last(), t.len()), (Some(&1_000_000_000), Some(&2_050_000_000), 22));
    }

    #[test]
    fn a_new_setup_continues_the_recording() {
        let dir = TempDir::new("setup");
        let mut r = SessionRecorder::new(&dir.0, LIMIT, setup(120.0));
        r.frame(header, &frame(1));
        r.set_setup(setup(121.0));
        r.frame(header, &frame(2));
        r.frame(header, &frame(3));
        assert_eq!(r.finish().frames, 3);
        let rec = Recording::load(&dir.0.join(FILE_NAME)).unwrap();
        assert_eq!(rec.setups, vec![(0, setup(120.0)), (1, setup(121.0))]);
    }

    #[test]
    fn the_size_limit_stops_the_recording_and_keeps_what_was_written() {
        let dir = TempDir::new("limit");
        let mut r = SessionRecorder::new(&dir.0, 2000, setup(120.0));
        for i in 0..100 {
            r.frame(header, &frame(i));
        }
        let out = r.finish();
        assert!(matches!(out.stopped, Some(StopReason::SizeLimit { bytes: 2000 })), "{out}");
        assert!(out.frames < 100);
        let rec = Recording::load(&dir.0.join(FILE_NAME)).unwrap();
        assert_eq!(rec.frames.len(), out.frames);
    }

    #[test]
    fn a_failed_write_stops_the_recording() {
        let dir = TempDir::new("io");
        let mut r = SessionRecorder::new(&dir.0.join("gone"), LIMIT, setup(120.0));
        r.frame(header, &frame(1));
        let out = r.finish();
        assert!(matches!(out.stopped, Some(StopReason::Io(_))), "{out}");
        assert_eq!(out.frames, 0);
    }

    #[test]
    fn a_session_without_frames_leaves_no_file() {
        let dir = TempDir::new("empty");
        let mut r = SessionRecorder::new(&dir.0, LIMIT, setup(120.0));
        r.push_gyro(&[gyro(0)]);
        let out = r.finish();
        assert_eq!((out.frames, out.stopped.is_none()), (0, true));
        assert!(!dir.0.join(FILE_NAME).exists());
    }
}
