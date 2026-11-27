//! The recording format the reconstruction runs on: per frame the board
//! corners and/or the pose, and the laser-stripe pixels, plus the camera and
//! laser-plane calibration the pipeline believes in. The simulator writes it
//! with the ground truth attached; the app streams every scan session into
//! one (`recording.jsonl`, `easyscanner-core`'s `recorder`), which then runs
//! through exactly the same reconstruction and metrics.

use std::fs::File;
use std::io::{self, BufRead, BufReader, BufWriter, Write};
use std::path::Path;

use serde::{Deserialize, Serialize};

use crate::config::VolumeCfg;
use crate::geom::{M3, Pose, V3};
use crate::imu::{GyroCalib, GyroSample};
use crate::rolling::RowClock;
use crate::volume::Volume;

use super::scene::{BoardDims, Scene};
use super::sim::CameraModel;

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct PlaneRec {
    /// Unit normal, camera frame.
    pub n: V3,
    pub d: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct PoseRec {
    /// Board→camera rotation (row-major) and translation (mm).
    pub r: M3,
    pub t: V3,
}

impl From<Pose> for PoseRec {
    fn from(p: Pose) -> Self {
        PoseRec { r: p.r, t: p.t }
    }
}

impl From<PoseRec> for Pose {
    fn from(p: PoseRec) -> Self {
        Pose { r: p.r, t: p.t }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct CornerRec {
    pub id: u32,
    pub u: f64,
    pub v: f64,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct FrameRec {
    /// Sensor timestamp: start of exposure of the first row.
    pub ts_ns: i64,
    /// Rolling-shutter readout, first to last row (ns); 0 = global shutter.
    #[serde(default)]
    pub readout_ns: i64,
    /// Exposure time (ns).
    #[serde(default)]
    pub exposure_ns: i64,
    /// The pose the device computed, if recorded.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub pose: Option<PoseRec>,
    /// Detected ChArUco corners, full-frame pixels.
    #[serde(default)]
    pub corners: Vec<CornerRec>,
    /// Detected stripe centre-line pixels.
    #[serde(default)]
    pub laser_px: Vec<[f32; 2]>,
}

impl FrameRec {
    /// When each row of this frame was exposed, for an image `height` rows tall.
    pub fn clock(&self, height: f64) -> RowClock {
        RowClock { start_ns: self.ts_ns, readout_ns: self.readout_ns, exposure_ns: self.exposure_ns, height }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct VolumeDims {
    pub margin_mm: f64,
    /// 0 = the board's longer side (as the app does).
    pub height_mm: f64,
}

impl Default for VolumeDims {
    fn default() -> Self {
        VolumeDims { margin_mm: 20.0, height_mm: 0.0 }
    }
}

impl From<VolumeCfg> for VolumeDims {
    fn from(v: VolumeCfg) -> Self {
        VolumeDims { margin_mm: v.margin_mm, height_mm: v.height_mm }
    }
}

impl VolumeDims {
    pub fn volume(&self, b: &BoardDims) -> Volume {
        let height = if self.height_mm > 0.0 { self.height_mm } else { b.width_mm().max(b.height_mm()) };
        Volume { board_w: b.width_mm(), board_h: b.height_mm(), margin: self.margin_mm, height }
    }
}

/// What really happened (synthetic recordings only).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Truth {
    pub scene: Scene,
    pub intrinsics: CameraModel,
    pub laser_plane: PlaneRec,
    /// Pose at each frame's reference instant: the middle row
    /// ([`RowClock::mid_ns`]), the instant a corrected pose describes.
    pub poses: Vec<PoseRec>,
    /// The gyro's true calibration, when one was simulated.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub gyro: Option<GyroCalib>,
}

/// Everything about a recording except its samples: the first line of the
/// file.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Header {
    pub version: u32,
    /// Free text: "synthetic: <scenario>", phone model, …
    pub source: String,
    pub board: BoardDims,
    #[serde(default)]
    pub volume: VolumeDims,
    /// The camera calibration the pipeline uses, at the frames' size.
    pub intrinsics: CameraModel,
    /// The laser plane the pipeline uses.
    pub laser_plane: PlaneRec,
    /// The IMU→camera rotation the platform defines (from Camera2's
    /// `SENSOR_ORIENTATION`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub cam_from_imu: Option<M3>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Recording {
    pub header: Header,
    pub frames: Vec<FrameRec>,
    /// Gyroscope samples (gyro clock, IMU frame), oldest first.
    pub gyro: Vec<GyroSample>,
    pub truth: Option<Truth>,
}

/// One line of a recording file, as written.
#[derive(Serialize)]
#[serde(rename_all = "snake_case")]
enum LineOut<'a> {
    Header(&'a Header),
    Frame(&'a FrameRec),
    Gyro(&'a [GyroSample]),
    Truth(&'a Truth),
}

/// One line of a recording file, as read.
#[derive(Deserialize)]
#[serde(rename_all = "snake_case")]
enum LineIn {
    Header(Header),
    Frame(FrameRec),
    Gyro(Vec<GyroSample>),
    Truth(Truth),
}

/// Writes a recording as it happens: JSON lines, the [`Header`] first, then
/// frames and gyro batches in the order they arrived. A file cut off
/// mid-line (the process was killed) still reads up to its last whole line.
pub struct RecordingWriter<W: Write> {
    out: W,
}

impl RecordingWriter<BufWriter<File>> {
    /// Create `path` and write the header.
    pub fn create(path: &Path, header: &Header) -> io::Result<Self> {
        RecordingWriter::new(BufWriter::new(File::create(path)?), header)
    }
}

impl<W: Write> RecordingWriter<W> {
    pub fn new(out: W, header: &Header) -> io::Result<Self> {
        let mut w = RecordingWriter { out };
        w.line(&LineOut::Header(header))?;
        Ok(w)
    }

    pub fn frame(&mut self, frame: &FrameRec) -> io::Result<()> {
        self.line(&LineOut::Frame(frame))
    }

    /// Gyro samples that arrived since the last batch; an empty batch writes nothing.
    pub fn gyro(&mut self, samples: &[GyroSample]) -> io::Result<()> {
        if samples.is_empty() {
            return Ok(());
        }
        self.line(&LineOut::Gyro(samples))
    }

    pub fn truth(&mut self, truth: &Truth) -> io::Result<()> {
        self.line(&LineOut::Truth(truth))
    }

    pub fn flush(&mut self) -> io::Result<()> {
        self.out.flush()
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

impl Recording {
    /// 3: JSON lines (header, frames, gyro batches, truth) instead of one
    /// document, so a phone can stream a session to disk.
    pub const VERSION: u32 = 3;

    /// Read a recording; one of another version is refused (its frame
    /// timestamps and truth poses mean other instants). A last line without
    /// its line break was cut off while being written and is left out.
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
                (None, LineIn::Header(header)) => {
                    if header.version != Self::VERSION {
                        return Err(invalid(
                            no,
                            format!("recording version {} (this build reads version {})", header.version, Self::VERSION),
                        ));
                    }
                    rec = Some(Recording { header, frames: Vec::new(), gyro: Vec::new(), truth: None });
                }
                (None, _) => return Err(invalid(no, "a recording starts with its header".into())),
                (Some(_), LineIn::Header(_)) => return Err(invalid(no, "a second header".into())),
                (Some(rec), LineIn::Frame(f)) => rec.frames.push(f),
                (Some(rec), LineIn::Gyro(g)) => rec.gyro.extend(g),
                (Some(rec), LineIn::Truth(t)) => rec.truth = Some(t),
            }
        }
        rec.ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, "empty recording"))
    }

    pub fn write(&self, out: impl Write) -> io::Result<()> {
        let mut w = RecordingWriter::new(out, &self.header)?;
        w.gyro(&self.gyro)?;
        for f in &self.frames {
            w.frame(f)?;
        }
        if let Some(t) = &self.truth {
            w.truth(t)?;
        }
        w.into_inner().map(drop)
    }

    pub fn load(path: &Path) -> io::Result<Recording> {
        Recording::read(BufReader::new(File::open(path)?))
    }

    pub fn save(&self, path: &Path) -> io::Result<()> {
        self.write(BufWriter::new(File::create(path)?))
    }
}
