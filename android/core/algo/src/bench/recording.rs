//! The recording format the reconstruction runs on: per frame the board
//! corners and/or the pose, and the laser-stripe pixels, plus the camera and
//! laser-plane calibration the pipeline believes in. The simulator writes it
//! (with the ground truth attached); a real phone session can be written in
//! the same shape later and then runs through exactly the same
//! reconstruction and metrics.

use std::io;
use std::path::Path;

use serde::{Deserialize, Serialize};

use crate::geom::{M3, Pose, V3};
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
    /// Pose at each frame's timestamp (first row).
    pub poses: Vec<PoseRec>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Recording {
    pub version: u32,
    /// Free text: "synthetic: <scenario>", phone model, …
    pub source: String,
    pub board: BoardDims,
    #[serde(default)]
    pub volume: VolumeDims,
    /// The camera calibration the pipeline uses.
    pub intrinsics: CameraModel,
    /// The laser plane the pipeline uses.
    pub laser_plane: PlaneRec,
    pub frames: Vec<FrameRec>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub truth: Option<Truth>,
}

impl Recording {
    pub const VERSION: u32 = 1;

    pub fn load(path: &Path) -> io::Result<Recording> {
        let f = std::fs::File::open(path)?;
        serde_json::from_reader(io::BufReader::new(f)).map_err(io::Error::other)
    }

    pub fn save(&self, path: &Path) -> io::Result<()> {
        let f = std::fs::File::create(path)?;
        serde_json::to_writer(io::BufWriter::new(f), self).map_err(io::Error::other)
    }
}
