//! The recording's records: what one line of `recording.jsonl` holds.

use std::borrow::Cow;

use serde::{Deserialize, Serialize};

use crate::board_surface::BoardSurface;
use crate::config::{BoardSpec, VolumeCfg};
use crate::fusion::FusionCfg;
use crate::gate::{Reject, ScanGateCfg};
use crate::geom::{Intrinsics, M3, Pose, V3};
use crate::rolling::RowClock;
use crate::volume::Volume;

/// What stays the same for the whole recording: the first line.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Header {
    /// Free text: "synthetic: <scenario>", app and camera, …
    pub source: String,
    pub board: BoardDims,
    #[serde(default)]
    pub volume: VolumeDims,
    /// The IMU→camera rotation the platform defines (from Camera2's
    /// `SENSOR_ORIENTATION`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub cam_from_imu: Option<M3>,
    /// The fusion settings the recording device ran with.
    #[serde(default)]
    pub fusion: FusionCfg,
    /// The scan gate the recording device ran with.
    #[serde(default)]
    pub scan_gate: ScanGateCfg,
}

/// The calibration the frames after it were processed with. A new one is
/// written whenever the device (re)loads its calibration.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Setup {
    /// The camera calibration, at its own size; each frame scales it to its
    /// size ([`Intrinsics::for_frame`]).
    pub intrinsics: CameraModel,
    /// The laser plane; `None` while the device had no usable one.
    pub laser_plane: Option<PlaneRec>,
    /// The board's measured shape; `None` while it was taken as flat.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub board_surface: Option<BoardSurface>,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct PlaneRec {
    /// Unit normal, camera frame.
    pub n: V3,
    pub d: f64,
}

/// A board pose the device computed, with what its trust was based on.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct PoseObs {
    /// Board→camera rotation (row-major) and translation (mm), at the
    /// frame's middle row (rolling-shutter corrected when the device could).
    pub r: M3,
    pub t: V3,
    pub reproj_rms: f64,
    /// Runner-up IPPE branch error / the chosen one's; `None` for a pose
    /// without a second branch.
    pub margin: Option<f64>,
}

impl PoseObs {
    pub fn pose(&self) -> Pose {
        Pose { r: self.r, t: self.t }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct CornerRec {
    pub id: u32,
    pub u: f64,
    pub v: f64,
}

/// What the device's scan did with a frame.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ScanOutcome {
    /// Not scanning (paused, or calibrating the laser plane).
    Paused,
    /// Scanning, but no board pose.
    NoPose,
    /// The scan gate refused the frame.
    Rejected(Reject),
    /// The frame's points inside the working volume went into the cloud.
    Kept,
}

/// One analysis frame. Borrows its pixel lists when written, owns them when read.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct FrameRec<'a> {
    /// Sensor timestamp: start of exposure of the first row.
    pub ts_ns: i64,
    /// Rolling-shutter readout, first to last row (ns); 0 = global shutter.
    #[serde(default)]
    pub readout_ns: i64,
    /// Exposure time (ns).
    #[serde(default)]
    pub exposure_ns: i64,
    /// Image size (px).
    pub width: u32,
    pub height: u32,
    /// The pose the device computed, if any.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub pose: Option<PoseObs>,
    /// What the device's scan did with it; `None` for synthetic frames.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub scan: Option<ScanOutcome>,
    /// Detected ChArUco corners, full-frame pixels.
    #[serde(default)]
    pub corners: Cow<'a, [CornerRec]>,
    /// Detected stripe centre-line pixels.
    #[serde(default)]
    pub laser_px: Cow<'a, [[f32; 2]]>,
}

impl FrameRec<'_> {
    /// When each row of this frame was exposed.
    pub fn clock(&self) -> RowClock {
        RowClock { start_ns: self.ts_ns, readout_ns: self.readout_ns, exposure_ns: self.exposure_ns, height: self.height as f64 }
    }

    /// `setup`'s camera model at this frame's size.
    pub fn intrinsics(&self, setup: &Setup) -> Intrinsics {
        Intrinsics::from(setup.intrinsics).for_frame(self.width, self.height)
    }

    pub fn into_owned(self) -> FrameRec<'static> {
        FrameRec { corners: Cow::Owned(self.corners.into_owned()), laser_px: Cow::Owned(self.laser_px.into_owned()), ..self }
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

/// The printed ChArUco board's geometry (OpenCV layout: square (0,0) black,
/// markers in the white squares).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct BoardDims {
    pub squares_x: u32,
    pub squares_y: u32,
    pub square_mm: f64,
    pub marker_mm: f64,
}

impl Default for BoardDims {
    /// The app's default board (`BoardSpec::default`): 8x8, 36 mm.
    fn default() -> Self {
        BoardDims { squares_x: 8, squares_y: 8, square_mm: 36.0, marker_mm: 26.64 }
    }
}

impl From<BoardSpec> for BoardDims {
    fn from(b: BoardSpec) -> Self {
        BoardDims {
            squares_x: b.squares_x as u32,
            squares_y: b.squares_y as u32,
            square_mm: b.square_mm.into(),
            marker_mm: b.marker_mm.into(),
        }
    }
}

impl BoardDims {
    pub fn width_mm(&self) -> f64 {
        self.squares_x as f64 * self.square_mm
    }

    pub fn height_mm(&self) -> f64 {
        self.squares_y as f64 * self.square_mm
    }

    /// The inner ChArUco corners with OpenCV's ids (row-major, x fastest).
    pub fn corners(&self) -> impl Iterator<Item = (u32, V3)> + '_ {
        let nx = self.squares_x - 1;
        (0..nx * (self.squares_y - 1)).map(move |id| {
            let (i, j) = (id % nx + 1, id / nx + 1);
            (id, [i as f64 * self.square_mm, j as f64 * self.square_mm, 0.0])
        })
    }
}

/// Serializable pinhole + Brown–Conrady model (same fields as [`Intrinsics`]).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct CameraModel {
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
    #[serde(default)]
    pub dist: [f64; 5],
    pub width: f64,
    pub height: f64,
}

impl Default for CameraModel {
    /// A 4:3 phone main camera streamed at 1280x960 (~70° horizontal FOV).
    fn default() -> Self {
        CameraModel { fx: 920.0, fy: 920.0, cx: 639.5, cy: 479.5, dist: [0.05, -0.1, 0.0, 0.0, 0.0], width: 1280.0, height: 960.0 }
    }
}

impl From<CameraModel> for Intrinsics {
    fn from(c: CameraModel) -> Self {
        Intrinsics { fx: c.fx, fy: c.fy, cx: c.cx, cy: c.cy, dist: c.dist, width: c.width, height: c.height }
    }
}

impl From<Intrinsics> for CameraModel {
    fn from(k: Intrinsics) -> Self {
        CameraModel { fx: k.fx, fy: k.fy, cx: k.cx, cy: k.cy, dist: k.dist, width: k.width, height: k.height }
    }
}
