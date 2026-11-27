//! What really happened in a synthetic recording. It is kept out of the
//! recording format (which the app writes too) and saved next to the
//! recording as `<recording>.truth.json`.

use std::fs::File;
use std::io::{self, BufReader, BufWriter};
use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};

use crate::geom::{M3, Pose, V3};
use crate::imu::GyroCalib;
use crate::recording::{CameraModel, PlaneRec, Recording};

use super::scene::Scene;

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

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Truth {
    pub scene: Scene,
    pub intrinsics: CameraModel,
    pub laser_plane: PlaneRec,
    /// Pose at each frame's reference instant: the middle row
    /// ([`crate::rolling::RowClock::mid_ns`]), the instant a corrected pose describes.
    pub poses: Vec<PoseRec>,
    /// The gyro's true calibration, when one was simulated.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub gyro: Option<GyroCalib>,
}

/// A simulated recording with its truth.
#[derive(Clone, Debug, PartialEq)]
pub struct Synthetic {
    pub recording: Recording,
    pub truth: Truth,
}

/// Where the truth of the recording at `path` is kept.
pub fn truth_path(path: &Path) -> PathBuf {
    let mut name = path.file_name().unwrap_or_default().to_os_string();
    name.push(".truth.json");
    path.with_file_name(name)
}

impl Truth {
    /// The truth saved next to the recording at `path`, if there is one.
    pub fn load_for(path: &Path) -> io::Result<Option<Truth>> {
        match File::open(truth_path(path)) {
            Ok(f) => serde_json::from_reader(BufReader::new(f)).map(Some).map_err(io::Error::other),
            Err(e) if e.kind() == io::ErrorKind::NotFound => Ok(None),
            Err(e) => Err(e),
        }
    }
}

impl Synthetic {
    /// The recording at `path`, its truth next to it.
    pub fn save(&self, path: &Path) -> io::Result<()> {
        self.recording.write(BufWriter::new(File::create(path)?))?;
        serde_json::to_writer(BufWriter::new(File::create(truth_path(path))?), &self.truth).map_err(io::Error::other)
    }
}
