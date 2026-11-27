//! Recording → point cloud, with the same steps the app's engine runs per
//! frame (`easyscanner-core/src/engine.rs`, "classify by 3-D volume
//! membership; accumulate the scan"): stripe pixel → camera ray ∩ laser
//! plane → board frame → keep if inside the working volume.

use serde::{Deserialize, Serialize};

use crate::geom::{Intrinsics, Pose, V3};
use crate::stripe;
use crate::volume::Volume;

use super::pnp;
use super::recording::{FrameRec, Recording};

/// Where a frame's pose comes from.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PoseFrom {
    /// The recorded pose when there is one, else solve from the corners.
    #[default]
    Auto,
    Recorded,
    Corners,
}

/// The app rejects a board with fewer corners (`board.rs`).
pub const MIN_CORNERS: usize = 4;

#[derive(Clone, Copy, Debug)]
pub struct FramePose {
    pub pose: Pose,
    /// NaN for a recorded pose.
    pub reproj_rms: f64,
    pub corners: usize,
}

#[derive(Clone, Debug, Default)]
pub struct Reconstruction {
    /// Board frame (OpenCV, −Z up), mm.
    pub points: Vec<V3>,
    /// Frame index of each point.
    pub point_frame: Vec<u32>,
    pub poses: Vec<Option<FramePose>>,
}

pub fn frame_pose(f: &FrameRec, rec: &Recording, k: &Intrinsics, from: PoseFrom) -> Option<FramePose> {
    let recorded = f.pose.map(|p| FramePose { pose: p.into(), reproj_rms: f64::NAN, corners: f.corners.len() });
    let solve = || {
        if f.corners.len() < MIN_CORNERS {
            return None;
        }
        let ids: Vec<(u32, V3)> = rec.board.corners().collect();
        let (obj, px): (Vec<V3>, Vec<[f64; 2]>) =
            f.corners.iter().filter_map(|c| ids.get(c.id as usize).map(|&(_, p)| (p, [c.u, c.v]))).unzip();
        pnp::solve_planar(k, &obj, &px).map(|(pose, rms)| FramePose { pose, reproj_rms: rms, corners: obj.len() })
    };
    match from {
        PoseFrom::Recorded => recorded,
        PoseFrom::Corners => solve(),
        PoseFrom::Auto => recorded.or_else(solve),
    }
}

/// Triangulate one frame's stripe into board-frame points inside the volume.
pub fn triangulate(px: &[[f32; 2]], k: &Intrinsics, plane: (V3, f64), pose: &Pose, vol: &Volume, mut out: impl FnMut(V3)) {
    stripe::to_laser_plane(px, k, plane.0, plane.1, |_, x| {
        let b = pose.to_board(x);
        if vol.contains(pose, b) {
            out(b);
        }
    });
}

pub fn reconstruct(rec: &Recording, from: PoseFrom) -> Reconstruction {
    let k: Intrinsics = rec.intrinsics.into();
    let vol = rec.volume.volume(&rec.board);
    let plane = (rec.laser_plane.n, rec.laser_plane.d);
    let mut r = Reconstruction::default();
    for (i, f) in rec.frames.iter().enumerate() {
        let fp = frame_pose(f, rec, &k, from);
        if let Some(fp) = &fp {
            triangulate(&f.laser_px, &k, plane, &fp.pose, &vol, |b| {
                r.points.push(b);
                r.point_frame.push(i as u32);
            });
        }
        r.poses.push(fp);
    }
    r
}
