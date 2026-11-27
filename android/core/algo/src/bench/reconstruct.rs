//! Recording → point cloud, with the same steps the app's engine runs per
//! frame (`easyscanner-core/src/engine.rs`): the gyro samples that arrived
//! so far into the [`PoseFusion`], the board pose from the corners (rolling
//! shutter corrected), then stripe pixel → camera ray ∩ laser plane → the
//! frame's reference instant → board frame → keep if inside the working
//! volume.

use std::convert::Infallible;

use serde::{Deserialize, Serialize};

use crate::fusion::{FusionCfg, PoseFusion};
use crate::gate::{FrameQuality, ScanGateCfg};
use crate::geom::{Intrinsics, Pose, V3};
use crate::imu::GyroFit;
use crate::rolling::{Correction, solve_corrected};
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

/// Gyro samples the app holds when it processes a frame: the frame's own
/// plus the sensor's delivery latency.
const GYRO_LATENCY_NS: i64 = 40_000_000;

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
    /// Pose at each frame's reference instant (the middle row).
    pub poses: Vec<Option<FramePose>>,
    /// Frames whose observations were rolling-shutter corrected.
    pub corrected_frames: usize,
    /// The gyro calibration at the end, and the first frame it was in use.
    pub gyro_fit: Option<GyroFit>,
    pub gyro_from_frame: Option<usize>,
}

/// The frame's pose: the recorded one, or solved from its corners (with the
/// rolling-shutter correction when there is one).
pub fn frame_pose(f: &FrameRec, rec: &Recording, k: &Intrinsics, from: PoseFrom, corr: Option<&Correction>) -> Option<FramePose> {
    let recorded = f.pose.map(|p| FramePose { pose: p.into(), reproj_rms: f64::NAN, corners: f.corners.len() });
    let solve = || {
        if f.corners.len() < MIN_CORNERS {
            return None;
        }
        let ids: Vec<(u32, V3)> = rec.board.corners().collect();
        let (obj, px): (Vec<V3>, Vec<[f64; 2]>) =
            f.corners.iter().filter_map(|c| ids.get(c.id as usize).map(|&(_, p)| (p, [c.u, c.v]))).unzip();
        let Ok(solved) = solve_corrected(corr, k, &obj, &px, |px| Ok::<_, Infallible>(pnp::solve_planar(k, &obj, px)), |s| s.0);
        let (pose, rms) = solved?;
        Some(FramePose { pose, reproj_rms: rms, corners: obj.len() })
    };
    match from {
        PoseFrom::Recorded => recorded,
        PoseFrom::Corners => solve(),
        PoseFrom::Auto => recorded.or_else(solve),
    }
}

/// Triangulate one frame's stripe into board-frame points inside the volume.
pub fn triangulate(
    px: &[[f32; 2]],
    k: &Intrinsics,
    plane: (V3, f64),
    pose: &Pose,
    corr: Option<&Correction>,
    vol: &Volume,
    mut out: impl FnMut(V3),
) {
    stripe::to_laser_plane(px, k, plane.0, plane.1, |i, x| {
        let x = corr.map_or(x, |c| c.stripe_point(x, px[i][1] as f64));
        let b = pose.to_board(x);
        if vol.contains(pose, b) {
            out(b);
        }
    });
}

pub fn reconstruct(rec: &Recording, from: PoseFrom, fusion_cfg: &FusionCfg) -> Reconstruction {
    let k: Intrinsics = rec.intrinsics.into();
    let vol = rec.volume.volume(&rec.board);
    let plane = (rec.laser_plane.n, rec.laser_plane.d);
    // the app trusts a pose for the gyro calibration when it passes the
    // scan gate's pose checks
    let gate = ScanGateCfg::default();
    let mut fusion = PoseFusion::new(*fusion_cfg);
    fusion.set_platform_mount(rec.cam_from_imu);
    let mut gyro = rec.gyro.iter().peekable();
    let mut r = Reconstruction::default();
    for (i, f) in rec.frames.iter().enumerate() {
        let clock = f.clock(k.height);
        let arrived = clock.span_ns().1 as i64 + GYRO_LATENCY_NS;
        while let Some(s) = gyro.next_if(|s| s.t_ns <= arrived) {
            fusion.push_gyro(*s);
        }
        if r.gyro_from_frame.is_none() && fusion.gyro_fit().is_some() {
            r.gyro_from_frame = Some(i);
        }
        let corr = fusion.correction(&clock);
        r.corrected_frames += usize::from(corr.is_some());
        let fp = frame_pose(f, rec, &k, from, corr.as_ref());
        // the benchmark's PnP has a single solution: no ambiguity margin
        let trusted = fp.filter(|p| {
            let q = FrameQuality { reproj_rms: p.reproj_rms, corners: p.corners, margin: f64::INFINITY, speed: None };
            gate.check_pose(&q).is_ok()
        });
        fusion.observe(&clock, trusted.as_ref().map(|p| &p.pose));
        if let Some(fp) = &fp {
            triangulate(&f.laser_px, &k, plane, &fp.pose, corr.as_ref(), &vol, |b| {
                r.points.push(b);
                r.point_frame.push(i as u32);
            });
        }
        r.poses.push(fp);
    }
    r.gyro_fit = fusion.gyro_fit().copied();
    r
}
