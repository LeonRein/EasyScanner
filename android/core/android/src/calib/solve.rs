//! The intrinsics solve — port of the desktop prototype's `calib_live.py`:
//! one `cv::calibrateCamera` (USE_INTRINSIC_GUESS | ZERO_TANGENT_DIST |
//! FIX_K3) over the collected ChArUco views, behind RMS, plausibility and
//! focal-uncertainty gates.
//!
//! The views also measure the board's shape (`scan_algo::board_surface`):
//! a printed board is rarely flat, and a K solved for a flat one absorbs
//! the warp. So the shape is fitted from the solve's poses, and K solved
//! again with the corners where the shape puts them.

use opencv::calib3d;
use opencv::core::{self, Mat, Point2f, Point3f, Size, TermCriteria, Vector};
use opencv::prelude::*;
use scan_algo::board_surface::{self, BoardSurface, SurfaceFit, SurfaceView};
use scan_algo::config::{BoardSpec, intrinsics_plausible};
use scan_algo::geom::{Intrinsics, Pose};

use crate::board::{make_board, pose_from};

/// Reprojection-RMS ceiling (px) at 1920 px, scaled up for larger frames.
/// A phone with a printed board reaches 0.3–0.6 px; more means blurred or
/// misdetected views, or a board that is not the one configured.
const MAX_RMS_PX: f64 = 1.5;
/// The focal length's standard deviation must stay below this fraction of
/// it: the views must pin K down, not just fit it (0.5 % of f costs about
/// 1.4 mm with the true laser plane in the benchmark).
const MAX_FOCAL_SIGMA_REL: f64 = 0.002;
/// A view reprojecting this many times worse than the median view (and at
/// least `OUTLIER_MIN_PX`) is dropped and K solved again without it.
const OUTLIER_FACTOR: f64 = 3.0;
const OUTLIER_MIN_PX: f64 = 0.5;
/// At most this fraction of the views is dropped as outliers. More than
/// that is no stray view but a systematic problem, left to the RMS gate.
const MAX_OUTLIER_FRAC: f64 = 0.2;

/// A camera calibration that passed the gates.
pub struct Calibration {
    pub k: Intrinsics,
    pub rms: f64,
    /// Standard deviations of fx, fy, cx, cy (px).
    pub sigma: [f64; 4],
    /// Views dropped as outliers before the final solve.
    pub dropped: usize,
    /// The board's shape measured with it, when asked for and the views
    /// determined one.
    pub surface: Option<SurfaceFit>,
}

/// Why a solve's result was not saved.
#[derive(Clone, Debug, PartialEq)]
pub enum Rejection {
    /// Fewer views than the target held enough matched corners.
    TooFewViews { usable: usize, needed: usize },
    /// The views do not agree on one camera model (blur, misdetections, a
    /// board other than the one configured).
    HighRms { rms: f64, limit: f64 },
    /// The solved K is no camera a phone has.
    Implausible { rms: f64, k: Intrinsics },
    /// The views fit, but do not pin the focal length down.
    FocalUncertain { rms: f64, sigma: f64, f: f64 },
    /// OpenCV failed, or the solver thread died.
    SolverFailed(String),
}

impl Rejection {
    /// The code the app reads from `h::CALIB_REJECTION` (0 is none).
    pub fn code(&self) -> u8 {
        match self {
            Rejection::TooFewViews { .. } => 1,
            Rejection::HighRms { .. } => 2,
            Rejection::Implausible { .. } => 3,
            Rejection::FocalUncertain { .. } => 4,
            Rejection::SolverFailed(_) => 5,
        }
    }

    /// The solve's reprojection RMS, when it got that far.
    pub fn rms(&self) -> Option<f64> {
        match self {
            Rejection::HighRms { rms, .. } | Rejection::Implausible { rms, .. } | Rejection::FocalUncertain { rms, .. } => Some(*rms),
            Rejection::TooFewViews { .. } | Rejection::SolverFailed(_) => None,
        }
    }
}

impl std::fmt::Display for Rejection {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Rejection::TooFewViews { usable, needed } => write!(f, "only {usable} usable views of {needed}"),
            Rejection::HighRms { rms, limit } => write!(f, "rms {rms:.2} px > {limit:.2}"),
            Rejection::Implausible { k, .. } => {
                write!(f, "K implausible (fx {:.0} fy {:.0} cx {:.0} cy {:.0} k1 {:.3})", k.fx, k.fy, k.cx, k.cy, k.dist[0])
            }
            Rejection::FocalUncertain { sigma, f: focal, .. } => write!(f, "focal length uncertain (±{sigma:.1} px of {focal:.0})"),
            Rejection::SolverFailed(e) => write!(f, "solver failed: {e}"),
        }
    }
}

/// What came of a solve.
pub enum Outcome {
    Solved(Box<Calibration>),
    Rejected(Rejection),
}

/// One view's matched-up ChArUco detection.
pub struct ViewCorners {
    pub corners: Vec<Point2f>,
    pub ids: Vec<i32>,
}

/// What a solve needs from the views, and whether it measures the shape.
#[derive(Clone, Copy)]
pub struct Limits {
    pub min_corners: usize,
    pub min_views: usize,
    pub surface_degree: Option<u8>,
}

/// Solve K from `views`, behind the RMS, plausibility and focal
/// uncertainty gates.
pub fn solve(spec: BoardSpec, views: &[ViewCorners], (w, h): (u32, u32), seed: Intrinsics, limits: Limits) -> opencv::Result<Outcome> {
    let Limits { min_corners, min_views, surface_degree } = limits;
    let board = make_board(&spec)?;
    let mut obj_all: Vector<Vector<Point3f>> = Vector::new();
    let mut img_all: Vector<Vector<Point2f>> = Vector::new();
    for v in views {
        let (mut obj, mut img) = (Vector::<Point3f>::new(), Vector::<Point2f>::new());
        board.match_image_points(&Vector::from_slice(&v.corners), &Vector::from_slice(&v.ids), &mut obj, &mut img)?;
        if obj.len() >= min_corners {
            obj_all.push(obj);
            img_all.push(img);
        }
    }
    if obj_all.len() < min_views {
        return Ok(Outcome::Rejected(Rejection::TooFewViews { usable: obj_all.len(), needed: min_views }));
    }

    // Seed with the camera's own estimate when it is sane, else the classic
    // centred guess with f ≈ the longer side.
    let seed = seed.for_frame(w, h);
    let seed = if intrinsics_plausible(&seed) {
        seed
    } else {
        let f = w.max(h) as f64;
        Intrinsics { fx: f, fy: f, cx: w as f64 / 2.0, cy: h as f64 / 2.0, dist: [0.0; 5], width: w as f64, height: h as f64 }
    };
    // the distortion starts from zero, as it always did
    let seed = Intrinsics { dist: [0.0; 5], ..seed };
    let size = Size::new(w as i32, h as i32);
    let mut solved = calibrate(&obj_all, &img_all, size, &seed)?;

    // views far worse than the rest (blurred, a misdetected corner): drop
    // them and solve again
    let mut dropped = 0;
    if let Some(keep) = inliers(&solved.view_errors) {
        dropped = keep.iter().filter(|&&k| !k).count();
        obj_all = obj_all.iter().zip(&keep).filter(|&(_, &k)| k).map(|(v, _)| v).collect();
        img_all = img_all.iter().zip(&keep).filter(|&(_, &k)| k).map(|(v, _)| v).collect();
        solved = calibrate(&obj_all, &img_all, size, &solved.k)?;
    }

    // the board's shape from the solve's poses, then K again with the
    // corners on that shape; kept when it fits the views better
    let mut surface = None;
    if let Some(degree) = surface_degree
        && let Some(fit) = measure_surface(&spec, degree, &solved, &obj_all, &img_all)?
    {
        let warped = on_surface(&obj_all, &fit.surface);
        let again = calibrate(&warped, &img_all, size, &solved.k)?;
        if again.rms < solved.rms {
            // the shape once more, seen through the K it gave
            surface = measure_surface(&spec, degree, &again, &obj_all, &img_all)?.or(Some(fit));
            solved = again;
        }
    }
    let Solved { k: out, rms, sigma, .. } = solved;

    let limit = MAX_RMS_PX * (w as f64 / 1920.0).max(1.0);
    if !rms.is_finite() || rms > limit {
        return Ok(Outcome::Rejected(Rejection::HighRms { rms, limit }));
    }
    if !intrinsics_plausible(&out) {
        return Ok(Outcome::Rejected(Rejection::Implausible { rms, k: out }));
    }
    if !(sigma[0] <= MAX_FOCAL_SIGMA_REL * out.fx && sigma[1] <= MAX_FOCAL_SIGMA_REL * out.fy) {
        return Ok(Outcome::Rejected(Rejection::FocalUncertain { rms, sigma: sigma[0].max(sigma[1]), f: out.fx }));
    }
    Ok(Outcome::Solved(Box::new(Calibration { k: out, rms, sigma, dropped, surface })))
}

/// One `calibrateCamera` result.
struct Solved {
    k: Intrinsics,
    rms: f64,
    /// Standard deviations of fx, fy, cx, cy (px).
    sigma: [f64; 4],
    /// Each view's reprojection RMS (px).
    view_errors: Vec<f64>,
    poses: Vec<Pose>,
}

/// Which views to keep, given each view's reprojection error: `None` when
/// there is no outlier, or more than [`MAX_OUTLIER_FRAC`] of them. The drop
/// may leave fewer views than the target: the solve runs as soon as the
/// target is reached, so any drop at all would otherwise be refused.
fn inliers(view_errors: &[f64]) -> Option<Vec<bool>> {
    let mut sorted = view_errors.to_vec();
    sorted.sort_by(f64::total_cmp);
    let median = *sorted.get(sorted.len() / 2)?;
    let limit = (OUTLIER_FACTOR * median).max(OUTLIER_MIN_PX);
    let keep: Vec<bool> = view_errors.iter().map(|&e| e <= limit).collect();
    let outliers = keep.iter().filter(|&&k| !k).count();
    (outliers > 0 && outliers as f64 <= MAX_OUTLIER_FRAC * view_errors.len() as f64).then_some(keep)
}

/// `cv::calibrateCamera` from `seed`. Board points off the plane z = 0 need
/// the intrinsic guess, which is always given.
fn calibrate(obj: &Vector<Vector<Point3f>>, img: &Vector<Vector<Point2f>>, size: Size, seed: &Intrinsics) -> opencv::Result<Solved> {
    let mut k = Mat::from_slice_2d(&[[seed.fx, 0.0, seed.cx], [0.0, seed.fy, seed.cy], [0.0, 0.0, 1.0]])?;
    let mut d = Mat::from_slice(&seed.dist)?.try_clone()?;
    let (mut rvecs, mut tvecs) = (Vector::<Mat>::new(), Vector::<Mat>::new());
    // never FIX_ASPECT_RATIO / FIX_PRINCIPAL_POINT (same as the server)
    let flags = calib3d::CALIB_USE_INTRINSIC_GUESS | calib3d::CALIB_ZERO_TANGENT_DIST | calib3d::CALIB_FIX_K3;
    let criteria = TermCriteria::new(core::TermCriteria_COUNT + core::TermCriteria_EPS, 30, f64::EPSILON)?;
    let (mut std_intrinsics, mut std_extrinsics, mut per_view) = (Mat::default(), Mat::default(), Mat::default());
    let rms = calib3d::calibrate_camera_extended(
        obj,
        img,
        size,
        &mut k,
        &mut d,
        &mut rvecs,
        &mut tvecs,
        &mut std_intrinsics,
        &mut std_extrinsics,
        &mut per_view,
        flags,
        criteria,
    )?;
    let mut sigma = [0.0; 4];
    for (i, v) in sigma.iter_mut().enumerate() {
        *v = *std_intrinsics.at::<f64>(i as i32)?;
    }
    let view_errors = (0..per_view.rows()).map(|i| per_view.at::<f64>(i).copied()).collect::<opencv::Result<Vec<f64>>>()?;
    let mut dist = [0.0; 5];
    for (i, v) in dist.iter_mut().enumerate() {
        *v = *d.at::<f64>(i as i32)?;
    }
    let k = Intrinsics {
        fx: *k.at_2d::<f64>(0, 0)?,
        fy: *k.at_2d::<f64>(1, 1)?,
        cx: *k.at_2d::<f64>(0, 2)?,
        cy: *k.at_2d::<f64>(1, 2)?,
        dist,
        width: f64::from(size.width),
        height: f64::from(size.height),
    };
    let poses = rvecs.iter().zip(tvecs.iter()).map(|(r, t)| pose_from(&r, &t)).collect::<opencv::Result<_>>()?;
    Ok(Solved { k, rms, sigma, view_errors, poses })
}

/// The board's shape from a solve's views and poses, when they determine it.
fn measure_surface(
    spec: &BoardSpec,
    degree: u8,
    solved: &Solved,
    obj: &Vector<Vector<Point3f>>,
    img: &Vector<Vector<Point2f>>,
) -> opencv::Result<Option<SurfaceFit>> {
    let views: Vec<SurfaceView> = obj
        .iter()
        .zip(img.iter())
        .zip(&solved.poses)
        .map(|((o, i), pose)| SurfaceView {
            xy: o.iter().map(|p| [f64::from(p.x), f64::from(p.y)]).collect(),
            px: i.iter().map(|p| [f64::from(p.x), f64::from(p.y)]).collect(),
            pose: *pose,
        })
        .collect();
    let (bw, bh) = (spec.width_mm(), spec.height_mm());
    Ok(board_surface::fit(&solved.k, &BoardSurface::for_board(bw, bh, degree), &views, bw, bh))
}

/// The views' board points moved onto `surface`.
fn on_surface(obj: &Vector<Vector<Point3f>>, surface: &BoardSurface) -> Vector<Vector<Point3f>> {
    obj.iter().map(|o| o.iter().map(|p| Point3f::new(p.x, p.y, surface.height(f64::from(p.x), f64::from(p.y)) as f32)).collect()).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn one_blurred_view_is_dropped_even_at_the_target_count() {
        let mut errors = vec![0.3; 64];
        errors[17] = 2.4;
        let keep = inliers(&errors).expect("an outlier");
        assert_eq!(keep.iter().filter(|&&k| !k).count(), 1);
        assert!(!keep[17]);
    }

    #[test]
    fn small_or_widespread_errors_drop_nothing() {
        assert_eq!(inliers(&[0.3; 64]), None);
        // above 3x the median but below the floor: the views are all fine
        assert_eq!(inliers(&[0.05, 0.05, 0.05, 0.4]), None);
        // a quarter of the views off is a systematic problem, not stray views
        let errors: Vec<f64> = (0..64).map(|i| if i % 4 == 0 { 3.0 } else { 0.3 }).collect();
        assert_eq!(inliers(&errors), None);
        assert_eq!(inliers(&[]), None);
    }
}
