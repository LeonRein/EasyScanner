//! Live intrinsics self-calibration from the stream — port of the desktop prototype's
//! `calib_live.py`: collect diverse ChArUco views, then one
//! `cv::calibrateCamera` (USE_INTRINSIC_GUESS | ZERO_TANGENT_DIST | FIX_K3)
//! behind the same RMS and plausibility gates. The solve runs on its own
//! thread so the viewfinder keeps going.
//!
//! The views also measure the board's shape (`scan_algo::board_surface`):
//! a printed board is rarely flat, and a K solved for a flat one absorbs
//! the warp. So the shape is fitted from the solve's poses, and K solved
//! again with the corners where the shape puts them.

use std::thread::JoinHandle;

use opencv::calib3d;
use opencv::core::{self, Mat, Point2f, Point3f, Size, TermCriteria, Vector};
use opencv::prelude::*;
use scan_algo::board_surface::{self, BoardSurface, SurfaceFit, SurfaceView};
use scan_algo::config::{BoardSpec, intrinsics_plausible};
use scan_algo::geom::{Intrinsics, Pose};

use crate::board::{make_board, pose_from};

/// Reprojection-RMS ceiling (px) at 1920 px, scaled up for larger frames.
const MAX_RMS_PX: f64 = 2.5;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CalibState {
    Idle = 0,
    Collecting = 1,
    Solving = 2,
    Saved = 3,
    Rejected = 4,
}

pub enum Outcome {
    /// `surface`: the board's shape measured with it, when asked for and
    /// the views determined one.
    Solved {
        k: Intrinsics,
        rms: f64,
        surface: Option<SurfaceFit>,
    },
    Rejected {
        rms: f64,
        reason: String,
    },
}

struct View {
    corners: Vec<Point2f>,
    ids: Vec<i32>,
}

pub struct LiveCalibrator {
    pub target: usize,
    min_corners: usize,
    views: Vec<View>,
    wh: Option<(u32, u32)>,
    solved_at: usize,
    job: Option<JoinHandle<Outcome>>,
    pub last_rms: Option<f64>,
}

impl LiveCalibrator {
    pub fn new(target: usize) -> Self {
        LiveCalibrator { target, min_corners: 8, views: Vec::new(), wh: None, solved_at: 0, job: None, last_rms: None }
    }

    pub fn count(&self) -> usize {
        self.views.len()
    }

    pub fn solving(&self) -> bool {
        self.job.is_some()
    }

    /// Store one detected view. Rejects too few corners and near-duplicates
    /// (same corner count, centroid moved less than ~8 px at 1920 px), so
    /// holding the board still does not flood the set. Views must share one
    /// resolution; a different one starts the set over.
    pub fn capture(&mut self, corners: &Vector<Point2f>, ids: &Vector<i32>, wh: (u32, u32)) -> bool {
        if self.job.is_some() || corners.len() < self.min_corners {
            return false;
        }
        if self.wh != Some(wh) {
            self.views.clear();
            self.solved_at = 0;
            self.wh = Some(wh);
        }
        let c: Vec<Point2f> = corners.to_vec();
        let centroid = mean(&c);
        let tol = 8.0 * (wh.0.max(wh.1) as f32 / 1920.0).max(0.25);
        let dup = self.views.iter().any(|v| {
            let m = mean(&v.corners);
            v.corners.len() == c.len() && ((m.0 - centroid.0).powi(2) + (m.1 - centroid.1).powi(2)).sqrt() < tol
        });
        if dup {
            return false;
        }
        self.views.push(View { corners: c, ids: ids.to_vec() });
        true
    }

    /// Start the set over. A solve still running is abandoned: its views
    /// belong to the set being dropped (another camera, focus or board), so
    /// its result must never be saved. The thread finishes on its own.
    pub fn reset(&mut self) {
        self.job = None;
        self.views.clear();
        self.solved_at = 0;
        self.last_rms = None;
    }

    /// Start a background solve once the set reaches the target, and again
    /// every `target/12` new views after a rejected attempt. With a
    /// `surface_degree` the board's shape is measured along.
    pub fn maybe_solve(&mut self, spec: BoardSpec, seed: Intrinsics, surface_degree: Option<u8>) -> bool {
        let step = (self.target / 12).max(1);
        let Some(wh) = self.wh else { return false };
        if self.job.is_some() || self.views.len() < self.target || self.views.len() - self.solved_at < step {
            return false;
        }
        self.solved_at = self.views.len();
        let views: Vec<(Vec<Point2f>, Vec<i32>)> = self.views.iter().map(|v| (v.corners.clone(), v.ids.clone())).collect();
        let (min_corners, min_views) = (self.min_corners, self.target);
        let limits = Limits { min_corners, min_views, surface_degree };
        self.job = Some(std::thread::spawn(move || match solve(spec, &views, wh, seed, limits) {
            Ok(o) => o,
            Err(e) => Outcome::Rejected { rms: f64::INFINITY, reason: format!("calibrateCamera failed: {e}") },
        }));
        true
    }

    /// The finished background solve, if any.
    pub fn poll(&mut self) -> Option<Outcome> {
        if !self.job.as_ref()?.is_finished() {
            return None;
        }
        let out = self.job.take()?.join().unwrap_or(Outcome::Rejected { rms: f64::INFINITY, reason: "solver panicked".into() });
        match &out {
            Outcome::Solved { rms, .. } | Outcome::Rejected { rms, .. } => self.last_rms = rms.is_finite().then_some(*rms),
        }
        Some(out)
    }
}

fn mean(c: &[Point2f]) -> (f32, f32) {
    let n = c.len().max(1) as f32;
    (c.iter().map(|p| p.x).sum::<f32>() / n, c.iter().map(|p| p.y).sum::<f32>() / n)
}

/// What a solve needs from the views, and whether it measures the shape.
#[derive(Clone, Copy)]
struct Limits {
    min_corners: usize,
    min_views: usize,
    surface_degree: Option<u8>,
}

fn solve(
    spec: BoardSpec,
    views: &[(Vec<Point2f>, Vec<i32>)],
    (w, h): (u32, u32),
    seed: Intrinsics,
    limits: Limits,
) -> opencv::Result<Outcome> {
    let Limits { min_corners, min_views, surface_degree } = limits;
    let board = make_board(&spec)?;
    let mut obj_all: Vector<Vector<Point3f>> = Vector::new();
    let mut img_all: Vector<Vector<Point2f>> = Vector::new();
    for (corners, ids) in views {
        let (mut obj, mut img) = (Vector::<Point3f>::new(), Vector::<Point2f>::new());
        board.match_image_points(&Vector::from_slice(corners), &Vector::from_slice(ids), &mut obj, &mut img)?;
        if obj.len() >= min_corners {
            obj_all.push(obj);
            img_all.push(img);
        }
    }
    if obj_all.len() < min_views {
        return Ok(Outcome::Rejected { rms: f64::INFINITY, reason: format!("only {} usable views of {min_views}", obj_all.len()) });
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
    let Solved { k: out, rms, .. } = solved;

    let gate = MAX_RMS_PX * (w as f64 / 1920.0).max(1.0);
    if !rms.is_finite() || rms > gate {
        return Ok(Outcome::Rejected { rms, reason: format!("rms {rms:.2} px > {gate:.2}") });
    }
    if !intrinsics_plausible(&out) {
        return Ok(Outcome::Rejected {
            rms,
            reason: format!(
                "K implausible (fx {:.0} fy {:.0} cx {:.0} cy {:.0} k1 {:.3}) — more varied angles",
                out.fx, out.fy, out.cx, out.cy, out.dist[0]
            ),
        });
    }
    Ok(Outcome::Solved { k: out, rms, surface })
}

/// One `calibrateCamera` result.
struct Solved {
    k: Intrinsics,
    rms: f64,
    poses: Vec<Pose>,
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
    let rms = calib3d::calibrate_camera(obj, img, size, &mut k, &mut d, &mut rvecs, &mut tvecs, flags, criteria)?;
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
    Ok(Solved { k, rms, poses })
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
