//! Accuracy benchmark for the scan pipeline.
//!
//! Real-scan accuracy has never been measured, and every improvement (quality
//! gates, rolling-shutter correction, IMU fusion, better calibration) needs a
//! number to move. This module produces that number:
//!
//! 1. [`sim`] simulates a hand-held scan of known solids ([`scene`]) on the
//!    board: ChArUco corner pixels with noise and occlusion, the laser stripe
//!    either projected exactly or rendered into a YUV frame and found by the
//!    app's own [`crate::laser::LaserDetector`], rolling shutter and wrongly
//!    reported frame timing, the phone's gyroscope (noise, bias, scale,
//!    jitter, clock offset, mount), a wrong camera calibration, and the
//!    app's laser-plane calibration replayed on simulated board stripes.
//! 2. The result is a [`crate::recording::Recording`] — the same format a real phone
//!    session is saved in — with the ground truth beside it ([`truth`]).
//! 3. [`reconstruct`] turns a recording into a cloud with the engine's steps
//!    (gyro fusion and rolling-shutter correction, pose from the corners,
//!    stripe ∩ laser plane, working-volume test).
//! 4. [`metrics`] scores it: pose error, laser-plane error, signed distance
//!    of every point to the true surface, and a sphere fit per ball (which
//!    also works on a real scan of a reference ball, see
//!    [`metrics::read_ply`] / [`metrics::fit_sphere`]).
//!
//! [`scenarios`] isolates one error source per run (an error budget). The
//! `scan_bench` binary runs them: `cargo run --release -p scan-algo --bin
//! scan_bench -- sweep`.

pub mod metrics;
pub mod reconstruct;
pub mod scene;
pub mod sim;
pub mod truth;

use std::time::Instant;

use serde::{Deserialize, Serialize};

use crate::board_surface::{self, BoardSurface, SurfaceView};
use crate::fusion::FusionCfg;
use crate::gate::ScanGateCfg;
use crate::geom::{Intrinsics, Pose, V3, dot, mat_vec, rodrigues};
use crate::plane::Rng;
use crate::plane::{PlaneCalibrator, PlaneFit};
use crate::recording::{BoardDims, CameraModel, CornerRec, FrameRec, Header, PlaneRec, Recording, Setup, VolumeDims};
use crate::stripe;

use metrics::Report;
use reconstruct::{Reconstruction, ReplayCfg};
use scene::{Scene, Shape};
use sim::{Camera, GyroModel, IntrinsicsError, LaserMount, StripeModel, TimingError, Trajectory, look_at, uniform};
use truth::{Synthetic, Truth};

/// Which laser plane the reconstruction uses.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum PlaneSource {
    /// The true sheet.
    Truth,
    /// The true sheet tilted about the camera's Y axis and shifted along its normal.
    Perturbed { tilt_deg: f64, offset_mm: f64 },
    /// Replay the app's laser-plane calibration (`PlaneCalibrator`, accept
    /// the first fit that passes once ready) on `frames` simulated static
    /// views of the bare board, using the same camera, noise and stripe model.
    Calibrated { frames: usize, min_lines: usize },
}

/// The board shape the pipeline computes with.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum SurfaceModel {
    /// The plane z = 0, whatever the board's true shape.
    #[default]
    Flat,
    /// The true shape.
    Truth,
    /// Measured as the app's camera calibration does: fitted
    /// (`board_surface::fit`) to `views` static views of the bare board.
    Measured { views: usize, degree: u8 },
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct BenchCfg {
    pub name: String,
    pub seed: u64,
    /// The true camera.
    pub camera: CameraModel,
    /// How the calibration the pipeline uses differs from it.
    pub intrinsics_error: IntrinsicsError,
    pub board: BoardDims,
    /// The board's true shape (flat by default).
    pub board_shape: BoardSurface,
    /// … and the one the pipeline uses.
    pub surface_model: SurfaceModel,
    pub volume: VolumeDims,
    pub shapes: Vec<Shape>,
    /// RGB tint of the solids (white = `[1, 1, 1]`).
    pub shape_tint: [f64; 3],
    pub laser: LaserMount,
    pub plane: PlaneSource,
    pub trajectory: Trajectory,
    /// σ of the ChArUco corner positions (px).
    pub corner_noise_px: f64,
    /// Rolling-shutter readout, first to last row (ms); 0 = global shutter.
    pub rolling_shutter_ms: f64,
    /// How the frame timing the camera reports is off.
    pub timing: TimingError,
    pub stripe: StripeModel,
    /// The phone's gyroscope; `None` = no gyro samples in the recording.
    pub gyro: Option<GyroModel>,
    /// The reconstruction's gyro fusion and rolling-shutter correction (the
    /// app's settings).
    pub fusion: FusionCfg,
}

impl Default for BenchCfg {
    /// The ideal case: every error source off.
    fn default() -> Self {
        BenchCfg {
            name: "ideal".into(),
            seed: 1,
            camera: CameraModel::default(),
            intrinsics_error: IntrinsicsError::default(),
            board: BoardDims::default(),
            board_shape: BoardSurface::flat(),
            surface_model: SurfaceModel::Flat,
            volume: VolumeDims::default(),
            shape_tint: [1.0; 3],
            shapes: vec![Shape::ball_on_board(144.0, 144.0, 25.0), Shape::block_on_board(60.0, 190.0, 40.0, 30.0, 20.0)],
            laser: LaserMount::default(),
            plane: PlaneSource::Truth,
            trajectory: Trajectory::default(),
            corner_noise_px: 0.0,
            rolling_shutter_ms: 0.0,
            timing: TimingError::default(),
            stripe: StripeModel::Analytic { noise_px: 0.0 },
            gyro: None,
            fusion: FusionCfg::default(),
        }
    }
}

impl BenchCfg {
    /// Every error source at a plausible level for a phone hand-held scan.
    pub fn realistic() -> Self {
        BenchCfg {
            name: "realistic".into(),
            intrinsics_error: IntrinsicsError { focal_rel: 0.003, cx_px: 1.0, cy_px: -1.0, k1: 0.0 },
            plane: PlaneSource::Calibrated { frames: 120, min_lines: 10 },
            corner_noise_px: 0.2,
            rolling_shutter_ms: 20.0,
            stripe: StripeModel::render(),
            gyro: Some(GyroModel::default()),
            ..BenchCfg::default()
        }
    }

    fn named(&self, name: &str) -> Self {
        BenchCfg { name: name.into(), ..self.clone() }
    }
}

/// One error source per scenario, then all together.
pub fn scenarios() -> Vec<BenchCfg> {
    let ideal = BenchCfg::default();
    let noisy = BenchCfg { corner_noise_px: 0.2, stripe: StripeModel::Analytic { noise_px: 0.3 }, ..ideal.clone() };
    let no_correction = FusionCfg { rolling_shutter: false, ..FusionCfg::default() };
    let mut out = vec![
        ideal.clone(),
        BenchCfg { corner_noise_px: 0.2, ..ideal.named("corner noise 0.2 px") },
        BenchCfg { stripe: StripeModel::Analytic { noise_px: 0.3 }, ..ideal.named("stripe noise 0.3 px") },
        BenchCfg { stripe: StripeModel::render(), ..ideal.named("stripe rendered + detector") },
        BenchCfg { rolling_shutter_ms: 20.0, fusion: no_correction, ..ideal.named("rolling shutter 20 ms") },
        BenchCfg { intrinsics_error: IntrinsicsError { focal_rel: 0.005, ..Default::default() }, ..ideal.named("focal length +0.5 %") },
        BenchCfg {
            intrinsics_error: IntrinsicsError { focal_rel: 0.005, ..Default::default() },
            plane: PlaneSource::Calibrated { frames: 120, min_lines: 10 },
            ..ideal.named("focal +0.5 %, plane calibrated")
        },
        BenchCfg { intrinsics_error: IntrinsicsError { k1: 0.01, ..Default::default() }, ..ideal.named("k1 +0.01") },
        BenchCfg { plane: PlaneSource::Perturbed { tilt_deg: 0.1, offset_mm: 0.0 }, ..ideal.named("laser plane tilted 0.1°") },
        BenchCfg { plane: PlaneSource::Perturbed { tilt_deg: 0.0, offset_mm: 0.5 }, ..ideal.named("laser plane shifted 0.5 mm") },
        noisy.named("corner + stripe noise"),
        BenchCfg { plane: PlaneSource::Calibrated { frames: 120, min_lines: 10 }, ..noisy.named("… + plane calibrated in sim") },
        BenchCfg { fusion: no_correction, ..BenchCfg::realistic().named("realistic, uncorrected") },
        BenchCfg::realistic(),
    ];
    // a brisk hand: ~200 mm/s, ~35°/s — what the scan gate rejects today
    let brisk = BenchCfg {
        rolling_shutter_ms: 20.0,
        trajectory: Trajectory { sweep_hz: 0.75, ..Trajectory::default() },
        gyro: Some(GyroModel::default()),
        ..noisy.clone()
    };
    out.extend([
        BenchCfg { fusion: no_correction, ..brisk.named("brisk, uncorrected") },
        BenchCfg { fusion: FusionCfg { gyro: false, ..FusionCfg::default() }, ..brisk.named("brisk, corrected from poses") },
        brisk.named("brisk, corrected with gyro"),
    ]);
    // What the simulated camera and gyro get right by construction, broken
    // one at a time. Timing and gyro errors leave an uncorrected
    // reconstruction as it is, so "brisk, uncorrected" is their baseline;
    // a different motion gets its own.
    let two_axis = Trajectory { wobble_deg: 5.0, ..brisk.trajectory };
    let skew = |rel: f64| TimingError { readout_rel: rel, ..TimingError::default() };
    let late_stamp = TimingError { timestamp_shift_ms: brisk.rolling_shutter_ms, ..TimingError::default() };
    let exposure = TimingError { exposure_ms: [2.0, 16.0], ..TimingError::default() };
    let all_timing = TimingError { readout_rel: 0.1, timestamp_shift_ms: late_stamp.timestamp_shift_ms, exposure_ms: exposure.exposure_ms };
    out.extend([
        BenchCfg { trajectory: two_axis, fusion: no_correction, ..brisk.named("brisk two-axis (wobble 5°), uncorrected") },
        BenchCfg { trajectory: two_axis, ..brisk.named("brisk two-axis (wobble 5°), gyro") },
        BenchCfg { timing: skew(0.1), ..brisk.named("brisk, gyro, skew reported +10 %") },
        BenchCfg { timing: skew(-0.1), ..brisk.named("brisk, gyro, skew reported −10 %") },
        BenchCfg { timing: late_stamp, ..brisk.named("brisk, gyro, timestamp at readout end") },
        BenchCfg { timing: exposure, ..brisk.named("brisk, gyro, exposure 2–16 ms") },
        BenchCfg { gyro: Some(GyroModel::imperfect()), ..brisk.named("brisk, imperfect gyro") },
        BenchCfg {
            trajectory: two_axis,
            timing: all_timing,
            gyro: Some(GyroModel::imperfect()),
            ..brisk.named("brisk two-axis, gyro, all of the above")
        },
    ]);
    // A board that does not lie flat (1.6 mm peak to valley), with the plane
    // calibrated on it: taken as flat, measured like the app does, and exact.
    let bent =
        BenchCfg { board_shape: bent_board(&ideal.board), plane: PlaneSource::Calibrated { frames: 120, min_lines: 10 }, ..noisy.clone() };
    out.extend([
        bent.named("bent board, taken as flat"),
        BenchCfg { surface_model: SurfaceModel::Measured { views: 60, degree: 3 }, ..bent.named("bent board, shape measured") },
        BenchCfg { surface_model: SurfaceModel::Truth, ..bent.named("bent board, true shape") },
    ]);
    // The stripe detector's hard cases, rendered: a wide, overexposed line
    // and a reddish object under a red laser.
    let rendered = BenchCfg { stripe: StripeModel::render(), corner_noise_px: 0.2, ..ideal.clone() };
    let wide = LaserMount { sheet_sigma_mm: 0.8, ..LaserMount::default() };
    out.extend([
        BenchCfg {
            laser: wide,
            stripe: StripeModel::Render { supersample: 2, ambient: 150.0, laser_gain: 1200.0, sensor_noise: 2.0 },
            ..rendered.named("wide (σ 0.8 mm), overexposed stripe")
        },
        BenchCfg { shape_tint: [1.0, 0.55, 0.5], ..rendered.named("reddish object") },
    ]);
    out
}

/// A sheet of paper bowed and twisted on a table: 1.6 mm peak to valley.
pub fn bent_board(b: &BoardDims) -> BoardSurface {
    BoardSurface::for_board(b.width_mm(), b.height_mm(), 3)
        .with_coeffs(vec![-1.5, 0.5, -1.2, 0.0, 0.15, 0.0, -0.1])
        .expect("7 terms for degree 3")
}

fn frac(x: f64) -> f64 {
    x - x.floor()
}

/// The `i`-th static view of the plane calibration: tilts, turns and
/// distances spread by low-discrepancy sequences.
fn calibration_pose(b: &BoardDims, i: usize) -> Pose {
    let i = i as f64;
    let el = (35.0 + 50.0 * frac(i * 0.618_034)).to_radians();
    let az = (360.0 * frac(i * 0.381_966 + 0.1)).to_radians();
    let dist = 280.0 + 160.0 * frac(i * 0.754_878);
    let aim = [0.5 * b.width_mm() + 80.0 * (frac(i * 0.569_840) - 0.5), 0.5 * b.height_mm() + 80.0 * (frac(i * 0.414_214) - 0.5), 0.0];
    let out = [el.cos() * az.cos(), el.cos() * az.sin(), -el.sin()];
    let side = [-az.sin(), az.cos(), 0.0];
    look_at([aim[0] + dist * out[0], aim[1] + dist * out[1], aim[2] + dist * out[2]], aim, side)
}

fn solve_corners(board: &BoardDims, surface: &BoardSurface, k: &Intrinsics, corners: &[(u32, [f64; 2])]) -> Option<Pose> {
    if corners.len() < reconstruct::MIN_CORNERS {
        return None;
    }
    let ids: Vec<(u32, V3)> = board.corners().collect();
    let (flat, px): (Vec<V3>, Vec<[f64; 2]>) = corners.iter().map(|&(id, q)| (ids[id as usize].1, q)).unzip();
    reconstruct::solve_on(surface, k, &flat, &px, None).map(|(p, _)| p)
}

/// The true scene of `cfg`.
fn scene_of(cfg: &BenchCfg) -> Scene {
    Scene { board: cfg.board, surface: cfg.board_shape.clone(), shapes: cfg.shapes.clone(), shape_tint: cfg.shape_tint }
}

/// The board shape the pipeline uses for `cfg` (see [`SurfaceModel`]).
pub fn model_surface(cfg: &BenchCfg) -> Result<BoardSurface, String> {
    let (w, h) = (cfg.board.width_mm(), cfg.board.height_mm());
    match cfg.surface_model {
        SurfaceModel::Flat => Ok(BoardSurface::flat()),
        SurfaceModel::Truth => Ok(cfg.board_shape.clone()),
        SurfaceModel::Measured { views, degree } => {
            let scene = Scene { shapes: Vec::new(), ..scene_of(cfg) };
            let k_est: Intrinsics = cfg.intrinsics_error.apply(cfg.camera).into();
            let mut cam = Camera::new(&scene, cfg.camera.into(), cfg.laser, cfg.stripe, cfg.corner_noise_px, 0.0, cfg.seed ^ 0x5EA7);
            let ids: Vec<(u32, V3)> = cfg.board.corners().collect();
            let flat = BoardSurface::flat();
            let observed: Vec<SurfaceView> = (0..views)
                .filter_map(|i| {
                    let truth = calibration_pose(&cfg.board, i);
                    let obs = cam.observe(&|_| truth, 0.0);
                    let pose = solve_corners(&cfg.board, &flat, &k_est, &obs.corners)?;
                    let (xy, px) = obs.corners.iter().map(|&(id, q)| ([ids[id as usize].1[0], ids[id as usize].1[1]], q)).unzip();
                    Some(SurfaceView { xy, px, pose })
                })
                .collect();
            board_surface::fit(&k_est, &BoardSurface::for_board(w, h, degree), &observed, w, h)
                .map(|f| f.surface)
                .ok_or_else(|| format!("the board shape could not be fitted from {} views", observed.len()))
        }
    }
}

/// Replay the app's laser-plane calibration on simulated board-only views,
/// computed on the board shape `surface`.
pub fn calibrate_plane(cfg: &BenchCfg, surface: &BoardSurface, frames: usize, min_lines: usize) -> Result<PlaneFit, String> {
    let scene = Scene { shapes: Vec::new(), ..scene_of(cfg) };
    let k_est: Intrinsics = cfg.intrinsics_error.apply(cfg.camera).into();
    let vol = cfg.volume.volume(&cfg.board);
    let mut cam = Camera::new(&scene, cfg.camera.into(), cfg.laser, cfg.stripe, cfg.corner_noise_px, 0.0, cfg.seed ^ 0xCA1B);
    let mut cal = PlaneCalibrator::new(min_lines.max(3));
    let mut solved_at = 0;
    for i in 0..frames {
        let truth = calibration_pose(&cfg.board, i);
        let obs = cam.observe(&|_| truth, 0.0);
        let Some(pose) = solve_corners(&cfg.board, surface, &k_est, &obs.corners) else { continue };
        if let Some(s) = stripe::on_board(&obs.laser_px, &k_est, &pose, None, surface, &vol) {
            let n = cal.add(&s.cam_pts);
            if cal.ready() && n != solved_at {
                solved_at = n;
                if let Some(fit) = cal.solve(false) {
                    return Ok(fit);
                }
            }
        }
    }
    Err(format!(
        "simulated plane calibration did not converge: {} lines, spread {:.2}, last fit {:?} mm",
        cal.n_lines(),
        cal.spread(),
        cal.last_rms
    ))
}

/// Simulate the scan described by `cfg` into a recording with its truth.
pub fn simulate(cfg: &BenchCfg) -> Result<Synthetic, String> {
    let scene = scene_of(cfg);
    let surface = model_surface(cfg)?;
    let (tn, td) = cfg.laser.plane();
    let used = match cfg.plane {
        PlaneSource::Truth => PlaneRec { n: tn, d: td },
        PlaneSource::Perturbed { tilt_deg, offset_mm } => {
            let n = mat_vec(&rodrigues([0.0, tilt_deg.to_radians(), 0.0]), tn);
            // keep the emitter on the plane, then shift along the normal
            PlaneRec { n, d: dot(n, cfg.laser.origin()) + offset_mm }
        }
        PlaneSource::Calibrated { frames, min_lines } => {
            let fit = calibrate_plane(cfg, &surface, frames, min_lines)?;
            PlaneRec { n: fit.n, d: fit.d }
        }
    };
    let readout_s = cfg.rolling_shutter_ms * 1e-3;
    let mut cam = Camera::new(&scene, cfg.camera.into(), cfg.laser, cfg.stripe, cfg.corner_noise_px, readout_s, cfg.seed);
    let traj = |t: f64| cfg.trajectory.pose(t);
    let mut timing_rng = Rng::new(cfg.seed ^ 0x7131);
    let ns = |s: f64| (s * 1e9).round() as i64;
    let mut frames = Vec::new();
    let mut poses = Vec::new();
    // `t`: the middle of the first row's exposure
    for t in cfg.trajectory.frame_times() {
        let obs = cam.observe(&traj, t);
        let [lo, hi] = cfg.timing.exposure_ms;
        let exposure_s = (lo + (hi - lo) * uniform(&mut timing_rng)) * 1e-3;
        frames.push(FrameRec {
            ts_ns: ns(t - 0.5 * exposure_s + cfg.timing.timestamp_shift_ms * 1e-3),
            readout_ns: ns(readout_s * (1.0 + cfg.timing.readout_rel)),
            exposure_ns: ns(exposure_s),
            width: cfg.camera.width as u32,
            height: cfg.camera.height as u32,
            pose: None,
            scan: None,
            corners: obs.corners.iter().map(|&(id, q)| CornerRec { id, u: q[0], v: q[1] }).collect(),
            laser_px: obs.laser_px.into(),
        });
        // the true middle row, whatever the reported timing says
        poses.push(traj(t + 0.5 * readout_s).into());
    }
    let gyro = cfg.gyro.map_or_else(Vec::new, |g| {
        let end = cfg.trajectory.duration_s + readout_s + 0.1;
        g.sample(&cfg.trajectory, end, &mut Rng::new(cfg.seed ^ 0x6E70))
    });
    let recording = Recording {
        header: Header {
            source: format!("synthetic: {}", cfg.name),
            board: cfg.board,
            volume: cfg.volume,
            cam_from_imu: cfg.gyro.map(|g| g.platform_mount()),
            fusion: cfg.fusion,
            scan_gate: ScanGateCfg::default(),
        },
        setups: vec![(
            0,
            Setup {
                intrinsics: cfg.intrinsics_error.apply(cfg.camera),
                laser_plane: Some(used),
                board_surface: (!surface.is_flat()).then_some(surface),
            },
        )],
        frames,
        gyro,
    };
    let truth = Truth { scene, intrinsics: cfg.camera, laser_plane: PlaneRec { n: tn, d: td }, poses, gyro: cfg.gyro.map(|g| g.calib()) };
    Ok(Synthetic { recording, truth })
}

pub struct Outcome {
    pub synthetic: Synthetic,
    pub reconstruction: Reconstruction,
    pub report: Report,
}

/// Simulate, reconstruct and score one scenario.
pub fn run(cfg: &BenchCfg) -> Result<Outcome, String> {
    let t0 = Instant::now();
    let synthetic = simulate(cfg)?;
    let replay = ReplayCfg { fusion: cfg.fusion, ..ReplayCfg::default() };
    let reconstruction = reconstruct::reconstruct(&synthetic.recording, &replay);
    let mut report = metrics::evaluate(&cfg.name, &synthetic.recording, Some(&synthetic.truth), &reconstruction);
    report.runtime_s = t0.elapsed().as_secs_f64();
    Ok(Outcome { synthetic, reconstruction, report })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::recording::PoseObs;
    use reconstruct::PoseFrom;

    fn short(mut c: BenchCfg) -> BenchCfg {
        c.trajectory.duration_s = 4.0;
        c.trajectory.azimuth_span_deg = 120.0;
        c
    }

    #[test]
    fn ideal_scan_is_exact() {
        let out = run(&short(BenchCfg::default())).unwrap();
        let r = &out.report;
        assert!(r.frames_posed == r.frames, "{} of {} posed", r.frames_posed, r.frames);
        assert!(r.points > 1_000, "{} points", r.points);
        let s = r.surface_mm.unwrap();
        assert!(s.max_abs < 1e-3, "{s:?}");
        let ball = r.spheres[0].radius_err_mm.unwrap();
        assert!(ball.abs() < 1e-3, "{ball}");
    }

    #[test]
    fn each_error_source_shows_up() {
        let ideal = short(BenchCfg::default());
        let rs = run(&BenchCfg { rolling_shutter_ms: 20.0, ..ideal.clone() }).unwrap().report;
        assert!(rs.surface_mm.unwrap().rms > 0.02, "rolling shutter: {:?}", rs.surface_mm);
        let f = run(&BenchCfg { intrinsics_error: IntrinsicsError { focal_rel: 0.005, ..Default::default() }, ..ideal.clone() })
            .unwrap()
            .report;
        assert!(f.surface_mm.unwrap().rms > 0.02, "focal: {:?}", f.surface_mm);
        let p = run(&BenchCfg { plane: PlaneSource::Perturbed { tilt_deg: 0.1, offset_mm: 0.0 }, ..ideal }).unwrap().report;
        assert!((p.plane_angle_deg.unwrap() - 0.1).abs() < 1e-6);
        assert!(p.surface_mm.unwrap().rms > 0.05, "plane: {:?}", p.surface_mm);
    }

    #[test]
    fn a_bent_board_needs_its_shape() {
        let bent = BenchCfg {
            board_shape: bent_board(&BoardDims::default()),
            plane: PlaneSource::Calibrated { frames: 120, min_lines: 10 },
            corner_noise_px: 0.2,
            ..short(BenchCfg::default())
        };
        let flat = run(&bent).unwrap().report;
        let measured = run(&BenchCfg { surface_model: SurfaceModel::Measured { views: 60, degree: 3 }, ..bent }).unwrap().report;
        let (f, m) = (flat.surface_mm.unwrap(), measured.surface_mm.unwrap());
        // taken as flat, the plane calibration and the poses carry the bow
        assert!(flat.plane_angle_deg.unwrap() > 5.0 * measured.plane_angle_deg.unwrap(), "{flat:?} vs {measured:?}");
        assert!(f.mean.abs() > 0.1 && m.mean.abs() < 0.05, "mean {} vs {}", f.mean, m.mean);
        assert!(measured.pose_pos_mm.unwrap().rms < 0.7 * flat.pose_pos_mm.unwrap().rms);
    }

    #[test]
    fn simulated_plane_calibration_converges() {
        let cfg = BenchCfg { corner_noise_px: 0.2, stripe: StripeModel::Analytic { noise_px: 0.3 }, ..BenchCfg::default() };
        let fit = calibrate_plane(&cfg, &BoardSurface::flat(), 120, 10).unwrap();
        let (n, _) = cfg.laser.plane();
        let ang = dot(n, fit.n).abs().min(1.0).acos().to_degrees();
        assert!(ang < 0.5, "plane off by {ang}°");
    }

    #[test]
    fn realistic_scan_stays_in_a_sane_range() {
        // regression guard, not a spec: the realistic scenario on a short path
        let mut cfg = short(BenchCfg::realistic());
        cfg.trajectory.duration_s = 2.0;
        cfg.trajectory.azimuth_span_deg = 60.0;
        let r = run(&cfg).unwrap().report;
        assert!(r.points > 1_000, "{} points", r.points);
        let s = r.surface_mm.unwrap();
        assert!(s.rms < 3.0, "{s:?}");
    }

    #[test]
    fn rolling_shutter_correction_pays_off_at_speed() {
        let brisk = BenchCfg {
            corner_noise_px: 0.2,
            rolling_shutter_ms: 20.0,
            gyro: Some(GyroModel::default()),
            trajectory: Trajectory { duration_s: 6.0, azimuth_span_deg: 108.0, sweep_hz: 0.75, ..Trajectory::default() },
            ..BenchCfg::default()
        };
        let off = BenchCfg { fusion: FusionCfg { rolling_shutter: false, ..FusionCfg::default() }, ..brisk.clone() };
        let (raw, corrected) = (run(&off).unwrap().report, run(&brisk).unwrap().report);
        let (a, b) = (raw.pose_pos_mm.unwrap().rms, corrected.pose_pos_mm.unwrap().rms);
        assert!(b < 0.6 * a, "pose error {a:.2} mm uncorrected, {b:.2} mm corrected");
        assert!(corrected.corrected_frames > corrected.frames * 9 / 10);
    }

    fn gyro_scan(duration_s: f64, wobble_deg: f64) -> BenchCfg {
        BenchCfg {
            corner_noise_px: 0.2,
            rolling_shutter_ms: 20.0,
            gyro: Some(GyroModel::default()),
            trajectory: Trajectory { duration_s, azimuth_span_deg: 18.0 * duration_s, wobble_deg, ..Trajectory::default() },
            ..BenchCfg::default()
        }
    }

    #[test]
    fn gyro_self_calibration_finds_the_simulated_gyro() {
        let g = run(&gyro_scan(8.0, 1.0)).unwrap().report.gyro.expect("the gyro calibrates");
        // the hand turns about one axis: the platform's mount (0.5° off) is kept
        assert_eq!(g.mount_sigma_deg, None, "{g:?}");
        assert!((g.rotation_err_deg.unwrap() - 0.5).abs() < 1e-6, "{g:?}");
        // the orbit turns the camera at a steady 18°/s, which hides the offset:
        // only changes of the rate show it (and only they suffer from it)
        assert!(g.offset_err_ms.unwrap().abs() < 8.0, "{g:?}");
        assert!(g.from_frame.is_some_and(|f| f <= 61), "{g:?}");
    }

    #[test]
    fn a_two_axis_hand_fits_the_mount() {
        let g = run(&gyro_scan(16.0, 5.0)).unwrap().report.gyro.expect("the gyro calibrates");
        assert!(g.mount_sigma_deg.is_some(), "the mount is fitted: {g:?}");
        assert!(g.rotation_err_deg.unwrap() < 0.4, "better than the platform's 0.5°: {g:?}");
        assert!(g.offset_err_ms.unwrap().abs() < 3.0, "{g:?}");
    }

    #[test]
    fn recorded_device_poses_are_trusted_and_drive_the_rolling_shutter_correction() {
        let cfg = short(BenchCfg { rolling_shutter_ms: 20.0, gyro: Some(GyroModel::default()), ..BenchCfg::default() });
        let Synthetic { mut recording, truth } = simulate(&cfg).unwrap();
        // what a phone records: its (here: the true) pose with the fit it was trusted on
        for (f, p) in recording.frames.iter_mut().zip(&truth.poses) {
            f.pose = Some(PoseObs { r: p.r, t: p.t, reproj_rms: 0.3, margin: Some(10.0) });
        }
        let replay = ReplayCfg { pose_from: PoseFrom::Recorded, ..ReplayCfg::as_recorded(&recording) };
        let r = reconstruct::reconstruct(&recording, &replay);
        assert!(r.corrected_frames > recording.frames.len() / 2, "{} of {}", r.corrected_frames, recording.frames.len());
        assert!(r.gyro_fit.is_some(), "the gyro calibrates against the recorded poses");
    }

    #[test]
    fn a_synthetic_recording_and_its_truth_round_trip() {
        let mut cfg = short(BenchCfg { gyro: Some(GyroModel::default()), ..BenchCfg::default() });
        cfg.trajectory.duration_s = 0.2;
        let syn = simulate(&cfg).unwrap();
        let path = std::env::temp_dir().join(format!("easyscanner-synthetic-{}.jsonl", std::process::id()));
        syn.save(&path).unwrap();
        let rec = Recording::load(&path).unwrap();
        let truth = Truth::load_for(&path).unwrap().expect("saved next to it");
        let _ = std::fs::remove_file(truth::truth_path(&path));
        let _ = std::fs::remove_file(&path);
        // serde_json may round the last bit of a float: compare the content
        assert_eq!(rec.header, syn.recording.header);
        assert_eq!(rec.frames.len(), syn.recording.frames.len());
        assert_eq!(rec.gyro.len(), syn.recording.gyro.len());
        assert_eq!(truth.scene, syn.truth.scene);
        for (a, b) in syn.recording.frames.iter().zip(&rec.frames) {
            assert_eq!(a.laser_px, b.laser_px);
            for (p, q) in a.corners.iter().zip(b.corners.iter()) {
                assert!(p.id == q.id && (p.u - q.u).abs() < 1e-9 && (p.v - q.v).abs() < 1e-9);
            }
        }
    }
}
