//! Accuracy benchmark for the scan pipeline.
//!
//! Real-scan accuracy has never been measured, and every improvement (quality
//! gates, rolling-shutter correction, IMU fusion, better calibration) needs a
//! number to move. This module produces that number:
//!
//! 1. [`sim`] simulates a hand-held scan of known solids ([`scene`]) on the
//!    board: ChArUco corner pixels with noise and occlusion, the laser stripe
//!    either projected exactly or rendered into a YUV frame and found by the
//!    app's own [`crate::laser::LaserDetector`], rolling shutter, the phone's
//!    gyroscope (noise, bias, clock offset, mounting), a wrong
//!    camera calibration, and the app's laser-plane calibration replayed on
//!    simulated board stripes.
//! 2. The result is a [`recording::Recording`] — the same format a real phone
//!    session can be saved in — with the ground truth attached.
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
pub mod pnp;
pub mod reconstruct;
pub mod recording;
pub mod scene;
pub mod sim;

use std::time::Instant;

use serde::{Deserialize, Serialize};

use crate::fusion::FusionCfg;
use crate::geom::{Intrinsics, Pose, V3, dot, mat_vec, rodrigues};
use crate::plane::Rng;
use crate::plane::{PlaneCalibrator, PlaneFit};
use crate::rolling::MotionSource;
use crate::stripe;

use metrics::Report;
use reconstruct::{PoseFrom, Reconstruction};
use recording::{CornerRec, FrameRec, PlaneRec, Recording, Truth, VolumeDims};
use scene::{BoardDims, Scene, Shape};
use sim::{Camera, CameraModel, GyroModel, IntrinsicsError, LaserMount, StripeModel, Trajectory, look_at};

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
    pub volume: VolumeDims,
    pub shapes: Vec<Shape>,
    pub laser: LaserMount,
    pub plane: PlaneSource,
    pub trajectory: Trajectory,
    /// σ of the ChArUco corner positions (px).
    pub corner_noise_px: f64,
    /// Rolling-shutter readout, first to last row (ms); 0 = global shutter.
    pub rolling_shutter_ms: f64,
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
            volume: VolumeDims::default(),
            shapes: vec![Shape::ball_on_board(144.0, 144.0, 25.0), Shape::block_on_board(60.0, 190.0, 40.0, 30.0, 20.0)],
            laser: LaserMount::default(),
            plane: PlaneSource::Truth,
            trajectory: Trajectory::default(),
            corner_noise_px: 0.0,
            rolling_shutter_ms: 0.0,
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
    let no_correction = FusionCfg { rolling_shutter: MotionSource::Off, ..FusionCfg::default() };
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
    let brisk = BenchCfg { rolling_shutter_ms: 20.0, trajectory: Trajectory { sweep_hz: 0.75, ..Trajectory::default() }, ..noisy };
    out.extend([
        BenchCfg { fusion: no_correction, ..brisk.named("brisk, RS 20 ms, uncorrected") },
        BenchCfg { fusion: FusionCfg { rolling_shutter: MotionSource::Poses, ..FusionCfg::default() }, ..brisk.named("brisk, RS corrected (poses)") },
        BenchCfg { gyro: Some(GyroModel::default()), ..brisk.named("brisk, RS corrected (gyro)") },
    ]);
    out
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
    let aim = [
        0.5 * b.width_mm() + 80.0 * (frac(i * 0.569_840) - 0.5),
        0.5 * b.height_mm() + 80.0 * (frac(i * 0.414_214) - 0.5),
        0.0,
    ];
    let out = [el.cos() * az.cos(), el.cos() * az.sin(), -el.sin()];
    let side = [-az.sin(), az.cos(), 0.0];
    look_at([aim[0] + dist * out[0], aim[1] + dist * out[1], aim[2] + dist * out[2]], aim, side)
}

fn solve_corners(board: &BoardDims, k: &Intrinsics, corners: &[(u32, [f64; 2])]) -> Option<Pose> {
    if corners.len() < reconstruct::MIN_CORNERS {
        return None;
    }
    let ids: Vec<(u32, V3)> = board.corners().collect();
    let (obj, px): (Vec<V3>, Vec<[f64; 2]>) = corners.iter().map(|&(id, q)| (ids[id as usize].1, q)).unzip();
    pnp::solve_planar(k, &obj, &px).map(|(p, _)| p)
}

/// Replay the app's laser-plane calibration on simulated board-only views.
pub fn calibrate_plane(cfg: &BenchCfg, frames: usize, min_lines: usize) -> Result<PlaneFit, String> {
    let scene = Scene { board: cfg.board, shapes: Vec::new() };
    let k_est: Intrinsics = cfg.intrinsics_error.apply(cfg.camera).into();
    let vol = cfg.volume.volume(&cfg.board);
    let mut cam = Camera::new(&scene, cfg.camera.into(), cfg.laser, cfg.stripe, cfg.corner_noise_px, 0.0, cfg.seed ^ 0xCA1B);
    let mut cal = PlaneCalibrator::new(min_lines.max(3));
    let mut solved_at = 0;
    for i in 0..frames {
        let truth = calibration_pose(&cfg.board, i);
        let obs = cam.observe(&|_| truth, 0.0);
        let Some(pose) = solve_corners(&cfg.board, &k_est, &obs.corners) else { continue };
        if let Some(s) = stripe::on_board(&obs.laser_px, &k_est, &pose, &vol) {
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
pub fn simulate(cfg: &BenchCfg) -> Result<Recording, String> {
    let scene = Scene { board: cfg.board, shapes: cfg.shapes.clone() };
    let (tn, td) = cfg.laser.plane();
    let used = match cfg.plane {
        PlaneSource::Truth => PlaneRec { n: tn, d: td },
        PlaneSource::Perturbed { tilt_deg, offset_mm } => {
            let n = mat_vec(&rodrigues([0.0, tilt_deg.to_radians(), 0.0]), tn);
            // keep the emitter on the plane, then shift along the normal
            PlaneRec { n, d: dot(n, cfg.laser.origin()) + offset_mm }
        }
        PlaneSource::Calibrated { frames, min_lines } => {
            let fit = calibrate_plane(cfg, frames, min_lines)?;
            PlaneRec { n: fit.n, d: fit.d }
        }
    };
    let readout_s = cfg.rolling_shutter_ms * 1e-3;
    let mut cam = Camera::new(&scene, cfg.camera.into(), cfg.laser, cfg.stripe, cfg.corner_noise_px, readout_s, cfg.seed);
    let traj = |t: f64| cfg.trajectory.pose(t);
    let mut frames = Vec::new();
    let mut poses = Vec::new();
    for t in cfg.trajectory.frame_times() {
        let obs = cam.observe(&traj, t);
        let frame = FrameRec {
            ts_ns: (t * 1e9).round() as i64,
            readout_ns: (readout_s * 1e9).round() as i64,
            exposure_ns: 0,
            pose: None,
            corners: obs.corners.iter().map(|&(id, q)| CornerRec { id, u: q[0], v: q[1] }).collect(),
            laser_px: obs.laser_px,
        };
        poses.push(traj(frame.clock(cfg.camera.height).mid_ns() * 1e-9).into());
        frames.push(frame);
    }
    let gyro = cfg.gyro.map_or_else(Vec::new, |g| {
        let end = cfg.trajectory.duration_s + readout_s + 0.1;
        g.sample(&cfg.trajectory, end, &mut Rng::new(cfg.seed ^ 0x6E70))
    });
    Ok(Recording {
        version: Recording::VERSION,
        source: format!("synthetic: {}", cfg.name),
        board: cfg.board,
        volume: cfg.volume,
        intrinsics: cfg.intrinsics_error.apply(cfg.camera),
        laser_plane: used,
        frames,
        cam_from_imu: cfg.gyro.map(|g| g.reported_mount()),
        gyro,
        truth: Some(Truth {
            scene,
            intrinsics: cfg.camera,
            laser_plane: PlaneRec { n: tn, d: td },
            poses,
            gyro: cfg.gyro.map(|g| g.calib()),
        }),
    })
}

pub struct Outcome {
    pub recording: Recording,
    pub reconstruction: Reconstruction,
    pub report: Report,
}

/// Simulate, reconstruct and score one scenario.
pub fn run(cfg: &BenchCfg) -> Result<Outcome, String> {
    let t0 = Instant::now();
    let recording = simulate(cfg)?;
    let reconstruction = reconstruct::reconstruct(&recording, PoseFrom::Auto, &cfg.fusion);
    let mut report = metrics::evaluate(&cfg.name, &recording, &reconstruction);
    report.runtime_s = t0.elapsed().as_secs_f64();
    Ok(Outcome { recording, reconstruction, report })
}

#[cfg(test)]
mod tests {
    use super::*;

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
    fn simulated_plane_calibration_converges() {
        let cfg = BenchCfg { corner_noise_px: 0.2, stripe: StripeModel::Analytic { noise_px: 0.3 }, ..BenchCfg::default() };
        let fit = calibrate_plane(&cfg, 120, 10).unwrap();
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
        let off = BenchCfg { fusion: FusionCfg { rolling_shutter: MotionSource::Off, ..FusionCfg::default() }, ..brisk.clone() };
        let (raw, corrected) = (run(&off).unwrap().report, run(&brisk).unwrap().report);
        let (a, b) = (raw.pose_pos_mm.unwrap().rms, corrected.pose_pos_mm.unwrap().rms);
        assert!(b < 0.6 * a, "pose error {a:.2} mm uncorrected, {b:.2} mm corrected");
        assert!(corrected.corrected_frames > corrected.frames * 9 / 10);
    }

    #[test]
    fn gyro_self_calibration_finds_the_simulated_gyro() {
        let cfg = BenchCfg {
            corner_noise_px: 0.2,
            rolling_shutter_ms: 20.0,
            gyro: Some(GyroModel::default()),
            trajectory: Trajectory { duration_s: 6.0, azimuth_span_deg: 108.0, ..Trajectory::default() },
            ..BenchCfg::default()
        };
        let g = run(&cfg).unwrap().report.gyro.expect("the gyro calibrates");
        // the smooth orbit turns about one axis: the reported mounting (0.5° off) is kept
        assert!(g.rotation_err_deg.unwrap() < 1.0, "{g:?}");
        // the offset is the one that makes gyro and board agree; it also takes
        // up the few ms by which the not yet gyro-corrected poses lag
        assert!(g.offset_err_ms.unwrap().abs() < 10.0, "{g:?}");
        assert!(g.from_frame.is_some_and(|f| f < 90), "{g:?}");
    }

    #[test]
    fn recording_round_trips_through_json() {
        let mut cfg = short(BenchCfg::default());
        cfg.trajectory.duration_s = 0.2;
        let rec = simulate(&cfg).unwrap();
        let json = serde_json::to_string(&rec).unwrap();
        let back: Recording = serde_json::from_str(&json).unwrap();
        // serde_json may round the last bit of a float: compare the content
        assert_eq!(rec.frames.len(), back.frames.len());
        assert_eq!(rec.truth.as_ref().unwrap().scene, back.truth.as_ref().unwrap().scene);
        for (a, b) in rec.frames.iter().zip(&back.frames) {
            assert_eq!(a.corners.len(), b.corners.len());
            assert_eq!(a.laser_px, b.laser_px);
            for (p, q) in a.corners.iter().zip(&b.corners) {
                assert!(p.id == q.id && (p.u - q.u).abs() < 1e-9 && (p.v - q.v).abs() < 1e-9);
            }
        }
    }
}
