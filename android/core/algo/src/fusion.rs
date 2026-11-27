//! Board pose + gyroscope: what the engine (and the benchmark, which replays
//! the engine's steps) asks about the camera's motion around each frame.
//!
//! [`PoseFusion`] keeps the recent gyro samples and trusted board poses,
//! calibrates the gyro against the board on the fly ([`crate::imu`]), and
//! answers per frame:
//!
//! - the rolling-shutter [`Correction`] for its corners and stripe,
//! - the rotation the camera should have now, for IPPE's branch choice
//!   ([`crate::ippe`]) — also right after the board was out of view,
//! - the turn rate for the scan gate, from the first gyro sample on.
//!
//! The camera position is never propagated from the IMU: the accelerometer,
//! integrated twice, drifts by about a millimetre within a few frames, well
//! above the board pose's own error.

use std::collections::VecDeque;

use serde::{Deserialize, Serialize};

use crate::geom::{M3, Pose, V3, add, mat_mul, mat_vec, norm, rotation_vector, scale, transpose};
use crate::imu::{GyroBuffer, GyroCalib, GyroCalibCfg, GyroCalibrator, GyroFit, GyroSample, MountSource};
use crate::rolling::{Correction, FrameMotion, RowClock};

/// The fusion's settings (config key `fusion`).
#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq)]
#[serde(default)]
pub struct FusionCfg {
    /// Use the gyroscope: calibrate it against the board, and take the turn
    /// rate, the rolling-shutter rotation and the expected IPPE branch from it.
    pub gyro: bool,
    /// Correct the rolling shutter. The rotation comes from the calibrated
    /// gyro, or from the board poses without one; the velocity always comes
    /// from the board poses.
    pub rolling_shutter: bool,
    /// Acceptance limits of the gyro self-calibration.
    pub calibration: GyroCalibCfg,
}

impl Default for FusionCfg {
    fn default() -> Self {
        FusionCfg { gyro: true, rolling_shutter: true, calibration: GyroCalibCfg::default() }
    }
}

/// How far the gyro is. The numeric codes go into the frame header.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GyroState {
    /// No samples (no gyroscope, or the fusion is off).
    Absent = 0,
    /// Samples arrive; the clock offset is not known yet.
    Calibrating = 1,
    /// Calibrated with the mount the platform defines.
    PlatformMount = 2,
    /// Calibrated with a mount fitted to the board poses.
    FittedMount = 3,
}

/// See the module docs.
#[derive(Clone, Debug)]
pub struct PoseFusion {
    cfg: FusionCfg,
    gyro: GyroBuffer,
    calibrator: GyroCalibrator,
    fit: Option<GyroFit>,
    /// The IMU→camera rotation the platform defines, if known.
    platform_mount: Option<M3>,
    /// Camera time of the last calibration attempt (ns).
    last_attempt_ns: Option<f64>,
    /// Reference instant of the last frame (camera clock, ns).
    last_frame_ns: Option<f64>,
    /// Recent trusted board poses at their frames' reference instants
    /// (camera clock, ns), newest last.
    history: VecDeque<(f64, Pose)>,
}

impl PoseFusion {
    /// Gyro history kept: the calibration's pose window plus slack.
    const GYRO_SPAN_NS: f64 = 12e9;
    /// Board poses the calibration fits to (~8 s at 30 fps).
    const CALIBRATION_POSES: usize = 240;
    /// Velocity from the poses of about this window: long enough that the
    /// pose jitter does not read as motion, short enough to follow the hand.
    const VELOCITY_WINDOW_NS: f64 = 100e6;
    /// Pose-derived motion older than this is not extrapolated.
    const MAX_MOTION_AGE_NS: f64 = 120e6;
    /// A hand-held scan moves well below this (mm/s); a faster camera
    /// centre is a pose error, not a motion to correct.
    const MAX_SPEED_MM_S: f64 = 1500.0;
    const HISTORY: usize = 8;
    /// The gyro carries the last pose's rotation across a board loss of at
    /// most this long (the residual bias, ≤ 0.1°/s, turns it by ≤ 0.3°).
    const MAX_PREDICTION_NS: f64 = 3e9;

    /// No samples, poses or calibration yet.
    pub fn new(cfg: FusionCfg) -> Self {
        PoseFusion {
            cfg,
            gyro: GyroBuffer::new(Self::GYRO_SPAN_NS),
            calibrator: GyroCalibrator::new(Self::CALIBRATION_POSES),
            fit: None,
            platform_mount: None,
            last_attempt_ns: None,
            last_frame_ns: None,
            history: VecDeque::with_capacity(Self::HISTORY),
        }
    }

    /// New settings; a calibration already found is kept.
    pub fn set_cfg(&mut self, cfg: FusionCfg) {
        self.cfg = cfg;
    }

    /// The IMU→camera rotation the platform defines for the camera in use
    /// (Android: from `SENSOR_ORIENTATION`). The calibration starts from it
    /// and refines it once the camera turns about two axes. A different
    /// camera invalidates the calibration.
    pub fn set_platform_mount(&mut self, cam_from_imu: Option<M3>) {
        if cam_from_imu != self.platform_mount {
            self.platform_mount = cam_from_imu;
            self.fit = None;
        }
    }

    /// The camera restarted (its clock may have too): forget the samples and
    /// poses, keep the calibration — the gyro is still mounted the same way.
    pub fn restart(&mut self) {
        self.gyro.clear();
        self.calibrator.clear();
        self.history.clear();
        self.last_attempt_ns = None;
        self.last_frame_ns = None;
    }

    /// A gyro sample, in the order they arrive.
    pub fn push_gyro(&mut self, s: GyroSample) {
        self.gyro.push(s);
    }

    /// The gyro calibration in use, if it has converged.
    pub fn gyro_fit(&self) -> Option<&GyroFit> {
        self.fit.as_ref()
    }

    /// How far the gyro is, for the HUD.
    pub fn gyro_state(&self) -> GyroState {
        if !self.cfg.gyro || self.gyro.is_empty() {
            return GyroState::Absent;
        }
        match self.fit.map(|f| f.mount) {
            None => GyroState::Calibrating,
            Some(MountSource::Platform) => GyroState::PlatformMount,
            Some(MountSource::Fitted { .. }) => GyroState::FittedMount,
        }
    }

    fn calib(&self) -> Option<&GyroCalib> {
        self.cfg.gyro.then_some(self.fit.as_ref()).flatten().map(|f| &f.calib)
    }

    /// The camera's angular rate over the frame's readout, from the gyro.
    fn gyro_rate(&self, clock: &RowClock) -> Option<V3> {
        let (t0, t1) = clock.span_ns();
        self.calib()?.rate(&self.gyro, t0, t1)
    }

    /// The camera motion during this frame, as far as it is known: the
    /// velocity of the recent trusted poses, if they are recent and span
    /// enough time to read it, and the rate from the gyro or those poses.
    fn motion(&self, clock: &RowClock) -> Option<FrameMotion> {
        let now = clock.mid_ns();
        let &(newest_ns, newest) = self.history.back()?;
        if now - newest_ns > Self::MAX_MOTION_AGE_NS {
            return None;
        }
        let window = || self.history.iter().filter(move |(t, _)| newest_ns - t <= Self::VELOCITY_WINDOW_NS);
        let &(oldest_ns, oldest) = window().next()?;
        if newest_ns - oldest_ns < 0.5 * Self::VELOCITY_WINDOW_NS {
            return None;
        }
        let centres = window().map(|(t, p)| ((t - now) * 1e-9, p.camera_position()));
        let velocity = mat_vec(&newest.r, slope(centres)?);
        if norm(velocity) > Self::MAX_SPEED_MM_S {
            return None;
        }
        let rate = self.gyro_rate(clock).unwrap_or_else(|| {
            let turn = rotation_vector(&mat_mul(&newest.r, &transpose(&oldest.r)));
            scale(turn, -1e9 / (newest_ns - oldest_ns))
        });
        Some(FrameMotion { rate, velocity })
    }

    /// The rolling-shutter correction for this frame, `None` for a global
    /// shutter, with the correction off, or while the motion is unknown.
    pub fn correction(&self, clock: &RowClock) -> Option<Correction> {
        if !self.cfg.rolling_shutter || clock.readout_ns <= 0 {
            return None;
        }
        self.motion(clock).map(|motion| Correction { clock: *clock, motion })
    }

    /// The board→camera rotation the camera should have at this frame's
    /// reference instant: the last trusted pose turned on by the gyro.
    pub fn expected_rotation(&self, clock: &RowClock) -> Option<M3> {
        let &(t, last) = self.history.back()?;
        let now = clock.mid_ns();
        if now - t > Self::MAX_PREDICTION_NS {
            return None;
        }
        let turn = self.calib()?.rotation(&self.gyro, t, now)?;
        Some(mat_mul(&turn, &last.r))
    }

    /// Turn rate (deg/s) over the frame's readout, from the gyro. The norm
    /// does not depend on the mount, so it is there before the calibration
    /// (with the clocks taken as equal until the offset is known).
    pub fn turn_rate_deg_s(&self, clock: &RowClock) -> Option<f64> {
        if !self.cfg.gyro {
            return None;
        }
        let offset = self.fit.map_or(0.0, |f| f.calib.offset_ns);
        let (t0, t1) = clock.span_ns();
        self.gyro.mean_rate(t0 + offset, t1 + offset).map(|w| norm(w).to_degrees())
    }

    /// Called once per frame with the frame's pose if it is trusted (good
    /// fit, enough corners, no ambiguity): only trusted poses feed the
    /// velocity, the expected rotation and the gyro calibration.
    pub fn observe(&mut self, clock: &RowClock, trusted_pose: Option<&Pose>) {
        let now = clock.mid_ns();
        if self.last_frame_ns.is_some_and(|t| now <= t) {
            // a restarted camera clock
            self.restart();
        }
        self.last_frame_ns = Some(now);
        let Some(pose) = trusted_pose else { return };
        if self.history.len() == Self::HISTORY {
            self.history.pop_front();
        }
        self.history.push_back((now, *pose));
        if !self.cfg.gyro {
            return;
        }
        self.calibrator.add(now, pose.r);
        let interval = self.cfg.calibration.interval_s * 1e9;
        if self.last_attempt_ns.is_none_or(|t| now - t >= interval) {
            self.last_attempt_ns = Some(now);
            let solved = self.calibrator.solve(&self.gyro, &self.cfg.calibration, self.platform_mount.as_ref(), self.fit.as_ref());
            if let Some(fit) = solved {
                self.fit = Some(fit);
            }
        }
    }
}

/// Slope of the least-squares line through `(τ, x)` samples (τ in
/// seconds): the mean velocity over the samples. `None` for fewer than two
/// distinct instants.
///
/// A line, not a quadratic extrapolated to now: the poses it is fitted to
/// were themselves corrected with an earlier velocity, and the larger gain
/// of an extrapolation turns that loop unstable.
fn slope(samples: impl Iterator<Item = (f64, V3)>) -> Option<V3> {
    let (mut n, mut st, mut stt, mut sx, mut stx) = (0.0, 0.0, 0.0, [0.0; 3], [0.0; 3]);
    for (t, x) in samples {
        n += 1.0;
        st += t;
        stt += t * t;
        sx = add(sx, x);
        stx = add(stx, scale(x, t));
    }
    // τ are a fraction of a second around now: the one-pass sums are exact enough
    let var = stt - st * st / n;
    if n < 2.0 || var <= 0.0 {
        return None;
    }
    Some(std::array::from_fn(|i| (stx[i] - st * sx[i] / n) / var))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geom::rodrigues;

    const FRAME_NS: i64 = 33_333_333;

    fn clock(i: i64) -> RowClock {
        RowClock { start_ns: 1_000_000_000 + i * FRAME_NS, readout_ns: 20_000_000, exposure_ns: 0, height: 960.0 }
    }

    /// A camera 400 mm from the board, its centre moving along x at `speed` mm/s.
    fn pose_at(t_ns: f64, speed: f64) -> Pose {
        let r = rodrigues([3.0, 0.0, 0.0]);
        let centre = [speed * t_ns * 1e-9, 0.0, -400.0];
        Pose { r, t: scale(mat_vec(&r, centre), -1.0) }
    }

    #[test]
    fn turn_rate_needs_no_calibration() {
        let mut f = PoseFusion::new(FusionCfg::default());
        for k in 0..400 {
            f.push_gyro(GyroSample { t_ns: 900_000_000 + k * 5_000_000, rate: [0.0, 0.0, 0.5] });
        }
        assert_eq!(f.gyro_state(), GyroState::Calibrating);
        let rate = f.turn_rate_deg_s(&clock(3)).unwrap();
        assert!((rate - 0.5f64.to_degrees()).abs() < 1e-9, "{rate}");
    }

    #[test]
    fn only_trusted_plausible_motion_is_corrected() {
        let mut f = PoseFusion::new(FusionCfg { gyro: false, ..FusionCfg::default() });
        for i in 0..6 {
            let c = clock(i);
            f.observe(&c, Some(&pose_at(c.mid_ns(), 200.0)));
        }
        let corr = f.correction(&clock(6)).expect("a steady 200 mm/s is corrected");
        assert!((norm(corr.motion.velocity) - 200.0).abs() < 1e-6, "{corr:?}");
        // an untrusted frame (say, a flipped IPPE branch) is not read as motion
        f.observe(&clock(6), None);
        assert!(f.correction(&clock(7)).is_some());
        // a jump by decimetres (a mirrored IPPE branch that passed as
        // trusted) is a pose error, not a hand
        let c = clock(7);
        f.observe(&c, Some(&pose_at(c.mid_ns(), 200.0)));
        let mut jumped = pose_at(clock(8).mid_ns(), 200.0);
        jumped.t[0] += 200.0;
        f.observe(&clock(8), Some(&jumped));
        assert!(f.correction(&clock(9)).is_none());
    }

    #[test]
    fn slope_of_a_line_and_of_too_few_samples() {
        let pts: Vec<(f64, V3)> = (0..5).map(|i| -f64::from(i) / 30.0).map(|t| (t, [5.0 + 30.0 * t, -2.0 * t, 2.0])).collect();
        let v = slope(pts.iter().copied()).unwrap();
        assert!((v[0] - 30.0).abs() < 1e-9 && (v[1] + 2.0).abs() < 1e-9 && v[2].abs() < 1e-9, "{v:?}");
        assert!(slope(pts[..1].iter().copied()).is_none());
    }
}
