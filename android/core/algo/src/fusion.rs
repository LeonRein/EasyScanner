//! Board pose + gyroscope: what the engine (and the benchmark, which replays
//! the engine's steps) asks about the camera's motion around each frame.
//!
//! [`PoseFusion`] keeps the recent gyro samples and board poses, calibrates
//! the gyro against the board on the fly ([`crate::imu`]), and answers per
//! frame:
//!
//! - the rolling-shutter [`Correction`] for its corners and stripe,
//! - the rotation the camera should have now, for IPPE's branch choice
//!   ([`crate::ippe`]) — also right after the board was out of view,
//! - the turn rate for the scan gate, available from the first frame.
//!
//! The camera position is never propagated from the IMU: the accelerometer,
//! integrated twice, drifts by about a millimetre within a few frames, well
//! above the board pose's own error.

use std::collections::VecDeque;

use serde::{Deserialize, Serialize};

use crate::geom::{M3, Pose, V3, add, mat_mul, mat_vec, norm, rotation_vector, scale, sub, transpose};
use crate::imu::{GyroBuffer, GyroCalib, GyroCalibCfg, GyroCalibrator, GyroFit, GyroSample};
use crate::rolling::{Correction, FrameMotion, MotionSource, RowClock};

#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq)]
#[serde(default)]
pub struct FusionCfg {
    /// Use the gyroscope: calibrate it against the board, and take the turn
    /// rate, the rolling-shutter rotation and the expected IPPE branch from it.
    pub gyro: bool,
    /// Where the rolling-shutter correction takes the camera motion from.
    pub rolling_shutter: MotionSource,
    pub calibration: GyroCalibCfg,
}

impl Default for FusionCfg {
    fn default() -> Self {
        FusionCfg { gyro: true, rolling_shutter: MotionSource::Gyro, calibration: GyroCalibCfg::default() }
    }
}

/// How far the gyro is. The numeric codes go into the frame header.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GyroState {
    /// No samples (no gyroscope, or the fusion is off).
    Absent = 0,
    /// Samples arrive; the calibration has not converged yet.
    Calibrating = 1,
    Calibrated = 2,
}

/// See the module docs.
#[derive(Clone, Debug)]
pub struct PoseFusion {
    cfg: FusionCfg,
    gyro: GyroBuffer,
    calibrator: GyroCalibrator,
    fit: Option<GyroFit>,
    /// The IMU→camera rotation the phone reports, if any.
    nominal_mount: Option<M3>,
    /// Camera time of the last calibration attempt (ns).
    last_attempt_ns: Option<f64>,
    /// Recent board poses at their frames' reference instants (camera
    /// clock, ns), newest last.
    history: VecDeque<(f64, Pose)>,
}

impl PoseFusion {
    /// Gyro history kept: the calibration's pose window plus slack.
    const GYRO_SPAN_NS: i64 = 12_000_000_000;
    /// Board poses the calibration fits to (~8 s at 30 fps).
    const CALIBRATION_POSES: usize = 240;
    /// Velocity from the poses of about this window: long enough that the
    /// pose jitter does not read as motion, short enough to follow the hand.
    const VELOCITY_WINDOW_NS: f64 = 100_000_000.0;
    /// Pose-derived motion older than this is not extrapolated.
    const MAX_MOTION_AGE_NS: f64 = 120_000_000.0;
    const HISTORY: usize = 8;
    /// The gyro carries the last pose's rotation across a board loss of at
    /// most this long (the residual bias drifts it by ~0.01°/s).
    const MAX_PREDICTION_NS: f64 = 3_000_000_000.0;

    pub fn new(cfg: FusionCfg) -> Self {
        PoseFusion {
            cfg,
            gyro: GyroBuffer::new(Self::GYRO_SPAN_NS),
            calibrator: GyroCalibrator::new(Self::CALIBRATION_POSES),
            fit: None,
            nominal_mount: None,
            last_attempt_ns: None,
            history: VecDeque::with_capacity(Self::HISTORY),
        }
    }

    pub fn cfg(&self) -> &FusionCfg {
        &self.cfg
    }

    /// New settings; a calibration already found is kept.
    pub fn set_cfg(&mut self, cfg: FusionCfg) {
        self.cfg = cfg;
    }

    /// The IMU→camera rotation the phone reports for the camera in use: the
    /// calibration falls back on it while the camera turns about one axis.
    /// A different camera invalidates the calibration.
    pub fn set_nominal_mount(&mut self, cam_from_imu: Option<M3>) {
        if cam_from_imu != self.nominal_mount {
            self.nominal_mount = cam_from_imu;
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
    }

    pub fn push_gyro(&mut self, s: GyroSample) {
        self.gyro.push(s);
    }

    /// The gyro calibration in use, if it has converged.
    pub fn gyro_fit(&self) -> Option<&GyroFit> {
        self.fit.as_ref()
    }

    pub fn gyro_state(&self) -> GyroState {
        if !self.cfg.gyro || self.gyro.is_empty() {
            GyroState::Absent
        } else if self.fit.is_some() {
            GyroState::Calibrated
        } else {
            GyroState::Calibrating
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

    /// Recent poses, oldest first, if the newest is recent enough to
    /// extrapolate to `now_ns` and they span enough time to read a motion
    /// from.
    fn recent_poses(&self, now_ns: f64) -> Option<Vec<(f64, Pose)>> {
        let &(newest, _) = self.history.back()?;
        if now_ns - newest > Self::MAX_MOTION_AGE_NS {
            return None;
        }
        let poses: Vec<(f64, Pose)> = self.history.iter().filter(|(t, _)| newest - t <= Self::VELOCITY_WINDOW_NS).copied().collect();
        (newest - poses[0].0 >= 0.5 * Self::VELOCITY_WINDOW_NS).then_some(poses)
    }

    /// The camera motion during this frame, as far as it is known.
    fn motion(&self, clock: &RowClock) -> Option<FrameMotion> {
        let now = clock.mid_ns();
        let poses = self.recent_poses(now)?;
        let (_, newest) = *poses.last()?;
        let centres: Vec<(f64, V3)> = poses.iter().map(|(t, p)| ((t - now) * 1e-9, p.camera_position())).collect();
        let velocity = mat_vec(&newest.r, slope(&centres)?);
        let pose_rate = || {
            let ((ta, a), (tb, b)) = (poses[0], poses[poses.len() - 1]);
            let turn = rotation_vector(&mat_mul(&b.r, &transpose(&a.r)));
            scale(turn, -1e9 / (tb - ta))
        };
        let rate = match self.cfg.rolling_shutter {
            MotionSource::Off => return None,
            MotionSource::Poses => pose_rate(),
            MotionSource::Gyro => self.gyro_rate(clock).unwrap_or_else(pose_rate),
        };
        Some(FrameMotion { rate, velocity })
    }

    /// The rolling-shutter correction for this frame, `None` for a global
    /// shutter, with the correction off, or while the motion is unknown.
    pub fn correction(&self, clock: &RowClock) -> Option<Correction> {
        if clock.readout_ns <= 0 {
            return None;
        }
        self.motion(clock).map(|motion| Correction { clock: *clock, motion })
    }

    /// The board→camera rotation the camera should have at this frame's
    /// reference instant: the last pose turned on by the gyro.
    pub fn expected_rotation(&self, clock: &RowClock) -> Option<M3> {
        let &(t, last) = self.history.back()?;
        let now = clock.mid_ns();
        if now - t > Self::MAX_PREDICTION_NS {
            return None;
        }
        let turn = self.calib()?.rotation(&self.gyro, t, now)?;
        Some(mat_mul(&turn, &last.r))
    }

    /// Turn rate (deg/s) over the frame, from the gyro.
    pub fn turn_rate_deg_s(&self, clock: &RowClock) -> Option<f64> {
        self.gyro_rate(clock).map(|w| norm(w).to_degrees())
    }

    /// The frame's pose, if it had one. `trusted` poses (good fit, enough
    /// corners, no ambiguity) also feed the gyro calibration.
    pub fn observe(&mut self, clock: &RowClock, pose: Option<&Pose>, trusted: bool) {
        let now = clock.mid_ns();
        if self.history.back().is_some_and(|&(t, _)| now <= t) {
            // a restarted camera clock
            self.restart();
        }
        let Some(pose) = pose else { return };
        if self.history.len() == Self::HISTORY {
            self.history.pop_front();
        }
        self.history.push_back((now, *pose));
        if !(self.cfg.gyro && trusted) {
            return;
        }
        self.calibrator.add(now as i64, pose.r);
        let interval = self.cfg.calibration.interval_s * 1e9;
        if self.last_attempt_ns.is_none_or(|t| now - t >= interval) {
            self.last_attempt_ns = Some(now);
            if let Some(fit) = self.calibrator.solve(&self.gyro, &self.cfg.calibration, self.nominal_mount.as_ref()) {
                self.fit = Some(fit);
            }
        }
    }
}

/// Slope of the least-squares line through `(τ, x)` samples (τ in
/// seconds): the mean velocity over the samples.
///
/// A line, not a quadratic extrapolated to now: the poses it is fitted to
/// were themselves corrected with an earlier velocity, and the larger gain
/// of an extrapolation turns that loop unstable.
fn slope(samples: &[(f64, V3)]) -> Option<V3> {
    let n = samples.len() as f64;
    let t_mean = samples.iter().map(|s| s.0).sum::<f64>() / n;
    let x_mean = scale(samples.iter().fold([0.0; 3], |acc, s| add(acc, s.1)), 1.0 / n);
    let var: f64 = samples.iter().map(|s| (s.0 - t_mean).powi(2)).sum();
    if samples.len() < 2 || var <= 0.0 {
        return None;
    }
    let cov = samples.iter().fold([0.0; 3], |acc, s| add(acc, scale(sub(s.1, x_mean), s.0 - t_mean)));
    Some(scale(cov, 1.0 / var))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn slope_of_a_line_and_of_too_few_samples() {
        let pts: Vec<(f64, V3)> = (0..5).map(|i| -(i as f64) / 30.0).map(|t| (t, [5.0 + 30.0 * t, -2.0 * t, 2.0])).collect();
        let v = slope(&pts).unwrap();
        assert!((v[0] - 30.0).abs() < 1e-9 && (v[1] + 2.0).abs() < 1e-9 && v[2].abs() < 1e-9, "{v:?}");
        assert!(slope(&pts[..1]).is_none());
    }
}
