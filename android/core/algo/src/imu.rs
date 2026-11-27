//! The phone's gyroscope: a short, pre-integrated buffer of its samples, its
//! calibration against the camera, and the online self-calibration that
//! finds it.
//!
//! The gyro measures the body's angular rate in its own frame and clock. To
//! use it for the camera two things are needed:
//!
//! - the **IMU→camera rotation** ("mount"). The platform defines it: Android
//!   fixes the sensor axes to the device and Camera2 reports the image's
//!   orientation (`SENSOR_ORIENTATION`), which gives the mount up to the
//!   assembly tolerance of a degree or less. Whenever the camera turns about
//!   two axes, [`GyroCalibrator`] fits the mount itself from board poses
//!   and keeps the fit once it is known better than that.
//! - the **clock offset** between gyro and camera timestamps: a few
//!   milliseconds of latency, fitted from board poses.
//!
//! There is no rate bias in the model: Android's `TYPE_GYROSCOPE` is bias
//! compensated by the platform, and what remains (≤ 0.1°/s) moves a 100 ms
//! pose increment by ≤ 0.01° — far below the poses' own noise — and the
//! 3 s bridged by [`crate::fusion::PoseFusion::expected_rotation`] by
//! ≤ 0.3°, far below the separation of IPPE's branches.
//!
//! Rotation convention: a world→body rotation `R(t)` (like a board→camera
//! pose) evolves as `Ṙ = −[ω]ₓ·R` for the body rate `ω`, so the rotation
//! *increment* from `t0` to `t1` is `Δ = R(t1)·R(t0)ᵀ ≈ exp(−ω·(t1 − t0))`.
//!
//! Time: sample timestamps arrive as `i64` ns from the sensor; every instant
//! inside is an `f64` in ns.

use std::collections::VecDeque;

use serde::{Deserialize, Serialize};

use crate::geom::{
    IDENTITY, M3, V3, add, dot, mat_add, mat_mul, mat_vec, nearest_rotation, norm, normalize, orthonormalize, outer, rodrigues,
    rotation_distance, rotation_vector, scale, sub, sym3_eigen, transpose,
};

/// One gyroscope reading.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct GyroSample {
    /// Gyro clock (ns).
    pub t_ns: i64,
    /// Angular rate (rad/s), IMU frame.
    pub rate: V3,
}

/// A sample with the IMU's attitude integrated up to it.
#[derive(Clone, Copy, Debug)]
struct Knot {
    t_ns: f64,
    rate: V3,
    /// `A(t)`: the IMU's rotation from a fixed but arbitrary frame, so that
    /// the increment between two instants is `A(t1)·A(t0)ᵀ`.
    attitude: M3,
    /// Dropouts between the first sample of the session and this one: two
    /// instants are connected by samples iff their knots count the same.
    dropouts: u64,
}

/// The last few seconds of gyro samples, integrated as they arrive: a
/// rotation increment over any interval is two lookups and a product,
/// however long the interval.
#[derive(Clone, Debug)]
pub struct GyroBuffer {
    knots: VecDeque<Knot>,
    span_ns: f64,
}

impl GyroBuffer {
    /// Samples further apart than this are a dropout, not a rate to
    /// interpolate across.
    pub const MAX_GAP_NS: f64 = 50e6;
    /// How far past the first or last sample a query may reach (the newest
    /// samples of a frame may still be in flight).
    pub const MAX_EXTRAPOLATION_NS: f64 = 20e6;

    /// A buffer that keeps `span_ns` of history.
    pub fn new(span_ns: f64) -> Self {
        GyroBuffer { knots: VecDeque::new(), span_ns }
    }

    /// Forget every sample (a restarted sensor).
    pub fn clear(&mut self) {
        self.knots.clear();
    }

    /// No samples have arrived (since the last clear).
    pub fn is_empty(&self) -> bool {
        self.knots.is_empty()
    }

    /// Append a sample. Out-of-order duplicates are dropped; a jump back by
    /// more than the span (a restarted sensor clock) starts over.
    pub fn push(&mut self, s: GyroSample) {
        let t_ns = s.t_ns as f64;
        if let Some(last) = self.knots.back()
            && t_ns <= last.t_ns
        {
            if last.t_ns - t_ns > self.span_ns {
                self.knots.clear();
            } else {
                return;
            }
        }
        let knot = match self.knots.back() {
            None => Knot { t_ns, rate: s.rate, attitude: IDENTITY, dropouts: 0 },
            Some(prev) => {
                let dt = t_ns - prev.t_ns;
                // trapezoid: the mean of the two rates over the step
                let step = rodrigues(scale(add(prev.rate, s.rate), -0.5 * dt * 1e-9));
                Knot {
                    t_ns,
                    rate: s.rate,
                    attitude: orthonormalize(&mat_mul(&step, &prev.attitude)),
                    dropouts: prev.dropouts + u64::from(dt > Self::MAX_GAP_NS),
                }
            }
        };
        self.knots.push_back(knot);
        let oldest = t_ns - self.span_ns;
        while self.knots.front().is_some_and(|k| k.t_ns < oldest) {
            self.knots.pop_front();
        }
    }

    /// Index of the last knot at or before `t_ns`, or 0 before the first.
    fn knot_before(&self, t_ns: f64) -> usize {
        self.knots.partition_point(|k| k.t_ns <= t_ns).saturating_sub(1)
    }

    /// The rate at `t_ns` interpolated linearly between the knot before it
    /// and the next, `frac` of the way from that knot to `t_ns` (held flat
    /// past either end).
    fn rate_towards(&self, i: usize, t_ns: f64, frac: f64) -> V3 {
        let k = &self.knots[i];
        match self.knots.get(i + 1) {
            Some(next) if t_ns > k.t_ns => add(k.rate, scale(sub(next.rate, k.rate), frac * (t_ns - k.t_ns) / (next.t_ns - k.t_ns))),
            _ => k.rate,
        }
    }

    /// The IMU attitude at `t_ns`: the knot before it, turned on by the mean
    /// of the interpolated rate over the rest of the way (its value halfway).
    fn attitude(&self, t_ns: f64) -> M3 {
        let i = self.knot_before(t_ns);
        let dt = t_ns - self.knots[i].t_ns;
        mat_mul(&rodrigues(scale(self.rate_towards(i, t_ns, 0.5), -dt * 1e-9)), &self.knots[i].attitude)
    }

    /// The samples cover `[t0, t1]` without a dropout.
    fn covers(&self, t0: f64, t1: f64) -> bool {
        let (Some(first), Some(last)) = (self.knots.front(), self.knots.back()) else { return false };
        if t0 < first.t_ns - Self::MAX_EXTRAPOLATION_NS || t1 > last.t_ns + Self::MAX_EXTRAPOLATION_NS {
            return false;
        }
        let after = self.knots.partition_point(|k| k.t_ns < t1).min(self.knots.len() - 1);
        self.knots[self.knot_before(t0)].dropouts == self.knots[after].dropouts
    }

    /// Rotation increment `Δ` of the IMU frame from `t0` to `t1` (gyro
    /// clock, ns; `t1 < t0` gives the inverse). `None` where the samples do
    /// not cover the interval.
    pub fn rotation(&self, t0: f64, t1: f64) -> Option<M3> {
        self.covers(t0.min(t1), t0.max(t1)).then(|| mat_mul(&self.attitude(t1), &transpose(&self.attitude(t0))))
    }

    /// Mean rate (rad/s, IMU frame) over `[t0, t1]` (gyro clock, ns); the
    /// interpolated rate at `t0` for an empty interval.
    pub fn mean_rate(&self, t0: f64, t1: f64) -> Option<V3> {
        let dt = (t1 - t0) * 1e-9;
        if dt > 0.0 {
            return self.rotation(t0, t1).map(|d| scale(rotation_vector(&d), -1.0 / dt));
        }
        self.covers(t0, t0).then(|| self.rate_towards(self.knot_before(t0), t0, 1.0))
    }
}

/// How the gyro relates to the camera.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct GyroCalib {
    /// IMU→camera rotation: `rate_cam = cam_from_imu · rate_imu`.
    pub cam_from_imu: M3,
    /// Gyro clock minus camera clock (ns): camera time `t` is gyro time
    /// `t + offset_ns`.
    pub offset_ns: f64,
}

impl GyroCalib {
    /// The camera's rotation increment from `t0` to `t1` (camera clock, ns).
    pub fn rotation(&self, buf: &GyroBuffer, t0: f64, t1: f64) -> Option<M3> {
        let d = buf.rotation(t0 + self.offset_ns, t1 + self.offset_ns)?;
        Some(mat_mul(&mat_mul(&self.cam_from_imu, &d), &transpose(&self.cam_from_imu)))
    }

    /// The camera's mean angular rate (rad/s, camera frame) over `[t0, t1]`.
    pub fn rate(&self, buf: &GyroBuffer, t0: f64, t1: f64) -> Option<V3> {
        let w = buf.mean_rate(t0 + self.offset_ns, t1 + self.offset_ns)?;
        Some(mat_vec(&self.cam_from_imu, w))
    }
}

/// Where a calibration's IMU→camera rotation comes from.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum MountSource {
    /// The platform's axis definitions (Android: `SENSOR_ORIENTATION`).
    Platform,
    /// Fitted to board poses that turned about two axes.
    Fitted {
        /// Standard error of the fitted rotation about its least determined
        /// axis (deg).
        sigma_deg: f64,
    },
}

/// Acceptance limits of the self-calibration.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct GyroCalibCfg {
    /// Pose pairs needed for a fit.
    pub min_pairs: usize,
    /// The time offset is searched within ± this (ms).
    pub max_offset_ms: f64,
    /// A fit is rejected when the RMS disagreement between camera and gyro
    /// rotation increments exceeds this fraction of their RMS turn: the
    /// gyro must explain the board's rotation.
    pub max_residual_rel: f64,
    /// The pose increments must turn by at least this much (RMS, deg) for
    /// the offset to show.
    pub min_turn_deg: f64,
    /// A fitted mount is used only when its standard error is below this
    /// (deg)…
    pub max_mount_sigma_deg: f64,
    /// … and it lies within this of the platform's (deg). Further off, the
    /// fit has latched onto something else than the mount.
    pub max_mount_deviation_deg: f64,
    /// Seconds between two calibration attempts.
    pub interval_s: f64,
}

impl Default for GyroCalibCfg {
    fn default() -> Self {
        GyroCalibCfg {
            min_pairs: 40,
            // Camera2 and SensorEvent share CLOCK_BOOTTIME on phones with a
            // REALTIME timestamp source; what remains is a few ms of latency
            max_offset_ms: 40.0,
            // the poses' own error grows with the turn rate (the rolling
            // shutter is only partly corrected before the gyro is): a
            // relative bound; a wrong clock or mount leaves far more
            max_residual_rel: 0.15,
            min_turn_deg: 0.5,
            // half the assembly tolerance of the platform's mount (~0.5°)
            max_mount_sigma_deg: 0.25,
            max_mount_deviation_deg: 5.0,
            interval_s: 2.0,
        }
    }
}

/// A calibration and how well it explains the poses it was fitted to.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GyroFit {
    /// The fitted calibration.
    pub calib: GyroCalib,
    /// Where its mount comes from.
    pub mount: MountSource,
    /// RMS of `|log Δ_cam − cam_from_imu · log Δ_imu|` (deg).
    pub residual_deg: f64,
    /// Pose pairs it was fitted to.
    pub pairs: usize,
}

/// Camera rotation increment between two board frames.
#[derive(Clone, Copy, Debug)]
struct PosePair {
    t0_ns: f64,
    t1_ns: f64,
    /// `log(R1·R0ᵀ)`, camera frame.
    cam: V3,
}

/// Collects board-pose rotations and fits a [`GyroCalib`] to them.
#[derive(Clone, Debug)]
pub struct GyroCalibrator {
    /// Camera time (ns) and board→camera rotation of recent trusted poses.
    poses: VecDeque<(f64, M3)>,
    cap: usize,
}

impl GyroCalibrator {
    /// Pose pairs are this far apart: long enough for the rotation to rise
    /// above the pose noise, short enough to stay well inside the buffer.
    const PAIR_NS: f64 = 100e6;
    /// Pairs further apart than this (a gap in the board track) are skipped.
    const MAX_PAIR_NS: f64 = 200e6;
    /// Offset grid step of the coarse search (ns). The fine search covers
    /// the half step either side of the coarse best in steps of 1/8.
    const OFFSET_STEP_NS: f64 = 2e6;
    const FINE_STEPS: i32 = 8;
    /// Once calibrated, the offset is only tracked within this of the last
    /// one: it moves by a millisecond or so, not across the whole range.
    const TRACK_NS: f64 = 4e6;

    /// Keeps the newest `cap` poses.
    pub fn new(cap: usize) -> Self {
        GyroCalibrator { poses: VecDeque::with_capacity(cap), cap }
    }

    /// Forget the poses (a restarted camera clock).
    pub fn clear(&mut self) {
        self.poses.clear();
    }

    /// A trusted board→camera rotation at camera time `t_ns`.
    pub fn add(&mut self, t_ns: f64, r: M3) {
        if self.poses.back().is_some_and(|&(t, _)| t_ns <= t) {
            self.poses.clear();
        }
        if self.poses.len() == self.cap {
            self.poses.pop_front();
        }
        self.poses.push_back((t_ns, r));
    }

    fn pairs(&self) -> Vec<PosePair> {
        let mut out = Vec::with_capacity(self.poses.len());
        let mut j = 0;
        for (i, &(t0_ns, r0)) in self.poses.iter().enumerate() {
            j = j.max(i + 1);
            while self.poses.get(j).is_some_and(|&(t, _)| t - t0_ns < Self::PAIR_NS) {
                j += 1;
            }
            let Some(&(t1_ns, r1)) = self.poses.get(j) else { break };
            if t1_ns - t0_ns <= Self::MAX_PAIR_NS {
                out.push(PosePair { t0_ns, t1_ns, cam: rotation_vector(&mat_mul(&r1, &transpose(&r0))) });
            }
        }
        out
    }

    /// Fit the calibration to the collected poses. `None` while there are
    /// too few pairs, the camera turned too little to show the offset, or
    /// the best fit still disagrees with the poses.
    ///
    /// The mount is the best one known — `previous` if that was fitted, else
    /// the `platform`'s — and only the offset is fitted, near `previous`'s
    /// if there is one. The mount is fitted
    /// too, and that fit replaces the known mount when it is more certain
    /// (see [`GyroCalibCfg`]): it is only once the camera turned about two
    /// axes. Without any known mount, only a fitted one will do.
    pub fn solve(&self, buf: &GyroBuffer, cfg: &GyroCalibCfg, platform: Option<&M3>, previous: Option<&GyroFit>) -> Option<GyroFit> {
        let pairs = self.pairs();
        if pairs.len() < cfg.min_pairs.max(3) {
            return None;
        }
        let turn_deg = rms_turn_deg(&pairs);
        if turn_deg < cfg.min_turn_deg {
            return None;
        }
        let known = match previous {
            Some(f) if matches!(f.mount, MountSource::Fitted { .. }) => Some((f.calib.cam_from_imu, f.mount)),
            _ => platform.map(|m| (*m, MountSource::Platform)),
        };
        let around = previous.map(|f| f.calib.offset_ns);
        let fitted = self.search(&pairs, buf, cfg, None, around).filter(|fit| {
            let MountSource::Fitted { sigma_deg } = fit.mount else { return false };
            sigma_deg <= cfg.max_mount_sigma_deg
                && known.is_none_or(|(m, source)| {
                    let near = rotation_distance(&m, &fit.calib.cam_from_imu).to_degrees() <= cfg.max_mount_deviation_deg;
                    let sharper = match source {
                        MountSource::Platform => true,
                        MountSource::Fitted { sigma_deg: known_sigma } => sigma_deg < known_sigma,
                    };
                    near && sharper
                })
        });
        let fit = fitted.or_else(|| self.search(&pairs, buf, cfg, Some(known?), around))?;
        let explained = fit.residual_deg <= cfg.max_residual_rel * turn_deg;
        (explained && fit.pairs >= cfg.min_pairs).then_some(fit)
    }

    /// The best offset — within the configured range, or near the offset
    /// found last (`around`): a coarse grid, then a fine one around its
    /// best. `mount` fixes the IMU→camera rotation, `None` fits it per
    /// offset.
    fn search(
        &self,
        pairs: &[PosePair],
        buf: &GyroBuffer,
        cfg: &GyroCalibCfg,
        mount: Option<(M3, MountSource)>,
        around: Option<f64>,
    ) -> Option<GyroFit> {
        let max_ns = cfg.max_offset_ms * 1e6;
        let best = |offsets: &mut dyn Iterator<Item = f64>| {
            offsets
                .filter(|off| off.abs() <= max_ns)
                .filter_map(|off| fit_at(pairs, buf, off, mount))
                .min_by(|a, b| a.residual_deg.total_cmp(&b.residual_deg))
        };
        let (centre, span) = around.map_or((0.0, max_ns), |c| (c, Self::TRACK_NS));
        let steps = (span / Self::OFFSET_STEP_NS).ceil() as i32;
        let coarse = best(&mut (-steps..=steps).map(|k| centre + f64::from(k) * Self::OFFSET_STEP_NS))?;
        let fine_step = Self::OFFSET_STEP_NS / f64::from(Self::FINE_STEPS);
        let centre = coarse.calib.offset_ns;
        let half = Self::FINE_STEPS / 2;
        best(&mut (-half..=half).map(|k| centre + f64::from(k) * fine_step))
    }
}

/// RMS rotation (deg) of the pose increments.
fn rms_turn_deg(pairs: &[PosePair]) -> f64 {
    (pairs.iter().map(|p| dot(p.cam, p.cam)).sum::<f64>() / pairs.len() as f64).sqrt().to_degrees()
}

/// Standard error (deg) of the mount `cam_from_imu` fitted to the
/// `(camera, gyro)` increments.
///
/// A small turn `δ` of the mount about the unit axis `e` changes an
/// increment `g` by `δ·e × g`, so the information about `δ` is
/// `eᵀ·Σ(|g|²·I − g·gᵀ)·e / σ²`, least along that matrix's weakest axis —
/// the axis the camera turned about most. `σ²` is the variance of the
/// residual across the increments; only its two components perpendicular
/// to `g` count (along `g` it is a scale error of the poses, which does not
/// turn the fit). The increments are taken as independent.
fn mount_sigma_deg(increments: &[(V3, V3)], cam_from_imu: &M3) -> f64 {
    let scatter = increments.iter().fold([[0.0; 3]; 3], |acc, &(cam, _)| mat_add(&acc, &outer(cam, cam)));
    let trace = scatter[0][0] + scatter[1][1] + scatter[2][2];
    let info: M3 = std::array::from_fn(|i| std::array::from_fn(|j| if i == j { trace } else { 0.0 } - scatter[i][j]));
    let (vals, _) = sym3_eigen(&info);
    let weakest = vals[2];
    let (mut ss, mut n) = (0.0, 0usize);
    for &(cam, imu) in increments {
        let Some(dir) = normalize(cam) else { continue };
        let r = sub(cam, mat_vec(cam_from_imu, imu));
        let across = sub(r, scale(dir, dot(r, dir)));
        ss += dot(across, across);
        n += 1;
    }
    if n == 0 || weakest <= 0.0 {
        return f64::INFINITY;
    }
    let sigma = (ss / (2 * n) as f64).sqrt();
    (sigma / weakest.sqrt()).to_degrees()
}

/// The fit at one time offset with the `mount` given, or with the mount by
/// Kabsch (the rotation that maps the gyro increments onto the camera's).
fn fit_at(pairs: &[PosePair], buf: &GyroBuffer, offset_ns: f64, mount: Option<(M3, MountSource)>) -> Option<GyroFit> {
    let increments: Vec<(V3, V3)> =
        pairs.iter().filter_map(|p| Some((p.cam, rotation_vector(&buf.rotation(p.t0_ns + offset_ns, p.t1_ns + offset_ns)?)))).collect();
    if increments.len() < 3 {
        return None;
    }
    let (cam_from_imu, mount) = mount.unwrap_or_else(|| {
        let r = nearest_rotation(&increments.iter().fold([[0.0; 3]; 3], |acc, &(cam, imu)| mat_add(&acc, &outer(cam, imu))));
        (r, MountSource::Fitted { sigma_deg: mount_sigma_deg(&increments, &r) })
    });
    let ss: f64 = increments.iter().map(|&(cam, imu)| norm(sub(cam, mat_vec(&cam_from_imu, imu))).powi(2)).sum();
    Some(GyroFit {
        calib: GyroCalib { cam_from_imu, offset_ns },
        mount,
        residual_deg: (ss / increments.len() as f64).sqrt().to_degrees(),
        pairs: increments.len(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A camera turning with `cam_rate`, sampled by a gyro mounted with
    /// `cam_from_imu`, biased by `bias` and running `offset_ns` ahead of the
    /// camera clock.
    struct Rig {
        cam_from_imu: M3,
        bias: V3,
        offset_ns: f64,
        cam_rate: fn(f64) -> V3,
    }

    /// Turns about a slowly wandering axis.
    fn wandering(t: f64) -> V3 {
        [0.6 * (2.1 * t).sin(), 0.4 * (1.3 * t + 1.0).cos(), 0.3 * (0.7 * t).sin()]
    }

    /// Turns about the camera's z axis only.
    fn single_axis(t: f64) -> V3 {
        [0.0, 0.0, 0.5 * (1.7 * t).sin()]
    }

    const MOUNT: M3 = [[0.0, -1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, -1.0]];

    impl Rig {
        fn new(cam_rate: fn(f64) -> V3) -> Self {
            // a residual bias of 0.1°/s per axis: what the model leaves out
            Rig { cam_from_imu: MOUNT, bias: [0.1f64, -0.1, 0.1].map(f64::to_radians), offset_ns: 6.4e6, cam_rate }
        }

        fn gyro(&self, until_s: f64, hz: f64) -> GyroBuffer {
            let mut buf = GyroBuffer::new(f64::INFINITY);
            let imu_from_cam = transpose(&self.cam_from_imu);
            for k in 0..=(until_s * hz) as usize {
                let t_gyro = k as f64 / hz;
                let t_cam = t_gyro - self.offset_ns * 1e-9;
                let rate = add(mat_vec(&imu_from_cam, (self.cam_rate)(t_cam)), self.bias);
                buf.push(GyroSample { t_ns: (t_gyro * 1e9) as i64, rate });
            }
            buf
        }

        /// Board→camera rotations at `times` (s): `Ṙ = −[ω]ₓ·R`, finely
        /// integrated.
        fn cam_rotations(&self, times: &[f64]) -> Vec<M3> {
            let mut r = rodrigues([2.5, 0.2, -0.1]);
            let mut t = 0.0;
            let h: f64 = 1e-4;
            times
                .iter()
                .map(|&target| {
                    while t < target - 1e-12 {
                        let dt = h.min(target - t);
                        r = mat_mul(&rodrigues(scale((self.cam_rate)(t + 0.5 * dt), -dt)), &r);
                        t += dt;
                    }
                    r
                })
                .collect()
        }

        /// 10 s of board poses at 30 fps.
        fn calibrator(&self) -> GyroCalibrator {
            let times: Vec<f64> = (1..300).map(|i| f64::from(i) / 30.0).collect();
            let mut cal = GyroCalibrator::new(400);
            for (t, r) in times.iter().zip(self.cam_rotations(&times)) {
                cal.add(t * 1e9, r);
            }
            cal
        }
    }

    #[test]
    fn integrated_rotation_matches_the_true_one() {
        let rig = Rig { cam_from_imu: IDENTITY, bias: [0.0; 3], offset_ns: 0.0, cam_rate: wandering };
        let buf = rig.gyro(2.0, 400.0);
        let rots = rig.cam_rotations(&[0.3, 0.8]);
        let truth = mat_mul(&rots[1], &transpose(&rots[0]));
        let d = buf.rotation(0.3e9, 0.8e9).unwrap();
        assert!(rotation_distance(&d, &truth).to_degrees() < 0.01);
        let back = buf.rotation(0.8e9, 0.3e9).unwrap();
        assert!(rotation_distance(&back, &transpose(&truth)).to_degrees() < 0.01);
    }

    #[test]
    fn buffer_refuses_gaps_and_uncovered_intervals() {
        let mut buf = GyroBuffer::new(10e9);
        for k in 0..10 {
            buf.push(GyroSample { t_ns: k * 5_000_000, rate: [0.1, 0.0, 0.0] });
        }
        buf.push(GyroSample { t_ns: 200_000_000, rate: [0.1, 0.0, 0.0] });
        buf.push(GyroSample { t_ns: 205_000_000, rate: [0.1, 0.0, 0.0] });
        assert!(buf.rotation(0.0, 40e6).is_some());
        assert!(buf.rotation(30e6, 150e6).is_none(), "dropout");
        assert!(buf.rotation(201e6, 204e6).is_some(), "after the dropout");
        assert!(buf.rotation(210e6, 260e6).is_none(), "past the newest sample");
        let w = buf.mean_rate(0.0, 40e6).unwrap();
        assert!((w[0] - 0.1).abs() < 1e-9 && w[1].abs() < 1e-12);
        assert_eq!(buf.mean_rate(12e6, 12e6), Some([0.1, 0.0, 0.0]));
    }

    #[test]
    fn two_axis_motion_fits_the_mount_and_the_offset() {
        let rig = Rig::new(wandering);
        let buf = rig.gyro(12.0, 200.0);
        let platform = mat_mul(&rodrigues([0.0, 0.5f64.to_radians(), 0.0]), &rig.cam_from_imu);
        let fit = rig.calibrator().solve(&buf, &GyroCalibCfg::default(), Some(&platform), None).expect("calibrates");
        let c = fit.calib;
        assert!(matches!(fit.mount, MountSource::Fitted { sigma_deg } if sigma_deg < 0.05), "{fit:?}");
        assert!(rotation_distance(&c.cam_from_imu, &rig.cam_from_imu).to_degrees() < 0.1, "{:?}", c.cam_from_imu);
        assert!((c.offset_ns - rig.offset_ns).abs() < 0.5e6, "offset {} ms", c.offset_ns * 1e-6);
        assert!(fit.residual_deg < 0.05, "{fit:?}");
    }

    #[test]
    fn single_axis_motion_keeps_the_known_mount() {
        let rig = Rig::new(single_axis);
        let buf = rig.gyro(12.0, 200.0);
        let cal = rig.calibrator();
        let cfg = GyroCalibCfg::default();
        assert!(cal.solve(&buf, &cfg, None, None).is_none(), "one axis cannot fix the mount");
        let platform = mat_mul(&rodrigues([0.0, 0.01, 0.0]), &rig.cam_from_imu);
        let fit = cal.solve(&buf, &cfg, Some(&platform), None).expect("calibrates with the platform's mount");
        assert_eq!((fit.calib.cam_from_imu, fit.mount), (platform, MountSource::Platform));
        assert!((fit.calib.offset_ns - rig.offset_ns).abs() < 0.5e6, "offset {} ms", fit.calib.offset_ns * 1e-6);
        // a mount fitted earlier outlives a phase that turns about one axis
        let earlier =
            GyroFit { mount: MountSource::Fitted { sigma_deg: 0.02 }, calib: GyroCalib { cam_from_imu: MOUNT, ..fit.calib }, ..fit };
        let again = cal.solve(&buf, &cfg, Some(&platform), Some(&earlier)).unwrap();
        assert_eq!((again.calib.cam_from_imu, again.mount), (MOUNT, earlier.mount));
    }

    #[test]
    fn a_mount_far_from_the_platform_is_refused() {
        let rig = Rig::new(wandering);
        let buf = rig.gyro(12.0, 200.0);
        let platform = mat_mul(&rodrigues([0.0, 0.0, 30f64.to_radians()]), &rig.cam_from_imu);
        // the fitted mount is 30° off the platform's, and the platform's does not fit
        assert!(rig.calibrator().solve(&buf, &GyroCalibCfg::default(), Some(&platform), None).is_none());
    }
}
