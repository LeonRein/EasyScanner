//! The phone's gyroscope: a short buffer of its samples, its calibration
//! against the camera, and the online self-calibration that finds it.
//!
//! The gyro measures the body's angular rate in its own frame and clock. To
//! use it for the camera it needs three numbers the phone does not publish
//! reliably: the IMU→camera rotation, the residual rate bias, and the offset
//! between the gyro and camera timestamps. [`GyroCalibrator`] estimates all
//! three from ordinary board poses — the camera's rotation between two board
//! frames must equal the gyro's integrated rotation over the same interval —
//! so nothing has to be calibrated by hand.
//!
//! Rotation convention: a world→body rotation `R(t)` (like a board→camera
//! pose) evolves as `Ṙ = −[ω]ₓ·R` for the body rate `ω`, so the rotation
//! *increment* from `t0` to `t1` is `Δ = R(t1)·R(t0)ᵀ ≈ exp(−ω·(t1 − t0))`.

use std::collections::VecDeque;

use serde::{Deserialize, Serialize};

use crate::geom::{
    IDENTITY, M3, V3, add, mat_mul, mat_t_vec, mat_vec, nearest_rotation, norm, rodrigues, rotation_vector, scale, sub,
    sym3_eigen, transpose,
};

/// One gyroscope reading.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct GyroSample {
    /// Gyro clock (ns).
    pub t_ns: i64,
    /// Angular rate (rad/s), IMU frame.
    pub rate: V3,
}

/// The last few seconds of gyro samples, oldest first.
#[derive(Clone, Debug)]
pub struct GyroBuffer {
    samples: VecDeque<GyroSample>,
    span_ns: i64,
}

impl GyroBuffer {
    /// Samples further apart than this are a dropout, not a rate to
    /// interpolate across.
    pub const MAX_GAP_NS: i64 = 50_000_000;
    /// How far past the first or last sample a query may reach (the newest
    /// samples of a frame may still be in flight).
    pub const MAX_EXTRAPOLATION_NS: i64 = 20_000_000;

    /// A buffer that keeps `span_ns` of history.
    pub fn new(span_ns: i64) -> Self {
        GyroBuffer { samples: VecDeque::new(), span_ns }
    }

    pub fn clear(&mut self) {
        self.samples.clear();
    }

    pub fn len(&self) -> usize {
        self.samples.len()
    }

    pub fn is_empty(&self) -> bool {
        self.samples.is_empty()
    }

    pub fn newest_ns(&self) -> Option<i64> {
        self.samples.back().map(|s| s.t_ns)
    }

    /// Append a sample. Out-of-order duplicates are dropped; a jump back by
    /// more than the span (a restarted sensor clock) starts over.
    pub fn push(&mut self, s: GyroSample) {
        if let Some(last) = self.newest_ns()
            && s.t_ns <= last
        {
            if last - s.t_ns > self.span_ns {
                self.samples.clear();
            } else {
                return;
            }
        }
        self.samples.push_back(s);
        let oldest = s.t_ns - self.span_ns;
        while self.samples.front().is_some_and(|f| f.t_ns < oldest) {
            self.samples.pop_front();
        }
    }

    /// Rate at `t_ns`, linearly interpolated (held flat at the ends).
    fn rate_at(&self, t_ns: f64) -> V3 {
        let i = self.samples.partition_point(|s| (s.t_ns as f64) < t_ns);
        match (i.checked_sub(1).and_then(|j| self.samples.get(j)), self.samples.get(i)) {
            (Some(a), Some(b)) => {
                let s = (t_ns - a.t_ns as f64) / (b.t_ns - a.t_ns).max(1) as f64;
                add(a.rate, scale(sub(b.rate, a.rate), s))
            }
            (Some(a), None) => a.rate,
            (None, Some(b)) => b.rate,
            (None, None) => [0.0; 3],
        }
    }

    /// The samples cover `[t0, t1]` (gyro clock) without a dropout.
    fn covers(&self, t0: f64, t1: f64) -> bool {
        let (Some(first), Some(last)) = (self.samples.front(), self.samples.back()) else { return false };
        if t0 < (first.t_ns - Self::MAX_EXTRAPOLATION_NS) as f64 || t1 > (last.t_ns + Self::MAX_EXTRAPOLATION_NS) as f64 {
            return false;
        }
        // the samples bracketing the interval, and every one in between
        let from = self.samples.partition_point(|s| (s.t_ns as f64) < t0).saturating_sub(1);
        let to = self.samples.partition_point(|s| (s.t_ns as f64) < t1).min(self.samples.len() - 1);
        let span = self.samples.range(from..=to.max(from));
        span.clone().zip(span.skip(1)).all(|(a, b)| b.t_ns - a.t_ns <= Self::MAX_GAP_NS)
    }

    /// Rotation increment `Δ` of the IMU frame from `t0` to `t1` (gyro
    /// clock, ns; `t1 < t0` gives the inverse), with `bias` subtracted from
    /// every rate. `None` where the samples do not cover the interval.
    pub fn rotation(&self, t0: f64, t1: f64, bias: V3) -> Option<M3> {
        if t1 < t0 {
            return self.rotation(t1, t0, bias).map(|d| transpose(&d));
        }
        if !self.covers(t0, t1) {
            return None;
        }
        // midpoint rule between consecutive sample times
        let from = self.samples.partition_point(|s| (s.t_ns as f64) <= t0);
        let to = self.samples.partition_point(|s| (s.t_ns as f64) < t1);
        let knots = std::iter::once(t0).chain(self.samples.range(from..to.max(from)).map(|s| s.t_ns as f64)).chain(std::iter::once(t1));
        let mut delta = IDENTITY;
        let mut prev = t0;
        for t in knots.skip(1) {
            let dt = (t - prev) * 1e-9;
            if dt > 0.0 {
                let w = sub(self.rate_at(0.5 * (prev + t)), bias);
                delta = mat_mul(&rodrigues(scale(w, -dt)), &delta);
            }
            prev = t;
        }
        Some(delta)
    }

    /// Mean rate over `[t0, t1]` (gyro clock, ns), bias removed.
    pub fn mean_rate(&self, t0: f64, t1: f64, bias: V3) -> Option<V3> {
        let dt = (t1 - t0) * 1e-9;
        if dt <= 0.0 {
            return self.covers(t0, t0).then(|| sub(self.rate_at(t0), bias));
        }
        self.rotation(t0, t1, bias).map(|d| scale(rotation_vector(&d), -1.0 / dt))
    }
}

/// How the gyro relates to the camera.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct GyroCalib {
    /// IMU→camera rotation: `rate_cam = cam_from_imu · rate_imu`.
    pub cam_from_imu: M3,
    /// Rate bias (rad/s), IMU frame.
    pub bias: V3,
    /// Gyro clock minus camera clock (ns): camera time `t` is gyro time
    /// `t + offset_ns`.
    pub offset_ns: f64,
}

impl GyroCalib {
    /// The camera's rotation increment from `t0` to `t1` (camera clock, ns).
    pub fn rotation(&self, buf: &GyroBuffer, t0: f64, t1: f64) -> Option<M3> {
        let d = buf.rotation(t0 + self.offset_ns, t1 + self.offset_ns, self.bias)?;
        Some(mat_mul(&mat_mul(&self.cam_from_imu, &d), &transpose(&self.cam_from_imu)))
    }

    /// The camera's mean angular rate (rad/s, camera frame) over `[t0, t1]`.
    pub fn rate(&self, buf: &GyroBuffer, t0: f64, t1: f64) -> Option<V3> {
        let w = buf.mean_rate(t0 + self.offset_ns, t1 + self.offset_ns, self.bias)?;
        Some(mat_vec(&self.cam_from_imu, w))
    }
}

/// Acceptance limits of the self-calibration.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct GyroCalibCfg {
    /// Pose pairs needed for a fit.
    pub min_pairs: usize,
    /// The time offset is searched within ± this (ms).
    pub max_offset_ms: f64,
    /// RMS disagreement between camera and gyro rotation increments (deg)
    /// above which a fit is rejected.
    pub max_residual_deg: f64,
    /// The rotation increments must turn about at least two axes by this
    /// much (RMS, deg): about a single axis the IMU→camera rotation is not
    /// determined.
    pub min_excitation_deg: f64,
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
            // a board pose is good to ~0.05–0.1°, two of them make an
            // increment; a fit much worse than that has the offset wrong
            max_residual_deg: 0.2,
            min_excitation_deg: 0.5,
            interval_s: 2.0,
        }
    }
}

/// A calibration and how well it explains the poses it was fitted to.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GyroFit {
    pub calib: GyroCalib,
    /// RMS of `|log Δ_cam − cam_from_imu · log Δ_imu|` (deg).
    pub residual_deg: f64,
    pub pairs: usize,
}

/// Camera rotation increment between two board frames, the gyro's view of
/// the same interval is integrated per candidate offset.
#[derive(Clone, Copy, Debug)]
struct PosePair {
    t0: f64,
    t1: f64,
    /// `log(R1·R0ᵀ)`, camera frame.
    cam: V3,
}

/// Collects board-pose rotations and fits a [`GyroCalib`] to them.
#[derive(Clone, Debug)]
pub struct GyroCalibrator {
    /// Camera time (ns) and board→camera rotation of recent trusted poses.
    poses: VecDeque<(i64, M3)>,
    cap: usize,
}

impl GyroCalibrator {
    /// Pose pairs are this far apart: long enough for the rotation to rise
    /// above the pose noise, short enough for the small-bias linearisation.
    const PAIR_NS: i64 = 100_000_000;
    /// Pairs further apart than this (a gap in the board track) are skipped.
    const MAX_PAIR_NS: i64 = 200_000_000;
    /// Offset grid step of the coarse search (ns).
    const OFFSET_STEP_NS: f64 = 2_000_000.0;

    /// Keeps the newest `cap` poses.
    pub fn new(cap: usize) -> Self {
        GyroCalibrator { poses: VecDeque::with_capacity(cap), cap }
    }

    pub fn clear(&mut self) {
        self.poses.clear();
    }

    pub fn len(&self) -> usize {
        self.poses.len()
    }

    pub fn is_empty(&self) -> bool {
        self.poses.is_empty()
    }

    /// A trusted board→camera rotation at camera time `t_ns`.
    pub fn add(&mut self, t_ns: i64, r: M3) {
        if self.poses.back().is_some_and(|&(t, _)| t_ns <= t) {
            self.poses.clear();
        }
        if self.poses.len() == self.cap {
            self.poses.pop_front();
        }
        self.poses.push_back((t_ns, r));
    }

    fn pairs(&self) -> Vec<PosePair> {
        let mut out = Vec::new();
        let mut j = 0;
        for (i, &(t0, r0)) in self.poses.iter().enumerate() {
            j = j.max(i + 1);
            while self.poses.get(j).is_some_and(|&(t, _)| t - t0 < Self::PAIR_NS) {
                j += 1;
            }
            let Some(&(t1, r1)) = self.poses.get(j) else { break };
            if t1 - t0 <= Self::MAX_PAIR_NS {
                out.push(PosePair { t0: t0 as f64, t1: t1 as f64, cam: rotation_vector(&mat_mul(&r1, &transpose(&r0))) });
            }
        }
        out
    }

    /// Fit the calibration to the collected poses. `None` while there are
    /// too few pairs, the motion does not determine the fit, or the best fit
    /// still disagrees with the poses.
    ///
    /// The IMU→camera rotation is fitted when the camera turned about at
    /// least two axes; otherwise `nominal` (the mounting the phone reports)
    /// is used and only the bias and the offset are fitted, which a turn
    /// about one axis already determines.
    pub fn solve(&self, buf: &GyroBuffer, cfg: &GyroCalibCfg, nominal: Option<&M3>) -> Option<GyroFit> {
        let pairs = self.pairs();
        if pairs.len() < cfg.min_pairs.max(3) {
            return None;
        }
        let [first, second] = excitation_deg(&pairs);
        let fixed = if second >= cfg.min_excitation_deg {
            None
        } else if first >= cfg.min_excitation_deg {
            Some(nominal?)
        } else {
            return None;
        };
        // coarse grid over the offset with one alternation, then a finer
        // search around the best with the full fit
        let max = cfg.max_offset_ms * 1e6;
        let steps = (max / Self::OFFSET_STEP_NS).ceil() as i64;
        let coarse = (-steps..=steps)
            .map(|k| k as f64 * Self::OFFSET_STEP_NS)
            .filter_map(|off| fit_at(&pairs, buf, off, fixed, 1))
            .min_by(|a, b| a.residual_deg.total_cmp(&b.residual_deg))?;
        let centre = coarse.calib.offset_ns;
        let fine = (-8..=8)
            .map(|k| centre + k as f64 * Self::OFFSET_STEP_NS / 8.0)
            .filter_map(|off| fit_at(&pairs, buf, off, fixed, 4))
            .min_by(|a, b| a.residual_deg.total_cmp(&b.residual_deg))?;
        (fine.residual_deg <= cfg.max_residual_deg && fine.pairs >= cfg.min_pairs).then_some(fine)
    }
}

/// RMS turn (deg) of the pairs about their strongest and their
/// second-strongest axis.
fn excitation_deg(pairs: &[PosePair]) -> [f64; 2] {
    let mut s = [[0.0; 3]; 3];
    for p in pairs {
        for (i, row) in s.iter_mut().enumerate() {
            for (j, cell) in row.iter_mut().enumerate() {
                *cell += p.cam[i] * p.cam[j];
            }
        }
    }
    let (vals, _) = sym3_eigen(&s);
    [vals[0], vals[1]].map(|v| (v.max(0.0) / pairs.len() as f64).sqrt().to_degrees())
}

/// Rotation and bias at a fixed time offset, by alternating Kabsch (the
/// rotation that maps the gyro increments onto the camera's) and linear
/// least squares for the bias. A `fixed` rotation is kept as it is.
fn fit_at(pairs: &[PosePair], buf: &GyroBuffer, offset_ns: f64, fixed: Option<&M3>, iters: usize) -> Option<GyroFit> {
    let mut bias = [0.0; 3];
    let mut cam_from_imu = fixed.copied().unwrap_or(IDENTITY);
    let mut gyro: Vec<V3> = Vec::with_capacity(pairs.len());
    for _ in 0..iters.max(1) {
        gyro.clear();
        let mut used = Vec::with_capacity(pairs.len());
        for p in pairs {
            if let Some(d) = buf.rotation(p.t0 + offset_ns, p.t1 + offset_ns, bias) {
                gyro.push(rotation_vector(&d));
                used.push(*p);
            }
        }
        if used.len() < 3 {
            return None;
        }
        let mut m = [[0.0; 3]; 3];
        for (p, g) in used.iter().zip(&gyro) {
            for (i, row) in m.iter_mut().enumerate() {
                for (j, cell) in row.iter_mut().enumerate() {
                    *cell += p.cam[i] * g[j];
                }
            }
        }
        if fixed.is_none() {
            cam_from_imu = nearest_rotation(&m);
        }
        // log Δ_imu(bias + δ) ≈ log Δ_imu(bias) + δ·dt  (Δ = exp(−ω·dt))
        let (mut num, mut den) = ([0.0; 3], 0.0);
        for (p, g) in used.iter().zip(&gyro) {
            let dt = (p.t1 - p.t0) * 1e-9;
            num = add(num, scale(sub(mat_t_vec(&cam_from_imu, p.cam), *g), dt));
            den += dt * dt;
        }
        bias = add(bias, scale(num, 1.0 / den));
    }
    // residual with the final bias
    let mut ss = 0.0;
    let mut n = 0usize;
    for p in pairs {
        if let Some(d) = buf.rotation(p.t0 + offset_ns, p.t1 + offset_ns, bias) {
            ss += norm(sub(p.cam, mat_vec(&cam_from_imu, rotation_vector(&d)))).powi(2);
            n += 1;
        }
    }
    (n >= 3).then(|| GyroFit {
        calib: GyroCalib { cam_from_imu, bias, offset_ns },
        residual_deg: (ss / n as f64).sqrt().to_degrees(),
        pairs: n,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geom::rotation_distance;

    /// A camera turning about a slowly wandering axis, sampled by a gyro
    /// mounted with `cam_from_imu`, biased by `bias` and running `offset` ns
    /// ahead of the camera clock.
    struct Rig {
        cam_from_imu: M3,
        bias: V3,
        offset_ns: f64,
    }

    fn cam_rate(t: f64) -> V3 {
        [0.6 * (2.1 * t).sin(), 0.4 * (1.3 * t + 1.0).cos(), 0.3 * (0.7 * t).sin()]
    }

    /// Board→camera rotation: integrate `Ṙ = −[ω]ₓ·R` finely.
    fn cam_rotations(times: &[f64]) -> Vec<M3> {
        let mut r = rodrigues([2.5, 0.2, -0.1]);
        let mut t = 0.0;
        let h: f64 = 1e-4;
        times
            .iter()
            .map(|&target| {
                while t < target - 1e-12 {
                    let dt = h.min(target - t);
                    r = mat_mul(&rodrigues(scale(cam_rate(t + 0.5 * dt), -dt)), &r);
                    t += dt;
                }
                r
            })
            .collect()
    }

    impl Rig {
        fn gyro(&self, until_s: f64, hz: f64) -> GyroBuffer {
            let mut buf = GyroBuffer::new(i64::MAX / 4);
            let imu_from_cam = transpose(&self.cam_from_imu);
            let n = (until_s * hz) as usize;
            for k in 0..=n {
                let t_gyro = k as f64 / hz;
                let t_cam = t_gyro - self.offset_ns * 1e-9;
                let rate = add(mat_vec(&imu_from_cam, cam_rate(t_cam)), self.bias);
                buf.push(GyroSample { t_ns: (t_gyro * 1e9) as i64, rate });
            }
            buf
        }
    }

    fn rig() -> Rig {
        Rig { cam_from_imu: [[0.0, -1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, -1.0]], bias: [0.004, -0.002, 0.003], offset_ns: 6.4e6 }
    }

    #[test]
    fn integrated_rotation_matches_the_true_one() {
        let r = Rig { cam_from_imu: IDENTITY, bias: [0.0; 3], offset_ns: 0.0 };
        let buf = r.gyro(2.0, 400.0);
        let rots = cam_rotations(&[0.3, 0.8]);
        let truth = mat_mul(&rots[1], &transpose(&rots[0]));
        let d = buf.rotation(0.3e9, 0.8e9, [0.0; 3]).unwrap();
        assert!(rotation_distance(&d, &truth).to_degrees() < 0.01);
        let back = buf.rotation(0.8e9, 0.3e9, [0.0; 3]).unwrap();
        assert!(rotation_distance(&back, &transpose(&truth)).to_degrees() < 0.01);
    }

    #[test]
    fn buffer_refuses_gaps_and_uncovered_intervals() {
        let mut buf = GyroBuffer::new(10_000_000_000);
        for k in 0..10 {
            buf.push(GyroSample { t_ns: k * 5_000_000, rate: [0.1, 0.0, 0.0] });
        }
        buf.push(GyroSample { t_ns: 200_000_000, rate: [0.1, 0.0, 0.0] });
        assert!(buf.rotation(0.0, 40e6, [0.0; 3]).is_some());
        assert!(buf.rotation(30e6, 150e6, [0.0; 3]).is_none(), "dropout");
        assert!(buf.rotation(210e6, 260e6, [0.0; 3]).is_none(), "past the newest sample");
        let w = buf.mean_rate(0.0, 40e6, [0.0; 3]).unwrap();
        assert!((w[0] - 0.1).abs() < 1e-9 && w[1].abs() < 1e-12);
    }

    #[test]
    fn self_calibration_recovers_rotation_bias_and_offset() {
        let rig = rig();
        let buf = rig.gyro(12.0, 200.0);
        let times: Vec<f64> = (1..300).map(|i| i as f64 / 30.0).collect();
        let mut cal = GyroCalibrator::new(400);
        for (t, r) in times.iter().zip(cam_rotations(&times)) {
            cal.add((t * 1e9) as i64, r);
        }
        let fit = cal.solve(&buf, &GyroCalibCfg::default(), None).expect("calibrates");
        let c = fit.calib;
        assert!(rotation_distance(&c.cam_from_imu, &rig.cam_from_imu).to_degrees() < 0.05, "{:?}", c.cam_from_imu);
        assert!(norm(sub(c.bias, rig.bias)) < 2e-4, "bias {:?}", c.bias);
        assert!((c.offset_ns - rig.offset_ns).abs() < 0.5e6, "offset {} ms", c.offset_ns * 1e-6);
        assert!(fit.residual_deg < 0.01, "{fit:?}");
    }

    #[test]
    fn single_axis_motion_needs_the_nominal_mounting() {
        let rig = rig();
        let imu_from_cam = transpose(&rig.cam_from_imu);
        let rate = |t: f64| [0.0, 0.0, 0.5 * (1.7 * t).sin()];
        let mut buf = GyroBuffer::new(i64::MAX / 4);
        for k in 0..=2400 {
            let t = k as f64 / 200.0;
            let w = add(mat_vec(&imu_from_cam, rate(t - rig.offset_ns * 1e-9)), rig.bias);
            buf.push(GyroSample { t_ns: (t * 1e9) as i64, rate: w });
        }
        let mut cal = GyroCalibrator::new(400);
        let mut r = rodrigues([2.5, 0.2, -0.1]);
        let (h, mut t) = (1e-4, 0.0);
        for i in 1..300 {
            let target = i as f64 / 30.0;
            while t < target - 1e-12 {
                r = mat_mul(&rodrigues(scale(rate(t + 0.5 * h), -h)), &r);
                t += h;
            }
            cal.add((target * 1e9) as i64, r);
        }
        let cfg = GyroCalibCfg::default();
        assert!(cal.solve(&buf, &cfg, None).is_none(), "one axis cannot fix the mounting");
        let nominal = mat_mul(&rodrigues([0.0, 0.01, 0.0]), &rig.cam_from_imu);
        let fit = cal.solve(&buf, &cfg, Some(&nominal)).expect("calibrates with the nominal mounting");
        assert_eq!(fit.calib.cam_from_imu, nominal);
        assert!((fit.calib.offset_ns - rig.offset_ns).abs() < 0.5e6, "offset {} ms", fit.calib.offset_ns * 1e-6);
    }
}
