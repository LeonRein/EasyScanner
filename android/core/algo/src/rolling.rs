//! Rolling-shutter correction.
//!
//! A phone sensor exposes its rows one after the other: the last row of a
//! frame is read out 10–30 ms after the first. While the camera moves, the
//! board corners and the laser stripe in different rows are seen from
//! different camera poses, which shears the board pose and with it every
//! scan point. Given the camera's motion over the frame ([`FrameMotion`]:
//! angular rate, from the gyro, and velocity, from the recent board poses),
//! every observation is moved to the frame's middle row, so the pose and the
//! stripe describe one instant.

use crate::geom::{Intrinsics, Pose, V3, add, mat_mul, mat_t_vec, mat_vec, rodrigues, scale, sub};

/// When each row of a frame was exposed.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RowClock {
    /// Sensor timestamp: start of the first row's exposure (ns).
    pub start_ns: i64,
    /// First to last row readout (ns); 0 for a global shutter.
    pub readout_ns: i64,
    /// Exposure time of one row (ns).
    pub exposure_ns: i64,
    /// Rows in the image the pixels refer to.
    pub height: f64,
}

impl RowClock {
    /// Middle of the exposure of the row at image height `v` (ns).
    pub fn row_ns(&self, v: f64) -> f64 {
        let frac = if self.height > 0.0 { (v / self.height).clamp(0.0, 1.0) } else { 0.0 };
        self.start_ns as f64 + 0.5 * self.exposure_ns as f64 + frac * self.readout_ns as f64
    }

    /// The frame's reference instant: the middle row (ns). Poses solved
    /// from a corrected frame are poses at this time.
    pub fn mid_ns(&self) -> f64 {
        self.row_ns(0.5 * self.height)
    }

    /// First and last row (ns).
    pub fn span_ns(&self) -> (f64, f64) {
        (self.row_ns(0.0), self.row_ns(self.height))
    }

    /// Seconds from the reference instant to the row at `v`.
    pub fn dt(&self, v: f64) -> f64 {
        (self.row_ns(v) - self.mid_ns()) * 1e-9
    }
}

/// The camera's motion during one frame, constant over the readout.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FrameMotion {
    /// Angular rate (rad/s), camera frame.
    pub rate: V3,
    /// Velocity of the camera centre (mm/s), in the camera frame at the
    /// reference instant.
    ///
    /// There is no rotation-only correction: a hand-held camera turns to
    /// keep the object in view while it moves, so the two shears largely
    /// cancel, and undoing only the rotation makes the pose worse than
    /// leaving both.
    pub velocity: V3,
}

impl FrameMotion {
    /// A camera-frame point seen `dt` seconds after the reference instant,
    /// expressed in the camera frame at the reference instant.
    ///
    /// With `R(t) = exp(−ω·dt)·R_ref` and the centre moving by `v·dt`:
    /// `x_ref = exp(ω·dt)·x + v·dt`.
    pub fn to_reference(&self, x: V3, dt: f64) -> V3 {
        add(mat_vec(&rodrigues(scale(self.rate, dt)), x), scale(self.velocity, dt))
    }

    /// The inverse of [`Self::to_reference`].
    pub fn from_reference(&self, x_ref: V3, dt: f64) -> V3 {
        mat_t_vec(&rodrigues(scale(self.rate, dt)), sub(x_ref, scale(self.velocity, dt)))
    }
}

/// One frame's row timing and motion: moves its observations to the
/// reference instant.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Correction {
    /// When the frame's rows were exposed.
    pub clock: RowClock,
    /// How the camera moved meanwhile.
    pub motion: FrameMotion,
}

impl Correction {
    /// Board-corner pixels as the camera would have seen them at the
    /// reference instant. Moving a pixel needs its depth, which `hint` (a
    /// first pose estimate) provides.
    pub fn corners(&self, k: &Intrinsics, obj: &[V3], px: &[[f64; 2]], hint: &Pose) -> Vec<[f64; 2]> {
        obj.iter()
            .zip(px)
            .map(|(o, &[u, v])| {
                let dt = self.clock.dt(v);
                // depth of the corner in this row's camera frame
                let z = self.motion.from_reference(hint.to_cam(*o), dt)[2];
                k.project(self.motion.to_reference(scale(k.ray(u, v), z), dt)).unwrap_or([u, v])
            })
            .collect()
    }

    /// The board pose of the row at image height `v`, from the pose `mid`
    /// at the reference instant: the inverse of [`FrameMotion::to_reference`]
    /// applied to the board→camera map.
    pub fn pose_at_row(&self, mid: &Pose, v: f64) -> Pose {
        let dt = self.clock.dt(v);
        let back = rodrigues(scale(self.motion.rate, -dt));
        Pose { r: mat_mul(&back, &mid.r), t: mat_vec(&back, sub(mid.t, scale(self.motion.velocity, dt))) }
    }

    /// A stripe point triangulated in the camera frame of its own row (at
    /// image height `v`), in the camera frame at the reference instant.
    ///
    /// The laser is rigid to the camera, so the row's ray ∩ laser plane is
    /// exact in that row's frame; only the step to the board needs the
    /// motion.
    pub fn stripe_point(&self, x: V3, v: f64) -> V3 {
        self.motion.to_reference(x, self.clock.dt(v))
    }
}

/// Solve a board pose from corners with a rolling-shutter correction: once
/// from the raw pixels for the corners' depths, then from the corrected
/// ones. Without a correction, one plain solve.
///
/// `solve` is the PnP solver (OpenCV's IPPE in the app, a pure-Rust one in
/// the benchmark), `Ok(None)` when the view gives no pose; `pose` reads the
/// pose out of its result. The corrected solve falls back to the raw one if
/// it fails.
pub fn solve_corrected<T, E>(
    corr: Option<&Correction>,
    k: &Intrinsics,
    obj: &[V3],
    px: &[[f64; 2]],
    mut solve: impl FnMut(&[[f64; 2]]) -> Result<Option<T>, E>,
    pose: impl Fn(&T) -> Pose,
) -> Result<Option<T>, E> {
    let Some(raw) = solve(px)? else { return Ok(None) };
    let Some(c) = corr else { return Ok(Some(raw)) };
    let hint = pose(&raw);
    Ok(Some(solve(&c.corners(k, obj, px, &hint))?.unwrap_or(raw)))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geom::{norm, rotation_distance};

    fn k() -> Intrinsics {
        Intrinsics { fx: 920.0, fy: 920.0, cx: 639.5, cy: 479.5, dist: [0.05, -0.1, 0.0, 0.0, 0.0], width: 1280.0, height: 960.0 }
    }

    fn clock() -> RowClock {
        RowClock { start_ns: 1_000_000_000, readout_ns: 30_000_000, exposure_ns: 4_000_000, height: 960.0 }
    }

    #[test]
    fn row_times_span_the_readout() {
        let c = clock();
        assert_eq!(c.row_ns(0.0), 1_002_000_000.0);
        assert_eq!(c.mid_ns(), 1_017_000_000.0);
        assert!((c.dt(960.0) - 0.015).abs() < 1e-12);
        let global = RowClock { readout_ns: 0, ..c };
        assert_eq!(global.dt(900.0), 0.0);
    }

    #[test]
    fn motion_round_trips() {
        let m = FrameMotion { rate: [0.3, -0.5, 0.2], velocity: [40.0, -10.0, 5.0] };
        let x = [12.0, -30.0, 400.0];
        let back = m.from_reference(m.to_reference(x, 0.012), 0.012);
        assert!(norm(sub(back, x)) < 1e-9);
    }

    /// Corners observed by a moving rolling-shutter camera: each pixel
    /// projected with the pose of its own row.
    #[test]
    fn corrected_corners_match_a_global_shutter_view() {
        let k = k();
        let c = clock();
        let rate = [0.4, -0.6, 0.25];
        let vel_b = [150.0, -60.0, 30.0];
        let mid = Pose { r: rodrigues([2.8, 0.3, -0.2]), t: [-140.0, 90.0, 420.0] };
        let at = |dt: f64| {
            let r = mat_mul(&rodrigues(scale(rate, -dt)), &mid.r);
            let centre = add(mid.camera_position(), scale(vel_b, dt));
            Pose { r, t: scale(mat_vec(&r, centre), -1.0) }
        };
        let obj: Vec<V3> = (0..7).flat_map(|i| (0..7).map(move |j| [36.0 * (i + 1) as f64, 36.0 * (j + 1) as f64, 0.0])).collect();
        let observed: Vec<[f64; 2]> = obj
            .iter()
            .map(|&o| {
                let mut v = c.height / 2.0;
                let mut q = [0.0; 2];
                for _ in 0..6 {
                    q = k.project(at(c.dt(v)).to_cam(o)).unwrap();
                    v = q[1];
                }
                q
            })
            .collect();
        let truth: Vec<[f64; 2]> = obj.iter().map(|&o| k.project(mid.to_cam(o)).unwrap()).collect();
        let err = |px: &[[f64; 2]]| px.iter().zip(&truth).map(|(a, b)| (a[0] - b[0]).hypot(a[1] - b[1])).fold(0.0, f64::max);
        let corr = Correction { clock: c, motion: FrameMotion { rate, velocity: mat_vec(&mid.r, vel_b) } };
        let raw = err(&observed);
        let corrected = err(&corr.corners(&k, &obj, &observed, &mid));
        assert!(raw > 3.0, "the test motion must matter: {raw}");
        assert!(corrected < 1e-6, "{corrected}");
        // the pose of a row is the moving camera's pose at that row's time
        let v = 900.0;
        let row = corr.pose_at_row(&mid, v);
        let truth = at(c.dt(v));
        assert!(rotation_distance(&row.r, &truth.r) < 1e-6 && norm(sub(row.t, truth.t)) < 1e-9, "{row:?}");
    }
}
