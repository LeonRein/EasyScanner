//! The synthetic camera: a hand-held trajectory around the board, the line
//! laser rigidly mounted next to the camera, and what one frame shows —
//! ChArUco corner pixels and laser-stripe pixels — including corner noise,
//! occlusion, rolling shutter and (optionally) a rendered YUV frame that the
//! app's real [`LaserDetector`] has to find the stripe in — and the phone's
//! gyroscope riding along.

use rayon::prelude::*;
use serde::{Deserialize, Serialize};

use crate::geom::{
    Intrinsics, M3, Pose, V3, add, cross, dot, mat_mul, mat_t_vec, mat_vec, normalize, rodrigues, rotation_vector, scale, sub, transpose,
};
use crate::imu::{GyroCalib, GyroSample};
use crate::laser::{FrameError, LaserDetector, LaserParams, Roi, YuvFrame, YuvLayout};
use crate::plane::Rng;

use super::scene::Scene;
use crate::recording::CameraModel;

const TWO53: f64 = (1u64 << 53) as f64;

/// Uniform sample in `[0, 1)`.
pub fn uniform(rng: &mut Rng) -> f64 {
    (rng.next_u64() >> 11) as f64 / TWO53
}

/// Standard normal sample (Box–Muller).
pub fn gauss(rng: &mut Rng) -> f64 {
    let u1 = ((rng.next_u64() >> 11) as f64 + 0.5) / TWO53;
    (-2.0 * u1.ln()).sqrt() * (std::f64::consts::TAU * uniform(rng)).cos()
}

/// How far the camera calibration the pipeline uses is off from the truth.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct IntrinsicsError {
    /// Relative focal-length error (0.005 = +0.5 %), both axes.
    pub focal_rel: f64,
    pub cx_px: f64,
    pub cy_px: f64,
    /// Added to k1.
    pub k1: f64,
}

impl IntrinsicsError {
    pub fn apply(&self, k: CameraModel) -> CameraModel {
        let mut e = k;
        e.fx *= 1.0 + self.focal_rel;
        e.fy *= 1.0 + self.focal_rel;
        e.cx += self.cx_px;
        e.cy += self.cy_px;
        e.dist[0] += self.k1;
        e
    }
}

/// The line laser, rigidly mounted beside the camera: its emitter sits
/// `baseline_mm` along the camera's +X, and its light sheet contains the
/// camera's Y axis direction and crosses the optical axis at `cross_mm`.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct LaserMount {
    pub baseline_mm: f64,
    pub cross_mm: f64,
    /// Gaussian half-thickness (σ) of the light sheet.
    pub sheet_sigma_mm: f64,
    /// Full fan angle of the line optics.
    pub fan_deg: f64,
}

impl Default for LaserMount {
    fn default() -> Self {
        LaserMount { baseline_mm: 100.0, cross_mm: 350.0, sheet_sigma_mm: 0.4, fan_deg: 70.0 }
    }
}

impl LaserMount {
    /// Emitter position, camera frame.
    pub fn origin(&self) -> V3 {
        [self.baseline_mm, 0.0, 0.0]
    }

    /// Central direction of the fan, camera frame.
    pub fn axis(&self) -> V3 {
        normalize([-self.baseline_mm, 0.0, self.cross_mm]).unwrap_or([0.0, 0.0, 1.0])
    }

    /// The sheet `n·X = d` in the camera frame, `n.z ≤ 0` like
    /// [`crate::plane::PlaneFit`].
    pub fn plane(&self) -> (V3, f64) {
        let mut n = normalize(cross([0.0, 1.0, 0.0], self.axis())).unwrap();
        if n[2] > 0.0 {
            n = scale(n, -1.0);
        }
        (n, dot(n, self.origin()))
    }

    /// Whether a camera-frame point is inside the fan (in front, within the
    /// fan angle), ignoring its distance from the sheet.
    fn in_fan(&self, x: V3) -> bool {
        let r = sub(x, self.origin());
        let along = dot(r, self.axis());
        along > 0.0 && r[1].abs() <= along * (0.5 * self.fan_deg).to_radians().tan()
    }
}

/// A hand-held scan: the camera orbits an aim point on the board while the
/// hand sweeps the aim (and with it the laser line) back and forth, plus a
/// small tremor.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Trajectory {
    pub duration_s: f64,
    pub fps: f64,
    /// Orbit centre, board frame (−Z is up).
    pub aim: V3,
    pub distance_mm: f64,
    /// Viewing elevation above the board plane.
    pub elevation_deg: f64,
    pub azimuth_start_deg: f64,
    /// How far the orbit turns over the whole scan.
    pub azimuth_span_deg: f64,
    /// Amplitude of the sideways sweep of the aim point.
    pub sweep_mm: f64,
    pub sweep_hz: f64,
    /// Amplitude of the camera-position tremor …
    pub tremor_mm: f64,
    /// … and of its turning (deg): a hand shakes the phone about its grip.
    pub tremor_deg: f64,
    pub tremor_hz: f64,
    /// A hand never aims perfectly: the camera turns about its own axes by
    /// up to this much …
    pub wobble_deg: f64,
    /// … this often.
    pub wobble_hz: f64,
}

impl Default for Trajectory {
    fn default() -> Self {
        Trajectory {
            duration_s: 20.0,
            fps: 30.0,
            aim: [144.0, 144.0, -15.0],
            distance_mm: 380.0,
            elevation_deg: 60.0,
            azimuth_start_deg: 0.0,
            azimuth_span_deg: 360.0,
            sweep_mm: 40.0,
            sweep_hz: 0.25,
            tremor_mm: 0.3,
            tremor_deg: 0.1,
            tremor_hz: 7.0,
            wobble_deg: 1.0,
            wobble_hz: 0.5,
        }
    }
}

/// Board→camera pose of a camera at `cam` looking at `aim`, its +X along
/// `side` (projected), board frame.
pub fn look_at(cam: V3, aim: V3, side: V3) -> Pose {
    let z = normalize(sub(aim, cam)).unwrap();
    let x = normalize(sub(side, scale(z, dot(z, side)))).unwrap();
    let y = cross(z, x);
    let r = [x, y, z];
    Pose { r, t: scale(mat_vec(&r, cam), -1.0) }
}

impl Trajectory {
    pub fn frame_times(&self) -> Vec<f64> {
        let n = (self.duration_s * self.fps).round().max(1.0) as usize;
        (0..n).map(|i| i as f64 / self.fps).collect()
    }

    /// Pose at time `t` (s).
    pub fn pose(&self, t: f64) -> Pose {
        let az = (self.azimuth_start_deg + self.azimuth_span_deg * t / self.duration_s.max(1e-9)).to_radians();
        let el = self.elevation_deg.to_radians();
        let out = [el.cos() * az.cos(), el.cos() * az.sin(), -el.sin()];
        let side = [-az.sin(), az.cos(), 0.0];
        let tau = std::f64::consts::TAU;
        let aim = add(self.aim, scale(side, self.sweep_mm * (tau * self.sweep_hz * t).sin()));
        let w = tau * self.tremor_hz * t;
        let tremor = scale([w.sin(), (1.3 * w + 1.0).sin(), (0.7 * w + 2.0).sin()], self.tremor_mm);
        let cam = add(add(aim, scale(out, self.distance_mm)), tremor);
        let aimed = look_at(cam, aim, side);
        let ww = tau * self.wobble_hz * t;
        let wobble = scale([ww.sin(), (1.37 * ww + 1.0).sin(), 0.5 * (0.83 * ww + 2.0).sin()], self.wobble_deg.to_radians());
        let shake = scale([(1.1 * w + 0.5).sin(), (0.9 * w + 2.5).sin(), 0.5 * (1.2 * w + 1.5).sin()], self.tremor_deg.to_radians());
        let r = mat_mul(&rodrigues(add(wobble, shake)), &aimed.r);
        Pose { r, t: scale(mat_vec(&r, cam), -1.0) }
    }
}

impl Trajectory {
    /// The camera's angular rate at `t` (rad/s, camera frame), from the
    /// rotation increment `R(t+h)·R(t−h)ᵀ = exp(−ω·2h)`.
    pub fn rate(&self, t: f64) -> V3 {
        const H: f64 = 1e-4;
        let d = mat_mul(&self.pose(t + H).r, &transpose(&self.pose(t - H).r));
        scale(rotation_vector(&d), -0.5 / H)
    }
}

/// The phone's gyroscope as the app sees it after Android's own bias
/// compensation: a sample stream in the IMU frame and clock, with white
/// noise, a residual bias that may wander, scale errors and timestamp
/// jitter.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct GyroModel {
    pub rate_hz: f64,
    /// White noise per sample, σ (deg/s).
    pub noise_dps: f64,
    /// Residual bias at the start (deg/s), IMU frame.
    pub bias_dps: V3,
    /// Random walk of the bias (deg/s per √s).
    pub bias_walk_dps: f64,
    /// Relative scale error per IMU axis (0.01 = reads 1 % high).
    pub scale_error: V3,
    /// σ of the sample timestamps around the true sampling instant (ms).
    pub jitter_ms: f64,
    /// Gyro clock minus camera clock (ms).
    pub offset_ms: f64,
    /// IMU→camera rotation.
    pub cam_from_imu: M3,
    /// How far the mount the platform defines (from `SENSOR_ORIENTATION`)
    /// is off the true one: the assembly tolerance (deg).
    pub platform_mount_error_deg: f64,
}

impl Default for GyroModel {
    /// A modern phone IMU (≈0.005 °/s/√Hz) at 200 Hz, a back camera of a
    /// phone whose sensor is mounted landscape (x ← −y, y ← −x, z ← −z).
    fn default() -> Self {
        GyroModel {
            rate_hz: 200.0,
            noise_dps: 0.07,
            bias_dps: [0.08, -0.05, 0.06],
            bias_walk_dps: 0.0,
            scale_error: [0.0; 3],
            jitter_ms: 0.0,
            offset_ms: 4.0,
            cam_from_imu: [[0.0, -1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, -1.0]],
            platform_mount_error_deg: 0.5,
        }
    }
}

impl GyroModel {
    /// The default gyro, but with the imperfections a calibration that
    /// assumes an ideal sensor does not model: 1 % scale errors, a bias
    /// that wanders by 0.02°/s/√s, and 0.5 ms timestamp jitter.
    pub fn imperfect() -> Self {
        GyroModel { bias_walk_dps: 0.02, scale_error: [0.01, -0.008, 0.012], jitter_ms: 0.5, ..GyroModel::default() }
    }

    /// The true calibration.
    pub fn calib(&self) -> GyroCalib {
        GyroCalib { cam_from_imu: self.cam_from_imu, offset_ns: self.offset_ms * 1e6 }
    }

    /// The IMU→camera rotation the platform defines.
    pub fn platform_mount(&self) -> M3 {
        let e = self.platform_mount_error_deg.to_radians();
        mat_mul(&rodrigues(scale([0.6, -0.8, 0.0], e)), &self.cam_from_imu)
    }

    /// Samples over `[0, until_s]` of camera time along `traj`.
    pub fn sample(&self, traj: &Trajectory, until_s: f64, rng: &mut Rng) -> Vec<GyroSample> {
        let imu_from_cam = transpose(&self.cam_from_imu);
        let sigma = self.noise_dps.to_radians();
        let dt = 1.0 / self.rate_hz;
        let walk = self.bias_walk_dps.to_radians() * dt.sqrt();
        let mut bias = self.bias_dps.map(f64::to_radians);
        let n = (until_s * self.rate_hz).ceil() as usize;
        (0..=n)
            .map(|k| {
                let t_cam = k as f64 * dt;
                let true_rate = mat_vec(&imu_from_cam, traj.rate(t_cam));
                let scaled: V3 = std::array::from_fn(|i| true_rate[i] * (1.0 + self.scale_error[i]));
                let noise = [gauss(rng), gauss(rng), gauss(rng)].map(|g| sigma * g);
                bias = add(bias, [gauss(rng), gauss(rng), gauss(rng)].map(|g| walk * g));
                let stamp = t_cam + self.jitter_ms * 1e-3 * gauss(rng);
                GyroSample { t_ns: (stamp * 1e9 + self.offset_ms * 1e6).round() as i64, rate: add(add(scaled, bias), noise) }
            })
            .collect()
    }
}

/// How the frame timing the camera reports differs from the truth.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct TimingError {
    /// Relative error of the reported rolling-shutter skew (0.1 = reports
    /// 10 % more than the true readout).
    pub readout_rel: f64,
    /// The reported timestamp lies this much after the true start of the
    /// first row's exposure (ms); a phone that stamps the end of the
    /// readout has the readout here.
    pub timestamp_shift_ms: f64,
    /// Per frame, the exposure is drawn uniformly from this range (ms) and
    /// reported correctly; `[0, 0]` for none.
    pub exposure_ms: [f64; 2],
}

/// How stripe pixels are produced.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum StripeModel {
    /// Exact projection of the sheet∩scene curve plus Gaussian pixel noise:
    /// fast, isolates the geometry.
    Analytic { noise_px: f64 },
    /// Render a YUV_420 frame (board print, solids, laser sheet with its
    /// thickness and shadows, sensor noise) and run the app's
    /// [`LaserDetector`] on it.
    Render {
        /// Sub-samples per pixel side.
        supersample: u32,
        /// Luma of a white surface under ambient light.
        ambient: f64,
        /// Red added at the sheet centre on a white surface (saturates at 255).
        laser_gain: f64,
        /// Per-channel sensor noise σ (8-bit units).
        sensor_noise: f64,
    },
}

impl Default for StripeModel {
    fn default() -> Self {
        StripeModel::Analytic { noise_px: 0.0 }
    }
}

impl StripeModel {
    pub fn render() -> Self {
        StripeModel::Render { supersample: 2, ambient: 150.0, laser_gain: 600.0, sensor_noise: 2.0 }
    }
}

/// One simulated frame's observations.
#[derive(Clone, Debug, Default)]
pub struct Observation {
    /// ChArUco corner id and pixel.
    pub corners: Vec<(u32, [f64; 2])>,
    pub laser_px: Vec<[f32; 2]>,
}

/// Everything that produces observations, shared by the scan and the
/// simulated laser-plane calibration.
pub struct Camera<'a> {
    pub scene: &'a Scene,
    pub k: Intrinsics,
    pub mount: LaserMount,
    pub stripe: StripeModel,
    pub corner_noise_px: f64,
    /// Rolling-shutter readout time top→bottom row (0 = global shutter).
    pub readout_s: f64,
    pub rng: Rng,
    detector: LaserDetector,
    /// Unit camera rays through the pixel centres (render cache).
    rays: Vec<V3>,
}

/// The detector's own limit on stripe points per frame.
const MAX_STRIPE_PX: usize = 400;

impl<'a> Camera<'a> {
    pub fn new(
        scene: &'a Scene,
        k: Intrinsics,
        mount: LaserMount,
        stripe: StripeModel,
        corner_noise_px: f64,
        readout_s: f64,
        seed: u64,
    ) -> Self {
        Camera {
            scene,
            k,
            mount,
            stripe,
            corner_noise_px,
            readout_s,
            rng: Rng::new(seed),
            detector: LaserDetector::default(),
            rays: Vec::new(),
        }
    }

    fn row_time(&self, t0: f64, v: f64) -> f64 {
        t0 + self.readout_s * (v / self.k.height).clamp(0.0, 1.0)
    }

    /// Pixel of board point `p` exposed by its own row's pose (rolling
    /// shutter fixed point), or `None` when not visible.
    fn project_rs(&self, traj: &(dyn Fn(f64) -> Pose + Sync), t0: f64, p: V3) -> Option<([f64; 2], Pose)> {
        let mut v = 0.5 * self.k.height;
        let mut out = None;
        for _ in 0..if self.readout_s > 0.0 { 4 } else { 1 } {
            let pose = traj(self.row_time(t0, v));
            let q = self.k.project(pose.to_cam(p))?;
            v = q[1];
            out = Some((q, pose));
        }
        out
    }

    fn in_image(&self, q: [f64; 2], margin: f64) -> bool {
        q[0] >= margin && q[1] >= margin && q[0] <= self.k.width - 1.0 - margin && q[1] <= self.k.height - 1.0 - margin
    }

    /// Observe the frame whose first row starts exposing at `t0`.
    pub fn observe(&mut self, traj: &(dyn Fn(f64) -> Pose + Sync), t0: f64) -> Observation {
        let mut obs = Observation::default();
        for (id, p) in self.scene.board.corners() {
            let Some((q, pose)) = self.project_rs(traj, t0, p) else { continue };
            if !self.in_image(q, 3.0) || !self.scene.visible(pose.camera_position(), p) {
                continue;
            }
            let n = self.corner_noise_px;
            let q = [q[0] + n * gauss(&mut self.rng), q[1] + n * gauss(&mut self.rng)];
            obs.corners.push((id, q));
        }
        obs.laser_px = match self.stripe {
            StripeModel::Analytic { noise_px } => self.stripe_analytic(traj, t0, noise_px),
            StripeModel::Render { .. } => {
                let img = self.render(traj, t0);
                let mut px = Vec::new();
                match img.frame() {
                    Ok(frame) => {
                        self.detector.detect(&frame, &Roi::full(img.width, img.height), &LaserParams::default(), &mut px);
                    }
                    // a camera model the renderer cannot produce a frame for sees no stripe
                    Err(e) => log::warn!("rendered frame unusable: {e}"),
                }
                px
            }
        };
        obs
    }

    /// The sheet's footprint on the scene, sampled along the fan.
    fn stripe_analytic(&mut self, traj: &(dyn Fn(f64) -> Pose + Sync), t0: f64, noise_px: f64) -> Vec<[f32; 2]> {
        const RAYS: usize = 1200;
        let (a, l) = (self.mount.axis(), self.mount.origin());
        let half = 0.5 * self.mount.fan_deg.to_radians();
        let mut px = Vec::new();
        for i in 0..RAYS {
            let ang = -half + 2.0 * half * i as f64 / (RAYS - 1) as f64;
            let dir = add(scale(a, ang.cos()), scale([0.0, 1.0, 0.0], ang.sin()));
            let mut v = 0.5 * self.k.height;
            let mut hit = None;
            for _ in 0..if self.readout_s > 0.0 { 4 } else { 1 } {
                let pose = traj(self.row_time(t0, v));
                let (o, d) = (pose.to_board(l), mat_t_vec(&pose.r, dir));
                let Some((s, _)) = self.scene.trace(o, d) else {
                    hit = None;
                    break;
                };
                let x = add(o, scale(d, s));
                let Some(q) = self.k.project(pose.to_cam(x)) else {
                    hit = None;
                    break;
                };
                v = q[1];
                hit = self.scene.visible(pose.camera_position(), x).then_some(q);
                if hit.is_none() {
                    break;
                }
            }
            if let Some(q) = hit
                && self.in_image(q, 1.0)
            {
                px.push([(q[0] + noise_px * gauss(&mut self.rng)) as f32, (q[1] + noise_px * gauss(&mut self.rng)) as f32]);
            }
        }
        if px.len() > MAX_STRIPE_PX {
            let step = px.len().div_ceil(MAX_STRIPE_PX);
            px = px.into_iter().step_by(step).collect();
        }
        px
    }

    /// Render the frame as YUV 4:2:0, the way the camera delivers it.
    pub fn render(&mut self, traj: &(dyn Fn(f64) -> Pose + Sync), t0: f64) -> YuvImage {
        let StripeModel::Render { supersample, ambient, laser_gain, sensor_noise } = self.stripe else {
            panic!("render() needs StripeModel::Render");
        };
        let (w, h) = (self.k.width as usize, self.k.height as usize);
        let ss = supersample.max(1) as usize;
        let (n, d) = self.mount.plane();
        let sigma = self.mount.sheet_sigma_mm;
        let l = self.mount.origin();
        let seed = self.rng.next_u64();
        let (scene, k, mount) = (self.scene, self.k, self.mount);
        let readout = self.readout_s;
        if self.rays.len() != w * h {
            self.rays = (0..w * h).map(|i| normalize(k.ray((i % w) as f64, (i / w) as f64)).unwrap()).collect();
        }
        let rays = &self.rays;
        let mut rgb = vec![0u8; w * h * 3];
        rgb.par_chunks_mut(w * 3).enumerate().for_each(|(v, row)| {
            let pose = traj(t0 + readout * ((v as f64 + 0.5) / k.height));
            let cam = pose.camera_position();
            let l_b = pose.to_board(l);
            let mut rng = Rng::new(seed ^ (v as u64 + 1).wrapping_mul(0x9E37_79B9_7F4A_7C15));
            // colour seen along one camera ray (camera frame direction)
            let shade = |dir_c: V3| -> Option<([f64; 3], f64)> {
                let dir = mat_t_vec(&pose.r, dir_c);
                let (s, surf) = scene.trace(cam, dir)?;
                let x = add(cam, scale(dir, s));
                let alb = scene.albedo(x, surf);
                let xc = pose.to_cam(x);
                let delta = dot(n, xc) - d;
                let lit = if delta.abs() < 5.0 * sigma && mount.in_fan(xc) && scene.visible(l_b, x) {
                    (-0.5 * (delta / sigma).powi(2)).exp()
                } else {
                    0.0
                };
                let base = alb * ambient;
                let red = laser_gain * lit * alb;
                // second value: how far (mm, beyond the sheet's reach) this pixel is from being lit
                let reach = delta.abs() - 5.0 * sigma - 4.0 * xc[2] / k.fx;
                Some(([base + red, base + 0.12 * red, base + 0.12 * red], reach))
            };
            for u in 0..w {
                let mut col = [40.0; 3];
                if let Some((c, reach)) = shade(rays[v * w + u]) {
                    col = c;
                    if ss > 1 && reach < 0.0 {
                        // near the sheet: average sub-samples for a smooth stripe profile
                        let mut acc = [0.0; 3];
                        for sy in 0..ss {
                            for sx in 0..ss {
                                let uu = u as f64 + (sx as f64 + 0.5) / ss as f64 - 0.5;
                                let vv = v as f64 + (sy as f64 + 0.5) / ss as f64 - 0.5;
                                let c = shade(normalize(k.ray(uu, vv)).unwrap()).map_or([40.0; 3], |s| s.0);
                                acc = add(acc, c);
                            }
                        }
                        col = scale(acc, 1.0 / (ss * ss) as f64);
                    }
                }
                for c in 0..3 {
                    let val = col[c] + sensor_noise * gauss(&mut rng);
                    row[u * 3 + c] = val.round().clamp(0.0, 255.0) as u8;
                }
            }
        });
        YuvImage::from_rgb(&rgb, w, h)
    }
}

/// An owned planar YUV 4:2:0 image.
pub struct YuvImage {
    pub width: usize,
    pub height: usize,
    pub y: Vec<u8>,
    pub u: Vec<u8>,
    pub v: Vec<u8>,
}

impl YuvImage {
    /// JFIF conversion with 2x2-averaged chroma, like a camera ISP (the same
    /// one `examples/laser_parity.rs` feeds the detector).
    pub fn from_rgb(rgb: &[u8], w: usize, h: usize) -> YuvImage {
        let mut y = vec![0u8; w * h];
        let (cw, ch) = (w / 2, h / 2);
        let mut uf = vec![0f64; cw * ch];
        let mut vf = vec![0f64; cw * ch];
        for yy in 0..ch * 2 {
            for xx in 0..cw * 2 {
                let i = (yy * w + xx) * 3;
                let (r, g, b) = (rgb[i] as f64, rgb[i + 1] as f64, rgb[i + 2] as f64);
                y[yy * w + xx] = (0.299 * r + 0.587 * g + 0.114 * b).round().clamp(0.0, 255.0) as u8;
                let ci = (yy / 2) * cw + xx / 2;
                uf[ci] += 0.25 * (128.0 - 0.168736 * r - 0.331264 * g + 0.5 * b);
                vf[ci] += 0.25 * (128.0 + 0.5 * r - 0.418688 * g - 0.081312 * b);
            }
        }
        let q = |c: &f64| c.round().clamp(0.0, 255.0) as u8;
        YuvImage { width: w, height: h, y, u: uf.iter().map(q).collect(), v: vf.iter().map(q).collect() }
    }

    pub fn frame(&self) -> Result<YuvFrame<'_>, FrameError> {
        YuvFrame::new(YuvLayout::packed_planar(self.width, self.height), &self.y, &self.u, &self.v)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bench::scene::Shape;
    use crate::recording::BoardDims;

    #[test]
    fn mount_plane_contains_origin_and_crossing() {
        let m = LaserMount::default();
        let (n, d) = m.plane();
        assert!((dot(n, m.origin()) - d).abs() < 1e-9);
        assert!((dot(n, [0.0, 0.0, m.cross_mm]) - d).abs() < 1e-9);
        assert!((dot(n, [0.0, 50.0, m.cross_mm]) - d).abs() < 1e-9);
        assert!(n[2] <= 0.0);
    }

    #[test]
    fn trajectory_looks_at_the_aim_from_above() {
        let tr = Trajectory::default();
        for t in [0.0, 3.3, 10.0] {
            let p = tr.pose(t);
            let c = p.camera_position();
            assert!(c[2] < -200.0, "camera above the board: {c:?}");
            assert!(p.r[2][2] > 0.0, "board +Z points away from the camera");
            let a = p.to_cam(tr.aim);
            assert!(a[0].abs() < 45.0 && a[1].abs() < 45.0 && a[2] > 300.0, "{a:?}");
        }
    }

    #[test]
    fn trajectory_rate_integrates_to_its_rotation() {
        let traj = Trajectory::default();
        let (t0, t1) = (3.0, 3.2);
        let mut d = crate::geom::IDENTITY;
        let steps = 2000;
        for i in 0..steps {
            let dt = (t1 - t0) / steps as f64;
            let t = t0 + (i as f64 + 0.5) * dt;
            d = mat_mul(&crate::geom::rodrigues(scale(traj.rate(t), -dt)), &d);
        }
        let truth = mat_mul(&traj.pose(t1).r, &transpose(&traj.pose(t0).r));
        assert!(crate::geom::rotation_distance(&d, &truth).to_degrees() < 1e-3);
    }

    #[test]
    fn rendered_stripe_matches_the_analytic_one() {
        let scene = Scene { board: BoardDims::default(), shapes: vec![Shape::ball_on_board(144.0, 144.0, 25.0)] };
        let k: Intrinsics = CameraModel::default().into();
        let tr = Trajectory::default();
        let traj = |t: f64| tr.pose(t);
        let mut ana = Camera::new(&scene, k, LaserMount::default(), StripeModel::Analytic { noise_px: 0.0 }, 0.0, 0.0, 1);
        let mut ren = Camera::new(&scene, k, LaserMount::default(), StripeModel::render(), 0.0, 0.0, 1);
        let a = ana.observe(&traj, 1.0).laser_px;
        let r = ren.observe(&traj, 1.0).laser_px;
        assert!(a.len() > 100 && r.len() > 100, "analytic {} rendered {}", a.len(), r.len());
        // every detected point lies near the true stripe curve
        let mut dists: Vec<f64> =
            r.iter().map(|p| a.iter().map(|q| ((p[0] - q[0]) as f64).hypot((p[1] - q[1]) as f64)).fold(f64::INFINITY, f64::min)).collect();
        dists.sort_by(f64::total_cmp);
        let med = dists[dists.len() / 2];
        assert!(med < 1.0, "median distance to the true stripe {med:.2} px");
    }
}
