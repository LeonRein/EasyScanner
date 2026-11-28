//! The printed board's shape: how far it lies off a plane.
//!
//! Everyone prints their own board and lays it on whatever table they have,
//! so it is rarely flat: paper bows, curls at the edges, sags into a soft
//! mat. Every board point the pipeline uses (the corners for the pose, the
//! laser stripe for the laser-plane calibration) would be off by that much
//! if the board were taken as the plane z = 0, and the laser plane, which
//! must be good to a few hundredths of a millimetre, takes the error 1:1.
//!
//! [`BoardSurface`] models the height as a smooth polynomial over the board
//! and is measured, not assumed: [`fit`] estimates it jointly with the
//! camera poses from corner views taken at different tilts (a height only
//! shows from an oblique view), and [`SurfaceViews`] gathers such views as
//! they come. The shape belongs to one placement of the board: it holds
//! while the board lies where it is and has to be measured again when it is
//! laid down anew, which the engine notices by re-fitting recent views and
//! comparing ([`BoardSurface::max_difference`]).
//!
//! Constant, tilt and offset terms are not part of the model: a tilted or
//! shifted plane is the same board in another pose, so the poses absorb
//! them, and the surface starts at the quadratic terms.

use serde::{Deserialize, Serialize};

use crate::geom::{Intrinsics, Pose, V3, add, cross, dot, mat_mul, mat_t_vec, mat_vec, norm, normalize, rodrigues, scale, sub};
use crate::linalg::{invert, solve_dense};

/// Highest total degree of the height polynomial a surface may have.
pub const MAX_DEGREE: u8 = 6;
/// Lowest degree a measured surface has: below it there are no terms (the
/// constant and the tilt are the pose's).
pub const MIN_DEGREE: u8 = 2;

/// Height of the board over its plane, `z = h(x, y)` in the board frame
/// (OpenCV's: the camera looks along +Z, so a bump toward the camera is
/// negative), mm.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(try_from = "SurfaceFile", into = "SurfaceFile")]
pub struct BoardSurface {
    /// Board centre and half-diagonal (mm): `(x, y)` are normalised by them
    /// to `(u, v)`, with `u² + v² = 1` at the board's corners, so every
    /// coefficient is in mm.
    centre: [f64; 2],
    half_diagonal: f64,
    degree: u8,
    /// One coefficient per term `uⁱ·vʲ`, `2 ≤ i + j ≤ degree`, in
    /// [`BoardSurface::terms`] order.
    coeffs: Vec<f64>,
}

/// The on-disk shape of a [`BoardSurface`], checked on the way in.
#[derive(Clone, Debug, Serialize, Deserialize)]
struct SurfaceFile {
    centre: [f64; 2],
    half_diagonal: f64,
    degree: u8,
    coeffs: Vec<f64>,
}

impl TryFrom<SurfaceFile> for BoardSurface {
    type Error = String;

    fn try_from(f: SurfaceFile) -> Result<Self, String> {
        if f.degree > MAX_DEGREE {
            return Err(format!("surface degree {} above {MAX_DEGREE}", f.degree));
        }
        if !(f.half_diagonal.is_finite() && f.half_diagonal > 0.0) {
            return Err(format!("bad surface scale {}", f.half_diagonal));
        }
        if f.coeffs.len() != BoardSurface::n_terms(f.degree) {
            return Err(format!("{} coefficients for degree {}", f.coeffs.len(), f.degree));
        }
        if !f.coeffs.iter().chain(&f.centre).all(|c| c.is_finite()) {
            return Err("non-finite surface coefficient".into());
        }
        Ok(BoardSurface { centre: f.centre, half_diagonal: f.half_diagonal, degree: f.degree, coeffs: f.coeffs })
    }
}

impl From<BoardSurface> for SurfaceFile {
    fn from(s: BoardSurface) -> Self {
        SurfaceFile { centre: s.centre, half_diagonal: s.half_diagonal, degree: s.degree, coeffs: s.coeffs }
    }
}

impl Default for BoardSurface {
    fn default() -> Self {
        Self::flat()
    }
}

impl BoardSurface {
    /// The plane z = 0.
    pub const fn flat() -> Self {
        BoardSurface { centre: [0.0, 0.0], half_diagonal: 1.0, degree: 1, coeffs: Vec::new() }
    }

    /// A flat start for a `width`×`height` mm board with terms up to `degree`
    /// (clamped to [`MIN_DEGREE`]..=[`MAX_DEGREE`]).
    pub fn for_board(width_mm: f64, height_mm: f64, degree: u8) -> Self {
        let degree = degree.clamp(MIN_DEGREE, MAX_DEGREE);
        BoardSurface {
            centre: [0.5 * width_mm, 0.5 * height_mm],
            half_diagonal: (0.5 * width_mm.hypot(height_mm)).max(1.0),
            degree,
            coeffs: vec![0.0; Self::n_terms(degree)],
        }
    }

    /// The same board's surface with these coefficients (`None` when their
    /// number does not match the degree).
    pub fn with_coeffs(&self, coeffs: Vec<f64>) -> Option<Self> {
        (coeffs.len() == self.coeffs.len()).then(|| BoardSurface { coeffs, ..self.clone() })
    }

    /// Number of terms of degree 2 up to `degree`.
    pub fn n_terms(degree: u8) -> usize {
        let d = degree as usize;
        if d < 2 { 0 } else { (d + 1) * (d + 2) / 2 - 3 }
    }

    pub fn degree(&self) -> u8 {
        self.degree
    }

    pub fn coeffs(&self) -> &[f64] {
        &self.coeffs
    }

    pub fn is_flat(&self) -> bool {
        self.coeffs.iter().all(|&c| c == 0.0)
    }

    /// The exponents `(i, j)` of each term `uⁱ·vʲ`.
    fn terms(&self) -> impl Iterator<Item = (i32, i32)> {
        (2..=self.degree as i32).flat_map(|d| (0..=d).rev().map(move |i| (i, d - i)))
    }

    fn normalised(&self, x: f64, y: f64) -> (f64, f64) {
        ((x - self.centre[0]) / self.half_diagonal, (y - self.centre[1]) / self.half_diagonal)
    }

    /// Each term's value at board point `(x, y)`: the derivative of the
    /// height by its coefficient.
    fn basis(&self, x: f64, y: f64, out: &mut Vec<f64>) {
        let (u, v) = self.normalised(x, y);
        out.clear();
        out.extend(self.terms().map(|(i, j)| u.powi(i) * v.powi(j)));
    }

    /// Height at board point `(x, y)` (mm).
    pub fn height(&self, x: f64, y: f64) -> f64 {
        let (u, v) = self.normalised(x, y);
        self.terms().zip(&self.coeffs).map(|((i, j), c)| c * u.powi(i) * v.powi(j)).sum()
    }

    /// `(∂h/∂x, ∂h/∂y)` at board point `(x, y)`.
    pub fn gradient(&self, x: f64, y: f64) -> [f64; 2] {
        let (u, v) = self.normalised(x, y);
        let pow = |b: f64, e: i32| if e > 0 { b.powi(e - 1) * e as f64 } else { 0.0 };
        let (mut gu, mut gv) = (0.0, 0.0);
        for ((i, j), c) in self.terms().zip(&self.coeffs) {
            gu += c * pow(u, i) * v.powi(j);
            gv += c * u.powi(i) * pow(v, j);
        }
        [gu / self.half_diagonal, gv / self.half_diagonal]
    }

    /// The board point at `(x, y)`, on the surface.
    pub fn point(&self, x: f64, y: f64) -> V3 {
        [x, y, self.height(x, y)]
    }

    /// Where the camera ray `s·dir` (camera frame) meets the surface, for
    /// the board→camera `pose`: `(camera point, board point)`. `None` when
    /// the ray runs (nearly) parallel to the board, misses it, or does not
    /// converge.
    pub fn intersect(&self, pose: &Pose, dir: V3) -> Option<(V3, V3)> {
        let o = pose.camera_position();
        let d = mat_t_vec(&pose.r, dir);
        let s = self.meet(o, d)?;
        Some((scale(dir, s), add(o, scale(d, s))))
    }

    /// The ray parameter `s > 0` where the board-frame ray `o + s·d` meets
    /// the surface: Newton from the plane's intersection.
    pub fn meet(&self, o: V3, d: V3) -> Option<f64> {
        if d[2].abs() < 1e-9 {
            return None;
        }
        let at = |s: f64| add(o, scale(d, s));
        let mut s = -o[2] / d[2];
        if self.is_flat() {
            return (s > 0.0).then_some(s);
        }
        for _ in 0..8 {
            let b = at(s);
            let f = b[2] - self.height(b[0], b[1]);
            let g = self.gradient(b[0], b[1]);
            let df = d[2] - g[0] * d[0] - g[1] * d[1];
            if df.abs() < 1e-12 {
                return None;
            }
            let step = f / df;
            s -= step;
            if step.abs() < 1e-9 * s.abs().max(1.0) {
                break;
            }
        }
        let b = at(s);
        ((b[2] - self.height(b[0], b[1])).abs() < 1e-6 && s > 0.0).then_some(s)
    }

    /// Largest height difference between two surfaces over a
    /// `width`×`height` mm board, sampled on a grid. A pose absorbs any
    /// offset or tilt between them, so the best-fitting plane of the
    /// difference is removed first.
    pub fn max_difference(&self, other: &BoardSurface, width_mm: f64, height_mm: f64) -> f64 {
        let diff: Vec<V3> = grid(width_mm, height_mm).map(|(x, y)| [x, y, self.height(x, y) - other.height(x, y)]).collect();
        let Some((a, b, c)) = plane_fit(&diff) else { return 0.0 };
        diff.iter().map(|p| (p[2] - (a + b * p[0] + c * p[1])).abs()).fold(0.0, f64::max)
    }

    /// Peak-to-valley height over a `width`×`height` mm board (mm), the
    /// best-fitting plane removed.
    pub fn peak_to_valley(&self, width_mm: f64, height_mm: f64) -> f64 {
        let pts: Vec<V3> = grid(width_mm, height_mm).map(|(x, y)| self.point(x, y)).collect();
        let Some((a, b, c)) = plane_fit(&pts) else { return 0.0 };
        let res = pts.iter().map(|p| p[2] - (a + b * p[0] + c * p[1]));
        let (lo, hi) = res.fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), r| (lo.min(r), hi.max(r)));
        if lo.is_finite() { hi - lo } else { 0.0 }
    }
}

/// A 9×9 grid of sample points over a `w`×`h` mm board.
fn grid(w: f64, h: f64) -> impl Iterator<Item = (f64, f64)> {
    const N: usize = 9;
    (0..N).flat_map(move |i| (0..N).map(move |j| (w * i as f64 / (N - 1) as f64, h * j as f64 / (N - 1) as f64)))
}

/// Least-squares `z = a + b·x + c·y`.
fn plane_fit(pts: &[V3]) -> Option<(f64, f64, f64)> {
    let mut ata = vec![vec![0.0; 3]; 3];
    let mut atb = vec![0.0; 3];
    for p in pts {
        let row = [1.0, p[0], p[1]];
        for r in 0..3 {
            for c in 0..3 {
                ata[r][c] += row[r] * row[c];
            }
            atb[r] += row[r] * p[2];
        }
    }
    let s = solve_dense(ata, atb)?;
    Some((s[0], s[1], s[2]))
}

/// One view of the board for the shape fit: the detected corners' nominal
/// board positions `(x, y)` (mm), their pixels, and a starting pose.
#[derive(Clone, Debug)]
pub struct SurfaceView {
    pub xy: Vec<[f64; 2]>,
    pub px: Vec<[f64; 2]>,
    pub pose: Pose,
}

impl SurfaceView {
    /// Direction from the board centre toward the camera (board frame, unit).
    fn direction(&self, centre: [f64; 2]) -> Option<V3> {
        normalize(sub(self.pose.camera_position(), [centre[0], centre[1], 0.0]))
    }
}

/// The result of [`fit`].
#[derive(Clone, Debug)]
pub struct SurfaceFit {
    pub surface: BoardSurface,
    /// The views' poses, refined with the surface.
    pub poses: Vec<Pose>,
    /// Reprojection RMS over all corners with the fitted surface (px).
    pub rms_px: f64,
    /// … and with the board taken as flat (poses refined as well).
    pub flat_rms_px: f64,
    /// Largest standard deviation of the fitted height over the board (mm):
    /// how well the views pin the shape down.
    pub height_sigma_mm: f64,
}

/// What [`fit`] solves for.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Unknowns {
    Poses,
    PosesAndSurface,
}

/// Estimate the board's shape (the terms of `start`) jointly with the
/// views' poses by Levenberg–Marquardt on the corners' reprojection error,
/// for the camera `k`. `None` with fewer than 3 views or when the problem
/// is singular (all views from one direction cannot see a height).
pub fn fit(k: &Intrinsics, start: &BoardSurface, views: &[SurfaceView], width_mm: f64, height_mm: f64) -> Option<SurfaceFit> {
    if views.len() < 3 {
        return None;
    }
    let flat = Bundle::solve(k, &start.with_coeffs(vec![0.0; start.coeffs.len()])?, views, Unknowns::Poses)?;
    let full = Bundle::solve(k, start, views, Unknowns::PosesAndSurface)?;
    let height_sigma_mm = full.height_sigma(width_mm, height_mm)?;
    Some(SurfaceFit { rms_px: full.rms(), flat_rms_px: flat.rms(), height_sigma_mm, poses: full.poses, surface: full.surface })
}

/// Refine one pose against the board's surface: the corners' nominal
/// positions `xy`, their pixels `px`. Returns the pose and its reprojection
/// RMS (px); `None` with fewer than 4 corners or when the fit diverges.
pub fn refine_pose(k: &Intrinsics, surface: &BoardSurface, xy: &[[f64; 2]], px: &[[f64; 2]], init: Pose) -> Option<(Pose, f64)> {
    if xy.len() < 4 || xy.len() != px.len() {
        return None;
    }
    let view = SurfaceView { xy: xy.to_vec(), px: px.to_vec(), pose: init };
    let b = Bundle::solve(k, surface, std::slice::from_ref(&view), Unknowns::Poses)?;
    let rms = b.rms();
    Some((b.poses[0], rms))
}

/// Normal equations of one LM step, split as poses | surface terms.
struct Normal {
    /// Per view: `JpᵀJp` (6×6), `JpᵀJs` (6×P), `Jpᵀr` (6).
    u: Vec<[[f64; 6]; 6]>,
    w: Vec<Vec<[f64; 6]>>,
    gp: Vec<[f64; 6]>,
    /// `JsᵀJs` (P×P), `Jsᵀr` (P).
    v: Vec<Vec<f64>>,
    gs: Vec<f64>,
}

/// The joint pose and surface problem at one estimate.
struct Bundle<'a> {
    k: &'a Intrinsics,
    views: &'a [SurfaceView],
    surface: BoardSurface,
    poses: Vec<Pose>,
    cost: f64,
    n_residuals: usize,
    /// The undamped `JsᵀJs` Schur complement at the solution, for the
    /// surface's covariance.
    schur: Option<Vec<Vec<f64>>>,
}

impl<'a> Bundle<'a> {
    const ITERATIONS: usize = 30;

    fn solve(k: &'a Intrinsics, surface: &BoardSurface, views: &'a [SurfaceView], unknowns: Unknowns) -> Option<Bundle<'a>> {
        let poses: Vec<Pose> = views.iter().map(|v| v.pose).collect();
        let (initial, n_residuals) = cost(k, surface, views, &poses)?;
        let mut b = Bundle { k, views, surface: surface.clone(), poses, cost: initial, n_residuals, schur: None };
        let p = match unknowns {
            Unknowns::Poses => 0,
            Unknowns::PosesAndSurface => b.surface.coeffs.len(),
        };
        let mut lambda = 1e-3;
        for _ in 0..Self::ITERATIONS {
            let ne = b.normal(p)?;
            let mut improved = false;
            for _ in 0..8 {
                let Some((poses, surface)) = b.step(&ne, p, lambda) else {
                    lambda *= 10.0;
                    continue;
                };
                match cost(k, &surface, views, &poses) {
                    Some((c, _)) if c < b.cost => {
                        let converged = b.cost - c < 1e-10 * b.cost.max(1e-12);
                        (b.poses, b.surface, b.cost) = (poses, surface, c);
                        lambda = (lambda * 0.3).max(1e-9);
                        improved = !converged;
                        break;
                    }
                    _ => lambda *= 10.0,
                }
            }
            if !improved {
                break;
            }
        }
        if p > 0 {
            let ne = b.normal(p)?;
            b.schur = Some(schur(&ne, p, 0.0)?.0);
        }
        Some(b)
    }

    fn rms(&self) -> f64 {
        (self.cost / (self.n_residuals / 2).max(1) as f64).sqrt()
    }

    /// Largest σ of the surface height over the board, from the Schur
    /// complement's inverse scaled by the residual variance.
    fn height_sigma(&self, width_mm: f64, height_mm: f64) -> Option<f64> {
        let s = self.schur.as_ref()?;
        let cov = invert(s)?;
        let dof = self.n_residuals.saturating_sub(6 * self.poses.len() + s.len()).max(1);
        let var = self.cost / dof as f64;
        let mut basis = Vec::new();
        let mut worst: f64 = 0.0;
        for (x, y) in grid(width_mm, height_mm) {
            self.surface.basis(x, y, &mut basis);
            let q: f64 = (0..basis.len()).map(|i| (0..basis.len()).map(|j| basis[i] * cov[i][j] * basis[j]).sum::<f64>()).sum();
            worst = worst.max((var * q.max(0.0)).sqrt());
        }
        Some(worst)
    }

    /// Accumulate `JᵀJ` and `Jᵀr` for the poses and the first `p` surface terms.
    fn normal(&self, p: usize) -> Option<Normal> {
        let mut ne = Normal {
            u: vec![[[0.0; 6]; 6]; self.views.len()],
            w: vec![vec![[0.0; 6]; p]; self.views.len()],
            gp: vec![[0.0; 6]; self.views.len()],
            v: vec![vec![0.0; p]; p],
            gs: vec![0.0; p],
        };
        let mut basis = Vec::new();
        for (vi, (view, pose)) in self.views.iter().zip(&self.poses).enumerate() {
            let normal_col = [pose.r[0][2], pose.r[1][2], pose.r[2][2]];
            for (xy, px) in view.xy.iter().zip(&view.px) {
                let x = self.surface.point(xy[0], xy[1]);
                let rx = mat_vec(&pose.r, x);
                let xc = add(rx, pose.t);
                let q = self.k.project(xc)?;
                let res = [q[0] - px[0], q[1] - px[1]];
                let jp = projection_jacobian(self.k, xc, q)?;
                // ∂xc/∂ω = −[R·X]ₓ for R ← exp(ω)·R, ∂xc/∂τ = I
                let mut jpose = [[0.0; 6]; 2];
                for (row, jrow) in jp.iter().zip(jpose.iter_mut()) {
                    let jw = cross(rx, *row);
                    jrow[..3].copy_from_slice(&jw);
                    jrow[3..].copy_from_slice(row);
                }
                // ∂xc/∂cₖ = R·e_z·bₖ(x, y)
                self.surface.basis(xy[0], xy[1], &mut basis);
                let js: Vec<[f64; 2]> = basis[..p].iter().map(|b| [dot(jp[0], normal_col) * b, dot(jp[1], normal_col) * b]).collect();
                for r in 0..2 {
                    for a in 0..6 {
                        ne.gp[vi][a] += jpose[r][a] * res[r];
                        for c in 0..6 {
                            ne.u[vi][a][c] += jpose[r][a] * jpose[r][c];
                        }
                        for (s, jsk) in js.iter().enumerate() {
                            ne.w[vi][s][a] += jpose[r][a] * jsk[r];
                        }
                    }
                    for (s, jsk) in js.iter().enumerate() {
                        ne.gs[s] += jsk[r] * res[r];
                        for (t, jst) in js.iter().enumerate() {
                            ne.v[s][t] += jsk[r] * jst[r];
                        }
                    }
                }
            }
        }
        Some(ne)
    }

    /// The damped LM step: new poses and surface.
    fn step(&self, ne: &Normal, p: usize, lambda: f64) -> Option<(Vec<Pose>, BoardSurface)> {
        let ds = if p > 0 {
            let (s, rhs) = schur(ne, p, lambda)?;
            solve_dense(s, rhs)?
        } else {
            Vec::new()
        };
        let mut poses = Vec::with_capacity(self.poses.len());
        for (vi, pose) in self.poses.iter().enumerate() {
            // δp = −U⁻¹·(g + W·δs)
            let mut rhs: Vec<f64> = (0..6).map(|a| -ne.gp[vi][a]).collect();
            for (s, d) in ds.iter().enumerate() {
                for (a, r) in rhs.iter_mut().enumerate() {
                    *r -= ne.w[vi][s][a] * d;
                }
            }
            let dp = solve_dense(damped(&ne.u[vi], lambda), rhs)?;
            poses.push(Pose { r: mat_mul(&rodrigues([dp[0], dp[1], dp[2]]), &pose.r), t: add(pose.t, [dp[3], dp[4], dp[5]]) });
        }
        let coeffs: Vec<f64> = self.surface.coeffs.iter().enumerate().map(|(i, c)| c + ds.get(i).copied().unwrap_or(0.0)).collect();
        Some((poses, self.surface.with_coeffs(coeffs)?))
    }
}

/// `U + λ·diag(U)` as a dense matrix.
fn damped(u: &[[f64; 6]; 6], lambda: f64) -> Vec<Vec<f64>> {
    (0..6).map(|a| (0..6).map(|c| u[a][c] + if a == c { lambda * u[a][a].max(1e-9) } else { 0.0 }).collect()).collect()
}

/// The reduced surface system `(V − Σ WᵀU⁻¹W)·δs = −(gs − Σ WᵀU⁻¹gp)`,
/// damped by `λ`.
fn schur(ne: &Normal, p: usize, lambda: f64) -> Option<(Vec<Vec<f64>>, Vec<f64>)> {
    let mut s: Vec<Vec<f64>> =
        (0..p).map(|a| (0..p).map(|c| ne.v[a][c] + if a == c { lambda * ne.v[a][a].max(1e-9) } else { 0.0 }).collect()).collect();
    let mut rhs: Vec<f64> = ne.gs.iter().map(|g| -g).collect();
    for vi in 0..ne.u.len() {
        let uinv = invert(&damped(&ne.u[vi], lambda))?;
        // U⁻¹·W (6×P) and U⁻¹·g (6)
        let uw: Vec<[f64; 6]> = (0..p).map(|t| std::array::from_fn(|a| (0..6).map(|c| uinv[a][c] * ne.w[vi][t][c]).sum())).collect();
        let ug: [f64; 6] = std::array::from_fn(|a| (0..6).map(|c| uinv[a][c] * ne.gp[vi][c]).sum());
        for a in 0..p {
            for c in 0..p {
                s[a][c] -= (0..6).map(|i| ne.w[vi][a][i] * uw[c][i]).sum::<f64>();
            }
            rhs[a] += (0..6).map(|i| ne.w[vi][a][i] * ug[i]).sum::<f64>();
        }
    }
    Some((s, rhs))
}

/// Sum of squared reprojection residuals and their count; `None` when a
/// corner falls behind the camera.
fn cost(k: &Intrinsics, surface: &BoardSurface, views: &[SurfaceView], poses: &[Pose]) -> Option<(f64, usize)> {
    let mut c = 0.0;
    let mut n = 0;
    for (view, pose) in views.iter().zip(poses) {
        for (xy, px) in view.xy.iter().zip(&view.px) {
            let q = k.project(pose.to_cam(surface.point(xy[0], xy[1])))?;
            c += (q[0] - px[0]).powi(2) + (q[1] - px[1]).powi(2);
            n += 2;
        }
    }
    Some((c, n))
}

/// `∂(u, v)/∂xc` (2×3) of the projection at the camera point `xc`, which
/// projects to `q`, by forward differences.
fn projection_jacobian(k: &Intrinsics, xc: V3, q: [f64; 2]) -> Option<[V3; 2]> {
    let h = 1e-6 * norm(xc).max(1.0);
    let mut j = [[0.0; 3]; 2];
    for a in 0..3 {
        let mut x = xc;
        x[a] += h;
        let qa = k.project(x)?;
        j[0][a] = (qa[0] - q[0]) / h;
        j[1][a] = (qa[1] - q[1]) / h;
    }
    Some(j)
}

/// Board views collected for the shape fit while the camera moves: one per
/// viewing direction, the newest from each, the oldest dropped first. Keeping
/// the newest keeps the set current: after the board was laid down anew, the
/// directions the camera comes back to hold views of the new placement.
#[derive(Clone, Debug)]
pub struct SurfaceViews {
    centre: [f64; 2],
    views: std::collections::VecDeque<(f64, SurfaceView)>,
}

impl SurfaceViews {
    /// Views kept at most.
    pub const CAPACITY: usize = 40;
    /// Views closer than this (deg) look from the same direction…
    pub const MIN_SEPARATION_DEG: f64 = 4.0;
    /// … and a newer one replaces the kept one once that is this old (s).
    pub const REFRESH_S: f64 = 3.0;
    /// Views older than this (s) belong to an earlier placement of the board.
    pub const MAX_AGE_S: f64 = 90.0;
    /// A fit needs views from directions at least this far apart (deg)…
    pub const MIN_SPREAD_DEG: f64 = 30.0;
    /// … and this many of them.
    pub const MIN_VIEWS: usize = 12;

    /// For a `width`×`height` mm board.
    pub fn new(width_mm: f64, height_mm: f64) -> Self {
        SurfaceViews { centre: [0.5 * width_mm, 0.5 * height_mm], views: std::collections::VecDeque::new() }
    }

    pub fn len(&self) -> usize {
        self.views.len()
    }

    pub fn is_empty(&self) -> bool {
        self.views.is_empty()
    }

    pub fn clear(&mut self) {
        self.views.clear();
    }

    /// Offer the view taken at `t_s` (seconds, any monotonic clock). Returns
    /// whether it was kept.
    pub fn offer(&mut self, t_s: f64, view: SurfaceView) -> bool {
        while self.views.front().is_some_and(|(t, _)| t_s - t > Self::MAX_AGE_S || *t > t_s) {
            self.views.pop_front();
        }
        let Some(dir) = view.direction(self.centre) else { return false };
        let min_cos = Self::MIN_SEPARATION_DEG.to_radians().cos();
        let same: Vec<usize> =
            (0..self.views.len()).filter(|&i| self.views[i].1.direction(self.centre).is_some_and(|d| dot(d, dir) > min_cos)).collect();
        if same.iter().any(|&i| t_s - self.views[i].0 < Self::REFRESH_S) {
            return false;
        }
        for &i in same.iter().rev() {
            self.views.remove(i);
        }
        if self.views.len() == Self::CAPACITY {
            self.views.pop_front();
        }
        self.views.push_back((t_s, view));
        true
    }

    /// Largest angle (deg) between two kept views' directions.
    pub fn spread_deg(&self) -> f64 {
        let dirs: Vec<V3> = self.views.iter().filter_map(|(_, v)| v.direction(self.centre)).collect();
        let mut min_cos: f64 = 1.0;
        for (i, a) in dirs.iter().enumerate() {
            for b in &dirs[i + 1..] {
                min_cos = min_cos.min(dot(*a, *b));
            }
        }
        min_cos.clamp(-1.0, 1.0).acos().to_degrees()
    }

    /// Enough views, from directions far enough apart, to see the shape.
    pub fn ready(&self) -> bool {
        self.views.len() >= Self::MIN_VIEWS && self.spread_deg() >= Self::MIN_SPREAD_DEG
    }

    /// The kept views, oldest first.
    pub fn views(&self) -> Vec<SurfaceView> {
        self.views.iter().map(|(_, v)| v.clone()).collect()
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use crate::bench::sim::gauss;
    use crate::geom::rotation_distance;
    use crate::plane::Rng;

    pub(crate) const W: f64 = 288.0;
    pub(crate) const H: f64 = 288.0;

    pub(crate) fn k() -> Intrinsics {
        Intrinsics { fx: 1400.0, fy: 1400.0, cx: 959.5, cy: 719.5, dist: [0.05, -0.1, 0.0, 0.0, 0.0], width: 1920.0, height: 1440.0 }
    }

    /// A bowed sheet: 1.2 mm sag in the middle plus a 0.4 mm twist.
    pub(crate) fn warped() -> BoardSurface {
        let mut s = BoardSurface::for_board(W, H, 3);
        // terms: u², uv, v², u³, u²v, uv², v³
        s.coeffs = vec![-1.2, 0.4, -1.0, 0.0, 0.1, 0.0, -0.05];
        s
    }

    /// A camera at `dist` mm looking at the board centre from elevation
    /// `el_deg`, azimuth `az_deg`.
    pub(crate) fn view_pose(el_deg: f64, az_deg: f64, dist: f64) -> Pose {
        let (el, az) = (el_deg.to_radians(), az_deg.to_radians());
        let c = [0.5 * W, 0.5 * H, 0.0];
        let eye = add(c, scale([el.cos() * az.cos(), el.cos() * az.sin(), -el.sin()], dist));
        let fwd = normalize(sub(c, eye)).unwrap();
        let right = normalize(cross(fwd, [-az.sin(), az.cos(), 0.0])).unwrap();
        let down = cross(fwd, right);
        let r = [right, down, fwd];
        Pose { r, t: scale(mat_vec(&r, eye), -1.0) }
    }

    fn corners() -> Vec<[f64; 2]> {
        (1..8).flat_map(|i| (1..8).map(move |j| [36.0 * i as f64, 36.0 * j as f64])).collect()
    }

    pub(crate) fn observe(truth: &BoardSurface, pose: &Pose, noise_px: f64, rng: &mut Rng) -> SurfaceView {
        let mut xy = Vec::new();
        let mut px = Vec::new();
        for c in corners() {
            if let Some(q) = k().project(pose.to_cam(truth.point(c[0], c[1]))) {
                xy.push(c);
                px.push([q[0] + noise_px * gauss(rng), q[1] + noise_px * gauss(rng)]);
            }
        }
        // the starting pose a flat-board PnP would give: off by a bit
        let start = Pose { r: mat_mul(&rodrigues([0.004, -0.003, 0.002]), &pose.r), t: add(pose.t, [1.0, -0.8, 2.0]) };
        SurfaceView { xy, px, pose: start }
    }

    fn views(truth: &BoardSurface, noise_px: f64) -> Vec<SurfaceView> {
        let mut rng = Rng::new(7);
        let mut out = Vec::new();
        for (i, el) in [45.0, 55.0, 65.0, 75.0].into_iter().enumerate() {
            for az in (0..6).map(|a| 60.0 * a as f64 + 15.0 * i as f64) {
                out.push(observe(truth, &view_pose(el, az, 420.0), noise_px, &mut rng));
            }
        }
        out
    }

    #[test]
    fn terms_and_derivatives() {
        assert_eq!(BoardSurface::n_terms(1), 0);
        assert_eq!(BoardSurface::n_terms(2), 3);
        assert_eq!(BoardSurface::n_terms(3), 7);
        let s = warped();
        let (x, y, h) = (70.0, 190.0, 1e-5);
        let g = s.gradient(x, y);
        assert!((g[0] - (s.height(x + h, y) - s.height(x - h, y)) / (2.0 * h)).abs() < 1e-7);
        assert!((g[1] - (s.height(x, y + h) - s.height(x, y - h)) / (2.0 * h)).abs() < 1e-7);
        assert!(BoardSurface::flat().is_flat() && BoardSurface::flat().height(10.0, 20.0) == 0.0);
    }

    #[test]
    fn rays_meet_the_surface() {
        let s = warped();
        let pose = view_pose(50.0, 30.0, 400.0);
        for b in [[30.0, 40.0], [144.0, 144.0], [250.0, 200.0]] {
            let p = s.point(b[0], b[1]);
            let xc = pose.to_cam(p);
            let (cam, board) = s.intersect(&pose, scale(xc, 1.0 / xc[2])).unwrap();
            assert!(norm(sub(board, p)) < 1e-6 && norm(sub(cam, xc)) < 1e-6);
        }
    }

    #[test]
    fn a_warped_board_is_measured_from_oblique_views() {
        let truth = warped();
        let v = views(&truth, 0.15);
        let fit = fit(&k(), &BoardSurface::for_board(W, H, 3), &v, W, H).unwrap();
        let err = fit.surface.max_difference(&truth, W, H);
        assert!(err < 0.1, "shape off by {err:.3} mm (p-v {:.2} mm)", truth.peak_to_valley(W, H));
        // 0.15 px per axis is 0.21 px per corner: the fit is down at the noise,
        // a flat board is not
        let noise = 0.15 * std::f64::consts::SQRT_2;
        assert!(fit.rms_px < 1.1 * noise && fit.flat_rms_px > 1.5 * fit.rms_px, "rms {:.3} flat {:.3}", fit.rms_px, fit.flat_rms_px);
        assert!(fit.height_sigma_mm < 0.1, "sigma {:.3}", fit.height_sigma_mm);
    }

    #[test]
    fn a_flat_board_stays_flat() {
        let v = views(&BoardSurface::flat(), 0.15);
        let fit = fit(&k(), &BoardSurface::for_board(W, H, 3), &v, W, H).unwrap();
        assert!(fit.surface.peak_to_valley(W, H) < 0.1, "p-v {:.3}", fit.surface.peak_to_valley(W, H));
    }

    #[test]
    fn the_pose_follows_the_surface() {
        let truth = warped();
        let pose = view_pose(55.0, 200.0, 380.0);
        let v = observe(&truth, &pose, 0.0, &mut Rng::new(1));
        let (with, rms) = refine_pose(&k(), &truth, &v.xy, &v.px, v.pose).unwrap();
        assert!(rms < 1e-6 && rotation_distance(&with.r, &pose.r) < 1e-8 && norm(sub(with.t, pose.t)) < 1e-5);
        let (_, flat_rms) = refine_pose(&k(), &BoardSurface::flat(), &v.xy, &v.px, v.pose).unwrap();
        assert!(flat_rms > 0.3, "a flat model misfits the warped board: {flat_rms:.3} px");
    }

    #[test]
    fn views_are_collected_by_direction() {
        let mut c = SurfaceViews::new(W, H);
        let mut rng = Rng::new(3);
        let flat = BoardSurface::flat();
        assert!(c.offer(0.0, observe(&flat, &view_pose(60.0, 0.0, 400.0), 0.0, &mut rng)));
        assert!(!c.offer(0.1, observe(&flat, &view_pose(61.0, 1.0, 420.0), 0.0, &mut rng)), "same direction");
        assert!(c.offer(0.2, observe(&flat, &view_pose(60.0, 90.0, 400.0), 0.0, &mut rng)));
        assert!(!c.ready());
        // a later view from a covered direction replaces the old one
        assert!(c.offer(SurfaceViews::REFRESH_S + 0.1, observe(&flat, &view_pose(61.0, 1.0, 420.0), 0.0, &mut rng)));
        assert_eq!(c.len(), 2);
        // an old view is dropped
        assert!(c.offer(SurfaceViews::MAX_AGE_S + 0.25, observe(&flat, &view_pose(45.0, 180.0, 400.0), 0.0, &mut rng)));
        assert_eq!(c.len(), 2);
    }
}
