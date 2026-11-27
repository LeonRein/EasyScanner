//! Planar PnP in pure Rust: homography initialisation, then Levenberg–
//! Marquardt on the full reprojection error through the distortion model.
//!
//! The app gets its pose from OpenCV (`solvePnPGeneric` with IPPE in
//! `easyscanner-core/src/board.rs`). This solver reaches the same
//! least-squares optimum for a planar target, so the benchmark measures how
//! corner noise, occlusion and rolling shutter propagate into the pose
//! without needing OpenCV on the host. It does not model IPPE's flip
//! ambiguity.

use crate::geom::{Intrinsics, M3, Pose, V3, mat_mul, rodrigues, sym3_eigen, transpose};

/// Solve the dense `n×n` system `a·x = b` by Gaussian elimination with
/// partial pivoting. `None` when singular.
#[allow(clippy::needless_range_loop)]
pub fn solve_dense(mut a: Vec<Vec<f64>>, mut b: Vec<f64>) -> Option<Vec<f64>> {
    let n = b.len();
    for c in 0..n {
        let p = (c..n).max_by(|&i, &j| a[i][c].abs().total_cmp(&a[j][c].abs()))?;
        if a[p][c].abs() < 1e-300 {
            return None;
        }
        a.swap(c, p);
        b.swap(c, p);
        for r in c + 1..n {
            let f = a[r][c] / a[c][c];
            if f != 0.0 {
                for k in c..n {
                    a[r][k] -= f * a[c][k];
                }
                b[r] -= f * b[c];
            }
        }
    }
    let mut x = vec![0.0; n];
    for r in (0..n).rev() {
        let s: f64 = (r + 1..n).map(|k| a[r][k] * x[k]).sum();
        x[r] = (b[r] - s) / a[r][r];
    }
    x.iter().all(|v| v.is_finite()).then_some(x)
}

/// Similarity that maps points to zero mean and √2 mean distance (Hartley).
fn normaliser(pts: impl Iterator<Item = [f64; 2]> + Clone) -> (f64, f64, f64) {
    let n = pts.clone().count() as f64;
    let (mx, my) = pts.clone().fold((0.0, 0.0), |a, p| (a.0 + p[0] / n, a.1 + p[1] / n));
    let md = pts.map(|p| ((p[0] - mx).powi(2) + (p[1] - my).powi(2)).sqrt()).sum::<f64>() / n;
    (mx, my, if md > 1e-12 { std::f64::consts::SQRT_2 / md } else { 1.0 })
}

/// Homography `H` (h33 = 1) with `[x y 1]ᵀ ~ H·[X Y 1]ᵀ`, by normalised DLT.
fn homography(obj: &[[f64; 2]], img: &[[f64; 2]]) -> Option<M3> {
    let (ox, oy, os) = normaliser(obj.iter().copied());
    let (ix, iy, is) = normaliser(img.iter().copied());
    let mut ata = vec![vec![0.0; 8]; 8];
    let mut atb = vec![0.0; 8];
    for (o, i) in obj.iter().zip(img) {
        let (x, y) = ((o[0] - ox) * os, (o[1] - oy) * os);
        let (u, v) = ((i[0] - ix) * is, (i[1] - iy) * is);
        for (row, rhs) in [([x, y, 1.0, 0.0, 0.0, 0.0, -u * x, -u * y], u), ([0.0, 0.0, 0.0, x, y, 1.0, -v * x, -v * y], v)] {
            for r in 0..8 {
                for c in 0..8 {
                    ata[r][c] += row[r] * row[c];
                }
                atb[r] += row[r] * rhs;
            }
        }
    }
    let h = solve_dense(ata, atb)?;
    let hn: M3 = [[h[0], h[1], h[2]], [h[3], h[4], h[5]], [h[6], h[7], 1.0]];
    // undo the normalisations: H = Ti⁻¹·Hn·To
    let to: M3 = [[os, 0.0, -os * ox], [0.0, os, -os * oy], [0.0, 0.0, 1.0]];
    let ti_inv: M3 = [[1.0 / is, 0.0, ix], [0.0, 1.0 / is, iy], [0.0, 0.0, 1.0]];
    Some(mat_mul(&ti_inv, &mat_mul(&hn, &to)))
}

/// Nearest rotation to `m` (polar decomposition `m·(mᵀm)^-½`).
fn nearest_rotation(m: &M3) -> M3 {
    let mtm = mat_mul(&transpose(m), m);
    let (vals, vecs) = sym3_eigen(&mtm);
    let mut inv_sqrt = [[0.0; 3]; 3];
    for k in 0..3 {
        let w = 1.0 / vals[k].max(1e-300).sqrt();
        for i in 0..3 {
            for j in 0..3 {
                inv_sqrt[i][j] += w * vecs[k][i] * vecs[k][j];
            }
        }
    }
    mat_mul(m, &inv_sqrt)
}

/// Initial pose from the homography of a z = 0 target.
fn pose_from_homography(h: &M3) -> Option<Pose> {
    let col = |j: usize| -> V3 { [h[0][j], h[1][j], h[2][j]] };
    let (h1, h2, h3) = (col(0), col(1), col(2));
    let n1 = crate::geom::norm(h1);
    let n2 = crate::geom::norm(h2);
    if n1 < 1e-12 || n2 < 1e-12 {
        return None;
    }
    let mut lam = 2.0 / (n1 + n2);
    if h3[2] * lam < 0.0 {
        lam = -lam; // the board is in front of the camera
    }
    let r1 = crate::geom::scale(h1, lam);
    let r2 = crate::geom::scale(h2, lam);
    let r3 = crate::geom::cross(r1, r2);
    let m: M3 = [[r1[0], r2[0], r3[0]], [r1[1], r2[1], r3[1]], [r1[2], r2[2], r3[2]]];
    Some(Pose { r: nearest_rotation(&m), t: crate::geom::scale(h3, lam) })
}

/// Board→camera pose from ≥ 4 board points (z = 0) and their pixels, with
/// the RMS reprojection error (px).
pub fn solve_planar(k: &Intrinsics, obj: &[V3], px: &[[f64; 2]]) -> Option<(Pose, f64)> {
    if obj.len() < 4 || obj.len() != px.len() {
        return None;
    }
    let o2: Vec<[f64; 2]> = obj.iter().map(|p| [p[0], p[1]]).collect();
    let norm_px: Vec<[f64; 2]> = px.iter().map(|p| k.undistort(p[0], p[1])).collect();
    let init = pose_from_homography(&homography(&o2, &norm_px)?)?;
    let pose = refine(k, obj, px, init, 30);
    let rms = reproj_rms(k, obj, px, &pose)?;
    Some((pose, rms))
}

pub fn reproj_rms(k: &Intrinsics, obj: &[V3], px: &[[f64; 2]], pose: &Pose) -> Option<f64> {
    let mut ss = 0.0;
    for (o, p) in obj.iter().zip(px) {
        let q = k.project(pose.to_cam(*o))?;
        ss += (q[0] - p[0]).powi(2) + (q[1] - p[1]).powi(2);
    }
    Some((ss / obj.len() as f64).sqrt())
}

fn residuals(k: &Intrinsics, obj: &[V3], px: &[[f64; 2]], pose: &Pose, out: &mut Vec<f64>) -> bool {
    out.clear();
    for (o, p) in obj.iter().zip(px) {
        let Some(q) = k.project(pose.to_cam(*o)) else { return false };
        out.push(q[0] - p[0]);
        out.push(q[1] - p[1]);
    }
    true
}

/// `pose` updated by `δ = (ω, τ)`: `R ← exp(ω)·R`, `t ← t + τ`.
fn apply(pose: &Pose, d: &[f64]) -> Pose {
    Pose { r: mat_mul(&rodrigues([d[0], d[1], d[2]]), &pose.r), t: [pose.t[0] + d[3], pose.t[1] + d[4], pose.t[2] + d[5]] }
}

/// Levenberg–Marquardt on the reprojection error (numeric Jacobian).
pub fn refine(k: &Intrinsics, obj: &[V3], px: &[[f64; 2]], init: Pose, iters: usize) -> Pose {
    let mut pose = init;
    let mut r0 = Vec::new();
    if !residuals(k, obj, px, &pose, &mut r0) {
        return pose;
    }
    let mut cost: f64 = r0.iter().map(|v| v * v).sum();
    let mut lambda = 1e-3;
    let mut r1 = Vec::new();
    let m = r0.len();
    let mut jac = vec![[0.0; 6]; m];
    for _ in 0..iters {
        for p in 0..6 {
            let h = if p < 3 { 1e-7 } else { 1e-5 };
            let mut d = [0.0; 6];
            d[p] = h;
            if !residuals(k, obj, px, &apply(&pose, &d), &mut r1) {
                return pose;
            }
            for i in 0..m {
                jac[i][p] = (r1[i] - r0[i]) / h;
            }
        }
        let mut jtj = vec![vec![0.0; 6]; 6];
        let mut jtr = vec![0.0; 6];
        for i in 0..m {
            for a in 0..6 {
                jtr[a] -= jac[i][a] * r0[i];
                for b in 0..6 {
                    jtj[a][b] += jac[i][a] * jac[i][b];
                }
            }
        }
        let mut improved = false;
        for _ in 0..8 {
            let mut a = jtj.clone();
            for (i, row) in a.iter_mut().enumerate() {
                row[i] += lambda * jtj[i][i].max(1e-12);
            }
            let Some(d) = solve_dense(a, jtr.clone()) else { break };
            let cand = apply(&pose, &d);
            if residuals(k, obj, px, &cand, &mut r1) {
                let c: f64 = r1.iter().map(|v| v * v).sum();
                if c < cost {
                    let small = d.iter().map(|v| v.abs()).fold(0.0, f64::max) < 1e-10;
                    pose = cand;
                    std::mem::swap(&mut r0, &mut r1);
                    cost = c;
                    lambda = (lambda * 0.3).max(1e-9);
                    improved = !small;
                    break;
                }
            }
            lambda *= 10.0;
        }
        if !improved {
            break;
        }
    }
    pose
}

/// Rotation angle (deg) and camera-centre distance (mm) between two poses.
pub fn pose_error(est: &Pose, truth: &Pose) -> (f64, f64) {
    let rot = crate::geom::rotation_distance(&est.r, &truth.r).to_degrees();
    let pos = crate::geom::norm(crate::geom::sub(est.camera_position(), truth.camera_position()));
    (rot, pos)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bench::scene::BoardDims;

    fn k() -> Intrinsics {
        Intrinsics { fx: 920.0, fy: 925.0, cx: 641.0, cy: 478.0, dist: [0.06, -0.12, 0.001, -0.0005, 0.02], width: 1280.0, height: 960.0 }
    }

    #[test]
    fn dense_solver() {
        let x = solve_dense(vec![vec![2.0, 1.0], vec![1.0, 3.0]], vec![3.0, 5.0]).unwrap();
        assert!((x[0] - 0.8).abs() < 1e-12 && (x[1] - 1.4).abs() < 1e-12);
    }

    #[test]
    fn recovers_an_oblique_pose_exactly() {
        let k = k();
        let truth = Pose { r: rodrigues([2.7, 0.35, -0.2]), t: [-130.0, 80.0, 420.0] };
        let (obj, px): (Vec<V3>, Vec<[f64; 2]>) =
            BoardDims::default().corners().filter_map(|(_, p)| k.project(truth.to_cam(p)).map(|q| (p, q))).unzip();
        assert!(obj.len() > 30);
        let (est, rms) = solve_planar(&k, &obj, &px).unwrap();
        let (rot, pos) = pose_error(&est, &truth);
        assert!(rms < 1e-6, "rms {rms}");
        assert!(rot < 1e-6 && pos < 1e-4, "rot {rot} pos {pos}");
    }
}
