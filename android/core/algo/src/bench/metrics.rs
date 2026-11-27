//! Error metrics: against the ground truth of a synthetic recording, and
//! reference-object fits (a sphere of known size) that also work on real
//! scans where no ground truth exists.

use std::io::{self, BufRead, Read};
use std::path::Path;

use serde::Serialize;

use crate::geom::{V3, dot, norm, rotation_distance, sub};

use crate::imu::MountSource;

use super::pnp::{pose_error, solve_dense};
use super::reconstruct::Reconstruction;
use super::recording::Recording;
use super::scene::Shape;

/// Summary of a signed error sample.
#[derive(Clone, Copy, Debug, Default, Serialize)]
pub struct Stats {
    pub n: usize,
    pub mean: f64,
    pub std: f64,
    pub rms: f64,
    pub p50_abs: f64,
    pub p95_abs: f64,
    pub max_abs: f64,
}

impl Stats {
    pub fn of(xs: &[f64]) -> Option<Stats> {
        let xs: Vec<f64> = xs.iter().copied().filter(|x| x.is_finite()).collect();
        if xs.is_empty() {
            return None;
        }
        let n = xs.len() as f64;
        let mean = xs.iter().sum::<f64>() / n;
        let rms = (xs.iter().map(|x| x * x).sum::<f64>() / n).sqrt();
        let std = (xs.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / n).sqrt();
        let mut a: Vec<f64> = xs.iter().map(|x| x.abs()).collect();
        a.sort_by(f64::total_cmp);
        let q = |p: f64| a[((a.len() - 1) as f64 * p).round() as usize];
        Some(Stats { n: xs.len(), mean, std, rms, p50_abs: q(0.5), p95_abs: q(0.95), max_abs: a[a.len() - 1] })
    }
}

#[derive(Clone, Copy, Debug, Serialize)]
pub struct SphereFit {
    pub center: V3,
    pub radius: f64,
    /// Radial residual RMS of the points kept.
    pub rms: f64,
    pub n: usize,
    /// Points dropped as outliers.
    pub rejected: usize,
}

/// Least-squares sphere (algebraic start, geometric Gauss–Newton refinement)
/// with iterative outlier rejection (3 × robust σ, at least 0.1 mm).
pub fn fit_sphere(pts: &[V3]) -> Option<SphereFit> {
    let mut keep: Vec<V3> = pts.to_vec();
    let mut fit = None;
    for _ in 0..4 {
        let (c, r) = sphere_ls(&keep)?;
        let res: Vec<f64> = pts.iter().map(|p| norm(sub(*p, c)) - r).collect();
        let mut a: Vec<f64> = res.iter().map(|x| x.abs()).collect();
        a.sort_by(f64::total_cmp);
        let sigma = 1.4826 * a[a.len() / 2];
        let lim = (3.0 * sigma).max(0.1);
        let next: Vec<V3> = pts.iter().zip(&res).filter(|(_, e)| e.abs() <= lim).map(|(p, _)| *p).collect();
        let kept_res: Vec<f64> = res.iter().copied().filter(|e| e.abs() <= lim).collect();
        let rms = (kept_res.iter().map(|e| e * e).sum::<f64>() / kept_res.len().max(1) as f64).sqrt();
        fit = Some(SphereFit { center: c, radius: r, rms, n: next.len(), rejected: pts.len() - next.len() });
        if next.len() == keep.len() || next.len() < 4 {
            break;
        }
        keep = next;
    }
    fit
}

fn sphere_ls(pts: &[V3]) -> Option<(V3, f64)> {
    if pts.len() < 4 {
        return None;
    }
    // algebraic: x²+y²+z² + a·x + b·y + c·z + e = 0, centred for conditioning
    let n = pts.len() as f64;
    let m: V3 = std::array::from_fn(|i| pts.iter().map(|p| p[i]).sum::<f64>() / n);
    let mut ata = vec![vec![0.0; 4]; 4];
    let mut atb = vec![0.0; 4];
    for p in pts {
        let q = sub(*p, m);
        let row = [q[0], q[1], q[2], 1.0];
        let rhs = -dot(q, q);
        for i in 0..4 {
            for j in 0..4 {
                ata[i][j] += row[i] * row[j];
            }
            atb[i] += row[i] * rhs;
        }
    }
    let s = solve_dense(ata, atb)?;
    let mut c = [-0.5 * s[0], -0.5 * s[1], -0.5 * s[2]];
    let mut r = (dot(c, c) - s[3]).max(0.0).sqrt();
    // geometric refinement: residual |p − c| − r
    for _ in 0..20 {
        let mut jtj = vec![vec![0.0; 4]; 4];
        let mut jtr = vec![0.0; 4];
        for p in pts {
            let d = sub(sub(*p, m), c);
            let l = norm(d).max(1e-12);
            let e = l - r;
            let j = [-d[0] / l, -d[1] / l, -d[2] / l, -1.0];
            for a in 0..4 {
                jtr[a] -= j[a] * e;
                for b in 0..4 {
                    jtj[a][b] += j[a] * j[b];
                }
            }
        }
        let Some(dx) = solve_dense(jtj, jtr) else { break };
        c = [c[0] + dx[0], c[1] + dx[1], c[2] + dx[2]];
        r += dx[3];
        if dx.iter().map(|v| v.abs()).fold(0.0, f64::max) < 1e-9 {
            break;
        }
    }
    Some((crate::geom::add(c, m), r))
}

#[derive(Clone, Debug, Serialize)]
pub struct SphereReport {
    /// Index into the truth scene's shapes.
    pub shape: usize,
    pub true_radius: f64,
    pub fit: Option<SphereFit>,
    pub radius_err_mm: Option<f64>,
    pub center_err_mm: Option<f64>,
}

/// The gyro self-calibration at the end of the reconstruction.
#[derive(Clone, Copy, Debug, Serialize)]
pub struct GyroReport {
    /// First frame the calibration was in use.
    pub from_frame: Option<usize>,
    /// Its fit residual (deg).
    pub residual_deg: f64,
    /// The standard error of the mount if it was fitted, `None` if it is the
    /// platform's (deg).
    pub mount_sigma_deg: Option<f64>,
    /// Errors against the simulated gyro (synthetic recordings only).
    pub rotation_err_deg: Option<f64>,
    pub offset_err_ms: Option<f64>,
}

#[derive(Clone, Debug, Default, Serialize)]
pub struct Report {
    pub name: String,
    pub frames: usize,
    pub frames_posed: usize,
    pub points: usize,
    /// Reprojection RMS of the solved poses (px).
    pub reproj_px: Option<Stats>,
    /// Frames whose observations were rolling-shutter corrected.
    pub corrected_frames: usize,
    /// `None` when the recording has no gyro or it never calibrated.
    pub gyro: Option<GyroReport>,
    /// Pose error against the truth at the frame's reference instant.
    pub pose_rot_deg: Option<Stats>,
    pub pose_pos_mm: Option<Stats>,
    /// Angle between the used and the true laser plane.
    pub plane_angle_deg: Option<f64>,
    /// Offset between the used and the true plane at the sheet's axis crossing.
    pub plane_offset_mm: Option<f64>,
    /// Signed distance of each point to the nearest true surface (mm).
    pub surface_mm: Option<Stats>,
    /// Fraction of points further than 1 mm from any true surface.
    pub outliers_1mm: Option<f64>,
    pub spheres: Vec<SphereReport>,
    pub runtime_s: f64,
}

/// Everything measurable about a reconstruction; ground-truth metrics only
/// when the recording carries a truth.
pub fn evaluate(name: &str, rec: &Recording, recon: &Reconstruction) -> Report {
    let mut rep = Report {
        name: name.to_string(),
        frames: rec.frames.len(),
        frames_posed: recon.poses.iter().filter(|p| p.is_some()).count(),
        points: recon.points.len(),
        corrected_frames: recon.corrected_frames,
        ..Report::default()
    };
    let rms: Vec<f64> = recon.poses.iter().flatten().map(|p| p.reproj_rms).collect();
    rep.reproj_px = Stats::of(&rms);
    let true_gyro = rec.truth.as_ref().and_then(|t| t.gyro);
    rep.gyro = recon.gyro_fit.map(|fit| {
        let c = fit.calib;
        GyroReport {
            from_frame: recon.gyro_from_frame,
            residual_deg: fit.residual_deg,
            mount_sigma_deg: match fit.mount {
                MountSource::Platform => None,
                MountSource::Fitted { sigma_deg } => Some(sigma_deg),
            },
            rotation_err_deg: true_gyro.map(|t| rotation_distance(&c.cam_from_imu, &t.cam_from_imu).to_degrees()),
            offset_err_ms: true_gyro.map(|t| (c.offset_ns - t.offset_ns) * 1e-6),
        }
    });
    let Some(truth) = &rec.truth else { return rep };

    let (mut rot, mut pos) = (Vec::new(), Vec::new());
    for (est, tp) in recon.poses.iter().zip(&truth.poses) {
        if let Some(e) = est {
            let (r, p) = pose_error(&e.pose, &(*tp).into());
            rot.push(r);
            pos.push(p);
        }
    }
    rep.pose_rot_deg = Stats::of(&rot);
    rep.pose_pos_mm = Stats::of(&pos);

    let (tn, td) = (truth.laser_plane.n, truth.laser_plane.d);
    let (un, ud) = (rec.header.laser_plane.n, rec.header.laser_plane.d);
    rep.plane_angle_deg = Some(dot(tn, un).abs().min(1.0).acos().to_degrees());
    // the true sheet's point on the optical axis, measured against the used plane
    if tn[2].abs() > 1e-9 {
        let p = [0.0, 0.0, td / tn[2]];
        rep.plane_offset_mm = Some((dot(un, p) - ud).abs());
    }

    let mut errs = Vec::with_capacity(recon.points.len());
    let mut per_shape: Vec<Vec<V3>> = vec![Vec::new(); truth.scene.shapes.len()];
    for p in &recon.points {
        if let Some((e, i)) = truth.scene.nearest_shape(*p) {
            errs.push(e);
            if e.abs() < 5.0 {
                per_shape[i].push(*p);
            }
        }
    }
    rep.surface_mm = Stats::of(&errs);
    if !errs.is_empty() {
        rep.outliers_1mm = Some(errs.iter().filter(|e| e.abs() > 1.0).count() as f64 / errs.len() as f64);
    }
    for (i, sh) in truth.scene.shapes.iter().enumerate() {
        if let Shape::Sphere { center, radius } = *sh {
            let fit = fit_sphere(&per_shape[i]);
            rep.spheres.push(SphereReport {
                shape: i,
                true_radius: radius,
                radius_err_mm: fit.map(|f| f.radius - radius),
                center_err_mm: fit.map(|f| norm(sub(f.center, center))),
                fit,
            });
        }
    }
    rep
}

/// Read the vertices of a PLY file (ASCII or binary little-endian, as the
/// app exports it).
pub fn read_ply(path: &Path) -> io::Result<Vec<V3>> {
    let mut r = io::BufReader::new(std::fs::File::open(path)?);
    let bad = |m: &str| io::Error::new(io::ErrorKind::InvalidData, m.to_string());
    let mut line = String::new();
    let mut format = String::new();
    let mut n = 0usize;
    let mut props: Vec<(String, String)> = Vec::new();
    let mut in_vertex = false;
    loop {
        line.clear();
        if r.read_line(&mut line)? == 0 {
            return Err(bad("PLY header ended early"));
        }
        let w: Vec<&str> = line.split_whitespace().collect();
        match w.as_slice() {
            ["format", f, ..] => format = f.to_string(),
            ["element", "vertex", c] => {
                n = c.parse().map_err(|_| bad("vertex count"))?;
                in_vertex = true;
            }
            ["element", ..] => in_vertex = false,
            ["property", "list", ..] if in_vertex => return Err(bad("list property on vertices")),
            ["property", t, name] if in_vertex => props.push((t.to_string(), name.to_string())),
            ["end_header"] => break,
            _ => {}
        }
    }
    let idx = |name: &str| props.iter().position(|(_, n)| n == name).ok_or_else(|| bad("no x/y/z"));
    let (ix, iy, iz) = (idx("x")?, idx("y")?, idx("z")?);
    let mut out = Vec::with_capacity(n);
    if format == "ascii" {
        for _ in 0..n {
            line.clear();
            r.read_line(&mut line)?;
            let v: Vec<f64> = line.split_whitespace().map(|s| s.parse().unwrap_or(f64::NAN)).collect();
            if v.len() < props.len() {
                return Err(bad("short vertex line"));
            }
            out.push([v[ix], v[iy], v[iz]]);
        }
        return Ok(out);
    }
    if format != "binary_little_endian" {
        return Err(bad("only ascii and binary_little_endian PLY"));
    }
    let size = |t: &str| match t {
        "char" | "uchar" | "int8" | "uint8" => Ok(1),
        "short" | "ushort" | "int16" | "uint16" => Ok(2),
        "int" | "uint" | "float" | "int32" | "uint32" | "float32" => Ok(4),
        "double" | "float64" => Ok(8),
        _ => Err(bad("unknown property type")),
    };
    let sizes: Vec<usize> = props.iter().map(|(t, _)| size(t)).collect::<io::Result<_>>()?;
    let stride: usize = sizes.iter().sum();
    let offs: Vec<usize> = sizes
        .iter()
        .scan(0, |a, s| {
            let o = *a;
            *a += s;
            Some(o)
        })
        .collect();
    let get = |rec: &[u8], i: usize| -> f64 {
        let o = offs[i];
        match props[i].0.as_str() {
            "double" | "float64" => f64::from_le_bytes(rec[o..o + 8].try_into().unwrap()),
            _ => f32::from_le_bytes(rec[o..o + 4].try_into().unwrap()) as f64,
        }
    };
    let mut buf = vec![0u8; stride];
    for _ in 0..n {
        r.read_exact(&mut buf)?;
        out.push([get(&buf, ix), get(&buf, iy), get(&buf, iz)]);
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bench::sim::gauss;
    use crate::plane::Rng;

    #[test]
    fn sphere_fit_recovers_a_noisy_cap_with_outliers() {
        let mut rng = Rng::new(5);
        let (c, r) = ([10.0, -20.0, -25.0], 25.0);
        let mut pts = Vec::new();
        for i in 0..2000 {
            // upper cap only, like a scan of a ball on the board
            let th = 0.2 + 1.2 * (i as f64 / 2000.0);
            let ph = i as f64 * 2.399;
            let dir = [th.sin() * ph.cos(), th.sin() * ph.sin(), -th.cos()];
            let rr = r + 0.05 * gauss(&mut rng);
            pts.push([c[0] + rr * dir[0], c[1] + rr * dir[1], c[2] + rr * dir[2]]);
        }
        for i in 0..50 {
            pts.push([i as f64, 0.0, -80.0]);
        }
        let f = fit_sphere(&pts).unwrap();
        assert!((f.radius - r).abs() < 0.02, "r {}", f.radius);
        assert!(norm(sub(f.center, c)) < 0.05);
        assert!(f.rejected >= 50);
    }

    #[test]
    fn stats_basics() {
        let s = Stats::of(&[1.0, -1.0, 2.0, -2.0]).unwrap();
        assert_eq!(s.n, 4);
        assert!((s.mean).abs() < 1e-12);
        assert!((s.rms - 2.5f64.sqrt()).abs() < 1e-12);
        assert_eq!(s.max_abs, 2.0);
    }

    #[test]
    fn reads_the_apps_ply() {
        let mut c = crate::cloud::PointCloud::new(10);
        c.push([1.0, 2.0, 3.0], [1, 2, 3]);
        c.push([-4.0, 5.5, 6.0], [1, 2, 3]);
        let dir = std::env::temp_dir().join(format!("scan_bench_ply_{}", std::process::id()));
        let mut buf = Vec::new();
        c.write_ply(&mut buf).unwrap();
        std::fs::write(&dir, buf).unwrap();
        let pts = read_ply(&dir).unwrap();
        std::fs::remove_file(&dir).ok();
        assert_eq!(pts, vec![[1.0, 2.0, 3.0], [-4.0, 5.5, 6.0]]);
    }
}
