//! Scan-accuracy benchmark (see `scan_algo::bench`).
//!
//! ```text
//! scan_bench sweep [--fast] [--seconds S] [--json]
//!     every scenario of the error budget, one table
//! scan_bench run [--config cfg.json] [--realistic] [--fast] [--seconds S]
//!                [--save rec.json] [--ply out.ply] [--json]
//!     one scenario; --save keeps the recording, --ply the cloud
//! scan_bench eval rec.json [--json]
//!     reconstruct and score a saved recording (synthetic or, later, real)
//! scan_bench eval-ply scan.ply [--radius R]
//!     sphere fit of a scanned reference ball (radius error if R is given)
//! scan_bench config [--realistic]
//!     print a scenario config to edit for --config
//! ```
//!
//! `--fast` swaps the rendered stripe for the analytic one with 0.3 px noise.
//! Run it with `cargo run --release -p scan-algo --bin scan_bench -- sweep`.

use std::path::Path;
use std::process::ExitCode;

use scan_algo::bench::metrics::{self, Report, Stats};
use scan_algo::bench::reconstruct::{self, PoseFrom};
use scan_algo::bench::recording::Recording;
use scan_algo::bench::sim::StripeModel;
use scan_algo::bench::{self, BenchCfg};
use scan_algo::cloud::PointCloud;
use scan_algo::fusion::FusionCfg;

struct Args {
    rest: Vec<String>,
}

impl Args {
    fn flag(&mut self, name: &str) -> bool {
        let n = self.rest.len();
        self.rest.retain(|a| a != name);
        self.rest.len() != n
    }

    fn value(&mut self, name: &str) -> Option<String> {
        let i = self.rest.iter().position(|a| a == name)?;
        if i + 1 >= self.rest.len() {
            return None;
        }
        let v = self.rest.remove(i + 1);
        self.rest.remove(i);
        Some(v)
    }
}

fn adjust(cfg: &mut BenchCfg, fast: bool, seconds: Option<f64>) {
    if fast && matches!(cfg.stripe, StripeModel::Render { .. }) {
        cfg.stripe = StripeModel::Analytic { noise_px: 0.3 };
    }
    if let Some(s) = seconds {
        let t = &mut cfg.trajectory;
        t.azimuth_span_deg *= s / t.duration_s;
        t.duration_s = s;
    }
}

fn f(x: Option<f64>, digits: usize) -> String {
    x.filter(|v| v.is_finite()).map_or("-".into(), |v| format!("{v:.digits$}"))
}

fn table(reports: &[Report]) {
    println!(
        "{:<42} {:>9} {:>7} {:>7} {:>6} {:>6} {:>7} {:>7} {:>7} {:>7} {:>7} {:>7} {:>6} {:>8} {:>7}",
        "scenario",
        "posed",
        "points",
        "pos mm",
        "rot °",
        "plane°",
        "mean",
        "median",
        "p95",
        "rms",
        ">1mm %",
        "ball Δr",
        "RS %",
        "gyro",
        "mount°"
    );
    for r in reports {
        let s = |x: Option<Stats>, g: fn(&Stats) -> f64| x.as_ref().map(g);
        let ball = r.spheres.first();
        println!(
            "{:<42} {:>9} {:>7} {:>7} {:>6} {:>6} {:>7} {:>7} {:>7} {:>7} {:>7} {:>7} {:>6} {:>8} {:>7}",
            r.name.chars().take(42).collect::<String>(),
            format!("{}/{}", r.frames_posed, r.frames),
            r.points,
            f(s(r.pose_pos_mm, |s| s.rms), 3),
            f(s(r.pose_rot_deg, |s| s.rms), 3),
            f(r.plane_angle_deg, 3),
            f(s(r.surface_mm, |s| s.mean), 3),
            f(s(r.surface_mm, |s| s.p50_abs), 3),
            f(s(r.surface_mm, |s| s.p95_abs), 3),
            f(s(r.surface_mm, |s| s.rms), 3),
            f(r.outliers_1mm.map(|x| 100.0 * x), 2),
            f(ball.and_then(|b| b.radius_err_mm), 3),
            f(Some(100.0 * r.corrected_frames as f64 / r.frames.max(1) as f64), 0),
            r.gyro.map_or("-".into(), |g| {
                let mount = if g.mount_sigma_deg.is_some() { "fit" } else { "pf" };
                g.from_frame.map_or("-".into(), |i| format!("@{i} {mount}"))
            }),
            f(r.gyro.and_then(|g| g.rotation_err_deg), 2),
        );
    }
    println!(
        "\npos/rot: camera pose error RMS · plane°: laser-plane angle error · mean … rms (mm): signed distance of\n\
         every point to the true surface (median/p95 of |e|) · >1mm %: points off by more than 1 mm ·\n\
         ball Δr: sphere-fit radius error of the 25 mm ball (mm) · RS %: frames rolling-shutter corrected ·\n\
         gyro: frame from which the self-calibrated gyro was in use, with the platform's (pf) or a fitted (fit)\n\
         mount at the end · mount°: error of that mount"
    );
}

fn print_reports(reports: &[Report], json: bool) {
    if json {
        println!("{}", serde_json::to_string_pretty(reports).unwrap());
    } else {
        table(reports);
    }
}

fn save_ply(points: &[[f64; 3]], path: &str) -> std::io::Result<()> {
    let mut c = PointCloud::new(points.len());
    for p in points {
        c.push(*p, [200, 200, 200]);
    }
    let mut w = std::io::BufWriter::new(std::fs::File::create(path)?);
    c.write_ply(&mut w).map(|_| ())
}

fn main() -> ExitCode {
    let mut all: Vec<String> = std::env::args().skip(1).collect();
    if all.is_empty() {
        eprintln!("usage: scan_bench sweep|run|eval|eval-ply|config … (see the source header)");
        return ExitCode::from(2);
    }
    let cmd = all.remove(0);
    let mut a = Args { rest: all };
    let json = a.flag("--json");
    let fast = a.flag("--fast");
    let seconds = a.value("--seconds").and_then(|s| s.parse().ok());
    let result: Result<(), String> = (|| match cmd.as_str() {
        "sweep" => {
            let mut reports = Vec::new();
            for mut cfg in bench::scenarios() {
                adjust(&mut cfg, fast, seconds);
                eprintln!("running: {}", cfg.name);
                reports.push(bench::run(&cfg)?.report);
            }
            print_reports(&reports, json);
            Ok(())
        }
        "run" => {
            let mut cfg = match a.value("--config") {
                Some(p) => {
                    serde_json::from_str(&std::fs::read_to_string(&p).map_err(|e| format!("{p}: {e}"))?).map_err(|e| format!("{p}: {e}"))?
                }
                None if a.flag("--realistic") => BenchCfg::realistic(),
                None => BenchCfg::default(),
            };
            adjust(&mut cfg, fast, seconds);
            let out = bench::run(&cfg)?;
            if let Some(p) = a.value("--save") {
                out.recording.save(Path::new(&p)).map_err(|e| format!("{p}: {e}"))?;
            }
            if let Some(p) = a.value("--ply") {
                save_ply(&out.reconstruction.points, &p).map_err(|e| format!("{p}: {e}"))?;
            }
            print_reports(&[out.report], json);
            Ok(())
        }
        "eval" => {
            let p = a.rest.first().ok_or("eval needs a recording")?.clone();
            let rec = Recording::load(Path::new(&p)).map_err(|e| format!("{p}: {e}"))?;
            let recon = reconstruct::reconstruct(&rec, PoseFrom::Auto, &FusionCfg::default());
            print_reports(&[metrics::evaluate(&rec.source, &rec, &recon)], json);
            Ok(())
        }
        "eval-ply" => {
            let radius: Option<f64> = a.value("--radius").and_then(|s| s.parse().ok());
            let p = a.rest.first().ok_or("eval-ply needs a PLY file")?.clone();
            let pts = metrics::read_ply(Path::new(&p)).map_err(|e| format!("{p}: {e}"))?;
            let fit = metrics::fit_sphere(&pts).ok_or("too few points for a sphere")?;
            println!("points        {}", pts.len());
            println!("kept / outl.  {} / {}", fit.n, fit.rejected);
            println!("radius        {:.3} mm", fit.radius);
            if let Some(r) = radius {
                println!("radius error  {:+.3} mm", fit.radius - r);
            }
            println!("residual RMS  {:.3} mm", fit.rms);
            println!("centre        [{:.2}, {:.2}, {:.2}] mm", fit.center[0], fit.center[1], fit.center[2]);
            Ok(())
        }
        "config" => {
            let cfg = if a.flag("--realistic") { BenchCfg::realistic() } else { BenchCfg::default() };
            println!("{}", serde_json::to_string_pretty(&cfg).unwrap());
            Ok(())
        }
        other => Err(format!("unknown command {other}")),
    })();
    match result {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("error: {e}");
            ExitCode::FAILURE
        }
    }
}
