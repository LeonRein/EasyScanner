//! Parity harness against the desktop prototype's Python `detect_laser`.
//!
//! Reads a raw interleaved RGB image, converts it to YUV 4:2:0 the way a
//! camera ISP does (JFIF matrix, 2x2-averaged chroma), runs the detector on
//! the full frame and prints one `x y` line per stripe point.
//!
//! `cargo run --release -p scan-algo --example laser_parity -- in.rgb 960 720 > rust.txt`

use std::env;
use std::error::Error;
use std::fs;

use scan_algo::bench::sim::YuvImage;
use scan_algo::laser::{LaserDetector, LaserParams, Roi};

fn main() -> Result<(), Box<dyn Error>> {
    let args: Vec<String> = env::args().collect();
    let [_, path, w, h] = args.as_slice() else {
        return Err("usage: laser_parity <in.rgb> <width> <height>".into());
    };
    let (w, h) = (w.parse::<usize>()?, h.parse::<usize>()?);
    let rgb = fs::read(path)?;
    if rgb.len() != w * h * 3 {
        return Err(format!("{path} holds {} bytes, {w}x{h} RGB needs {}", rgb.len(), w * h * 3).into());
    }

    let img = YuvImage::from_rgb(&rgb, w, h);
    let frame = img.frame()?;
    let mut det = LaserDetector::default();
    let mut pts = Vec::new();
    let found = det.detect(&frame, &Roi::full(w, h), &LaserParams::default(), &mut pts);
    eprintln!("found={found} points={} peak={}", pts.len(), det.stats.peak);
    for p in pts {
        println!("{} {}", p[0], p[1]);
    }
    Ok(())
}
