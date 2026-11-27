# Scan accuracy benchmark

Real scans have not been measured yet, and the hardware for a measurement is
not always at hand. `scan-algo` therefore has a synthetic benchmark
(`android/core/algo/src/bench/`, binary `scan_bench`) that simulates a
hand-held scan of known objects and scores the reconstructed cloud against
the ground truth. Every later change to the pipeline (quality gates, pose
refinement, rolling-shutter correction, IMU fusion, calibration) can be
measured with it.

```sh
cd android/core
cargo run --release -p scan-algo --bin scan_bench -- sweep          # error budget, ~2 min
cargo run --release -p scan-algo --bin scan_bench -- sweep --fast   # analytic stripe only, ~20 s
cargo run --release -p scan-algo --bin scan_bench -- config --realistic > my.json   # edit, then:
cargo run --release -p scan-algo --bin scan_bench -- run --config my.json --save rec.json --ply cloud.ply
cargo run --release -p scan-algo --bin scan_bench -- eval rec.json
cargo run --release -p scan-algo --bin scan_bench -- eval-ply scan.ply --radius 25
cargo test -p scan-algo bench                                        # the benchmark's own tests
```

## What is simulated

| Piece | Model |
|---|---|
| Scene | The app's default board (8x8, 36 mm, black squares and marker bits for the stripe's contrast), a Ø50 mm ball and a 40x30x20 mm block on it |
| Camera | 1280x960, f = 920 px, mild radial distortion |
| Laser | Emitter 100 mm beside the camera, sheet crosses the optical axis at 350 mm, σ = 0.4 mm thick, 70° fan, shadows |
| Motion | One orbit around the board in 20 s at 380 mm and 60° elevation, the aim swept ±40 mm sideways at 0.25 Hz, the aim wobbling ±1° at 0.5 Hz, 0.3 mm / 0.1° tremor at 7 Hz (≈ 60–100 mm/s) |
| Corners | Projected ChArUco corners, occluded by the objects, Gaussian noise |
| Stripe | Either the exact sheet∩scene curve plus pixel noise (`analytic`), or a rendered YUV 4:2:0 frame (sensor noise, 2x2 supersampling near the stripe) that the app's own `LaserDetector` has to find (`render`) |
| Rolling shutter | Every row (corners, stripe, render) exposed at its own time |
| Gyroscope | 200 Hz, 0.07 °/s noise, 0.1 °/s residual bias, 4 ms clock offset, mounted like a back camera with its sensor landscape; the mounting the phone reports is 0.5° off |
| Calibration errors | Wrong focal length / principal point / k1; laser plane tilted or shifted; or the app's `PlaneCalibrator` replayed on 120 simulated static board views with the same noise and the same wrong intrinsics |

The reconstruction follows `engine.rs` `process()`: the gyro fusion and
rolling-shutter correction (`fusion::PoseFusion`, see
[`imu-fusion.md`](imu-fusion.md)), pose from the corners (a pure-Rust planar
PnP with LM refinement instead of OpenCV's IPPE: same optimum, no flip
ambiguity), `stripe::to_laser_plane`, `Volume::contains`. Poses are compared
with the truth at each frame's middle row, the instant a corrected pose
describes.

## Error budget (seed 1, 600 frames, `sweep`)

Distances are the signed distance of every point to the true surface, in mm.

| Scenario | Pose pos RMS | median | p95 | RMS | > 1 mm | Ball Δr |
|---|---|---|---|---|---|---|
| ideal | 0.000 | 0.000 | 0.000 | 0.000 | 0 % | 0.000 |
| corner noise 0.2 px | 0.240 | 0.026 | 0.084 | 0.042 | 0 % | −0.000 |
| stripe noise 0.3 px | 0.000 | 0.194 | 0.658 | 0.324 | 0.65 % | +0.003 |
| stripe rendered + detector | 0.000 | 0.079 | 0.308 | 0.262 | 0.34 % | −0.029 |
| rolling shutter 20 ms, uncorrected | 1.796 | 0.099 | 0.299 | 0.158 | 0 % | +0.006 |
| focal length +0.5 % | 1.849 | 1.392 | 1.793 | 1.362 | 74 % | +0.713 |
| focal +0.5 %, plane calibrated | 1.849 | 0.128 | 0.234 | 0.147 | 0 % | +0.051 |
| k1 +0.01 | 0.520 | 0.292 | 0.387 | 0.287 | 0 % | +0.146 |
| laser plane tilted 0.1° | 0.000 | 1.647 | 2.480 | 1.654 | 77 % | −0.834 |
| laser plane shifted 0.5 mm | 0.000 | 1.368 | 1.756 | 1.338 | 74 % | +0.707 |
| corner + stripe noise | 0.240 | 0.195 | 0.668 | 0.326 | 0.65 % | +0.002 |
| … + plane calibrated in sim | 0.240 | 0.196 | 0.666 | 0.326 | 0.66 % | −0.007 |
| realistic, rolling shutter uncorrected | 2.070 | 0.149 | 0.488 | 0.334 | 0.61 % | +0.024 |
| realistic (all of the above, rendered, gyro, corrected) | 1.263 | 0.098 | 0.381 | 0.288 | 0.43 % | +0.020 |
| brisk (~190 mm/s), RS 20 ms, uncorrected | 3.879 | 0.275 | 0.818 | 0.414 | 1.85 % | −0.037 |
| brisk, RS corrected from the poses | 1.844 | 0.213 | 0.731 | 0.353 | 1.12 % | +0.068 |
| brisk, RS corrected with the gyro | 1.773 | 0.211 | 0.717 | 0.348 | 1.04 % | +0.069 |

What it says (for this geometry, synthetic data):

- **The stripe dominates, not the board pose.** 0.2 px corner noise moves
  the camera by 0.24 mm but the points by only 0.04 mm RMS; 0.3 px of stripe
  noise costs 0.32 mm. Sub-pixel stripe quality and per-frame averaging pay
  off more than pose filtering.
- **The RMS is driven by a few outliers.** With the real detector the median
  error is 0.08 mm but 0.45 % of the points are off by more than 1 mm (up to
  ~10 mm): stripe points at depth edges (ball/board silhouette). A per-point
  filter (neighbour consistency, stripe width) is the cheapest accuracy win.
- **Rolling shutter is small at this speed** (0.16 mm RMS) although the pose
  itself is off by 1.8 mm, because board and stripe are distorted together.
  It grows with speed, and the correction ([`imu-fusion.md`](imu-fusion.md))
  halves the pose error and cuts the realistic median by a third.
- **A wrong camera calibration is mostly compensated by the laser-plane
  calibration** as long as the plane is calibrated with the same
  intrinsics: +0.5 % focal error costs 1.35 mm with the true plane, 0.15 mm
  with the re-calibrated one. Re-calibrate the plane after every camera
  calibration (the app already invalidates it).
- **The laser plane must be good to ~0.01°/0.05 mm**: 0.1° tilt or 0.5 mm
  shift already give 1.3–1.6 mm. The app's 6 mm RMS gate for the plane fit
  is far too loose to guarantee that; the simulated calibration reaches
  0.003°.

## Real recordings

`bench::recording::Recording` (JSON) holds per frame the corner pixels and/or
the device pose and the stripe pixels, plus the intrinsics and laser plane
the pipeline uses. The simulator writes it with a `truth` attached; a real
session saved in the same shape (corners and stripe pixels from
`engine.rs`, follow-up work) runs through the same `reconstruct` and
`evaluate`, then without ground-truth metrics but with the sphere fit. Until
then, a real scan of a reference ball exported as PLY can be scored with
`eval-ply --radius <mm>`.
