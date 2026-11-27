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
| Motion | One orbit around the board in 20 s at 380 mm and 60° elevation, the aim swept ±40 mm sideways at 0.25 Hz, 0.3 mm tremor at 7 Hz (≈ 60–100 mm/s) |
| Corners | Projected ChArUco corners, occluded by the objects, Gaussian noise |
| Stripe | Either the exact sheet∩scene curve plus pixel noise (`analytic`), or a rendered YUV 4:2:0 frame (sensor noise, 2x2 supersampling near the stripe) that the app's own `LaserDetector` has to find (`render`) |
| Rolling shutter | Every row (corners, stripe, render) exposed at its own time |
| Calibration errors | Wrong focal length / principal point / k1; laser plane tilted or shifted; or the app's `PlaneCalibrator` replayed on 120 simulated static board views with the same noise and the same wrong intrinsics |

The reconstruction follows `engine.rs` `process()`: pose from the corners
(a pure-Rust planar PnP with LM refinement instead of OpenCV's IPPE: same
optimum, no flip ambiguity), `stripe::to_laser_plane`, `Volume::contains`.

## Error budget (seed 1, 600 frames)

Distances are the signed distance of every point to the true surface, in mm.

| Scenario | Pose pos RMS | median | p95 | RMS | > 1 mm | Ball Δr |
|---|---|---|---|---|---|---|
| ideal | 0.000 | 0.000 | 0.000 | 0.000 | 0 % | 0.000 |
| corner noise 0.2 px | 0.241 | 0.024 | 0.087 | 0.042 | 0 % | +0.004 |
| stripe noise 0.3 px | 0.000 | 0.194 | 0.658 | 0.320 | 0.48 % | −0.003 |
| stripe rendered + detector | 0.000 | 0.081 | 0.338 | 0.256 | 0.45 % | −0.032 |
| rolling shutter 20 ms | 0.851 | 0.046 | 0.207 | 0.099 | 0 % | −0.046 |
| focal length +0.5 % | 1.850 | 1.371 | 1.787 | 1.353 | 75 % | +0.694 |
| focal +0.5 %, plane calibrated | 1.850 | 0.129 | 0.232 | 0.145 | 0 % | +0.050 |
| k1 +0.01 | 0.513 | 0.288 | 0.379 | 0.283 | 0 % | +0.143 |
| laser plane tilted 0.1° | 0.000 | 1.620 | 2.492 | 1.641 | 77 % | −0.811 |
| laser plane shifted 0.5 mm | 0.000 | 1.347 | 1.752 | 1.328 | 74 % | +0.688 |
| corner + stripe noise | 0.241 | 0.198 | 0.660 | 0.323 | 0.49 % | +0.001 |
| … + plane calibrated in sim | 0.241 | 0.199 | 0.658 | 0.324 | 0.52 % | −0.009 |
| realistic (all of the above, rendered) | 1.395 | 0.112 | 0.397 | 0.383 | 0.72 % | −0.034 |

What it says (for this geometry, synthetic data):

- **The stripe dominates, not the board pose.** 0.2 px corner noise moves
  the camera by 0.24 mm but the points by only 0.04 mm RMS; 0.3 px of stripe
  noise costs 0.32 mm. Sub-pixel stripe quality and per-frame averaging pay
  off more than pose filtering.
- **The RMS is driven by a few outliers.** With the real detector the median
  error is 0.08 mm but 0.45 % of the points are off by more than 1 mm (up to
  ~10 mm): stripe points at depth edges (ball/board silhouette). A per-point
  filter (neighbour consistency, stripe width) is the cheapest accuracy win.
- **Rolling shutter is small at this speed** (0.1 mm RMS, −0.05 mm on the
  ball radius) although the pose itself is off by 0.85 mm, because board and
  stripe are distorted together. It grows with speed (try `sweep_hz`).
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
