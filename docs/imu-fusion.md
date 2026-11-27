# Gyroscope fusion and rolling-shutter correction

The board pose from the ChArUco corners stays the only source of the camera
position. The phone's gyroscope is fused with it for four narrow jobs, each
measured with the synthetic benchmark ([`accuracy-benchmark.md`](accuracy-benchmark.md)).

| Job | Where | What it changes |
|---|---|---|
| Rolling-shutter correction | `algo/src/rolling.rs` | Corners and stripe points are moved to the frame's middle row before the pose is solved and the points are triangulated; the laser-plane calibration intersects each stripe pixel with the board as its own row saw it |
| Gyro calibration | `algo/src/imu.rs` | The IMU→camera rotation comes from the platform's axis definitions and is refined from board poses; the clock offset is fitted from board poses |
| Turn rate for the scan gate | `algo/src/fusion.rs`, `engine.rs` | `max_turn_deg_s` is checked against the gyro (instant, from the first sample) instead of a 200 ms pose window |
| IPPE branch choice | `algo/src/ippe.rs`, `board.rs` | Near-frontal views take the branch nearest the last pose *turned on by the gyro*, also right after the board was out of view |

`PoseFusion` (`algo/src/fusion.rs`) owns the gyro samples, the calibration
and the recent trusted poses, and answers these questions per frame. The
engine and the benchmark's reconstruction drive it the same way.

## Data flow

```
GyroStream (Kotlin, TYPE_GYROSCOPE at 200 Hz, own thread)
  └─ batch before each frame ─▶ nativePushGyro ─▶ PoseFusion::push_gyro
CameraController capture results, matched to the frame by SENSOR_TIMESTAMP:
  SENSOR_ROLLING_SHUTTER_SKEW, SENSOR_EXPOSURE_TIME
  └─ nativeProcess(…, timestamp, readout, exposure) ─▶ RowClock
Engine::process
  correction = fusion.correction(clock)          rate (gyro) + velocity (poses)
  expected   = fusion.expected_rotation(clock)   last trusted pose ⊕ gyro
  pose       = solve(raw corners) → solve(corrected corners), IPPE branch by `expected`
  fusion.observe(clock, trusted pose)            trusted = passes the gate's pose checks
  stripe point (its own row's camera frame) ─▶ middle-row frame ─▶ board frame
```

The gyro is used only when Camera2 reports a `REALTIME` timestamp source: then
frame and `SensorEvent` timestamps share `CLOCK_BOOTTIME` and the remaining
offset is a few milliseconds, which the calibration finds.

Only *trusted* poses (good fit, enough corners, no IPPE ambiguity) enter the
fusion: an untrusted one never becomes a velocity, an expected rotation or a
calibration sample. A velocity above 1.5 m/s is taken for a pose error (a
mirrored branch that slipped through moves the camera by decimetres) and the
frame is left uncorrected.

## Rolling-shutter model

Row `v` of a frame is exposed at `start + exposure/2 + readout·v/height`. Over
one readout (10–30 ms) the camera is taken to turn at a constant rate `ω`
(the gyro's mean over the readout) and move at a constant velocity `v` (the
slope of the camera centre over the last 100 ms of trusted poses). A point
seen `dt` after the middle row maps to the middle-row camera frame as
`x_ref = exp(ω·dt)·x + v·dt`.

- **Corners** need their depth for the translation term: the pose is solved
  once from the raw corners, which gives every corner's depth, then again from
  the corrected ones.
- **Stripe points** need no depth: the laser is rigid to the camera, so the
  ray of row `v` ∩ laser plane is exact in that row's camera frame, and only
  the step to the board frame needs the motion.
- **No rotation-only correction.** A hand-held camera turns to keep the object
  in view while it moves, so the two shears largely cancel. Undoing only the
  rotation made the pose worse than leaving both (12 mm vs. 5 mm in a test),
  so without a velocity there is no correction.
- The velocity is a straight-line fit, not an extrapolation with
  acceleration: the poses it is fitted to were themselves corrected with an
  earlier velocity, and the larger gain of an extrapolation made that loop
  unstable in the benchmark.

## Gyro calibration

### Mount: defined by the platform, refined when the motion allows

The IMU→camera rotation is not guessed. Android fixes the sensor axes to the
device (x right, y up, z out of the screen in the natural orientation), a back
camera looks along −z, and Camera2's `SENSOR_ORIENTATION` θ says how far its
raw image is turned against the device. Together they define the mount:
camera x = (cos θ, −sin θ, 0), y = (−sin θ, −cos θ, 0), z = (0, 0, −1) in
device axes. What is left is the assembly tolerance of the gyro and the
camera module, a fraction of a degree.

The calibration starts from this platform mount and fits its own whenever the
camera turned about two axes. A fitted mount replaces the platform's only if
its standard error is below 0.25° (half the assumed tolerance) and it lies
within 5° of the platform's; a later fit replaces it only if it is more
certain still, so a phase that turns about one axis never discards it. The
standard error comes from the information the increments carry about each
axis of the mount (`Σ(|g|²·I − g·gᵀ)`, weakest axis) and the residual across
them, taken as independent; in the benchmark the true error was 1–2.5 times
this standard error. If the platform mount were wrong by more than 5°,
neither it nor a fit would be accepted and the gyro would stay unused:
the mount is either defined or calibrated, never assumed.

Getting the mount exactly right matters little for the jobs above: 0.5° off
changes the camera rate by under 1 %.

### Clock offset

Between two trusted board frames 100 ms apart, the camera's rotation
increment `log(R₁·R₀ᵀ)` must equal the gyro's increment over the same
interval, rotated into the camera frame. `GyroCalibrator` collects the last
~8 s of trusted poses and, every 2 s, searches the offset (±40 ms in 2 ms
steps, then 0.25 ms around the best; once calibrated, ±4 ms around the last
offset). A fit is accepted when it rests on at least 40 pairs, when the poses
turned by at least 0.5° RMS, and when its residual stays below 15 % of that
turn — relative, because the poses' own rolling-shutter error grows with the
turn rate until the gyro corrects them. The fitted offset is the one that
makes gyro and board agree: it also absorbs the few milliseconds by which the
poses lag before the gyro is in use.

### No bias in the model

`TYPE_GYROSCOPE` is bias compensated by Android. What remains (≤ 0.1°/s)
moves a 100 ms pose increment by ≤ 0.01°, far below the poses' own noise, and
the rotation carried across a 3 s board loss by ≤ 0.3°, far below the
separation of IPPE's branches. An earlier version fitted a bias; with the
mount fixed to the platform's, the bias absorbed the mount error and came out
worse than none. The benchmark keeps a residual bias as a disturbance.

### Cost

The gyro samples are integrated as they arrive (`GyroBuffer` stores the
attitude at every sample), so a rotation over any interval is two lookups and
a product. A calibration attempt takes ~4–6 ms on a desktop core (it was
84–197 ms before), once every 2 s.

## What the benchmark says

`scan_bench sweep` (seed 1, 600 frames, 20 ms readout). Distances in mm;
"brisk" sweeps the aim at 0.75 Hz (~190 mm/s, ~35°/s) with corner noise
0.2 px and an analytic stripe with 0.3 px noise; "realistic" is every error
source at once with the rendered stripe. Each corrected row runs on exactly
the frames of the uncorrected row above it, with the same gates.

| Scenario | Pose pos RMS | median | p95 | RMS | > 1 mm | Gyro from | Mount error |
|---|---|---|---|---|---|---|---|
| realistic, uncorrected | 2.070 | 0.149 | 0.488 | 0.334 | 0.61 % | (frame 61) | |
| realistic, corrected (gyro) | 1.261 | 0.099 | 0.381 | 0.288 | 0.43 % | frame 61 | 0.50° (platform) |
| brisk, uncorrected | 3.879 | 0.275 | 0.818 | 0.414 | 1.85 % | | |
| brisk, corrected from the poses | 1.844 | 0.213 | 0.731 | 0.353 | 1.12 % | – | |
| brisk, corrected with the gyro | 1.662 | 0.207 | 0.700 | 0.340 | 0.88 % | frame 61 | 0.50° (platform) |
| brisk two-axis (wobble 5°), uncorrected | 4.939 | 0.348 | 1.130 | 6.078 | 7.59 % | | |
| brisk two-axis (wobble 5°), gyro | 1.621 | 0.214 | 0.708 | 0.350 | 1.02 % | frame 61 | 0.43° (fitted) |

- **The correction pays off**: the pose error drops by 39–67 %; in the
  realistic scan the median point error falls from 0.15 to 0.10 mm and the
  points off by more than 1 mm from 0.61 to 0.43 %, in the brisk one the
  outliers from 1.85 to 0.88 %, and with a two-axis hand from 7.6 to 1.0 %.
- **The gyro adds a little over the board poses** (brisk: 1.66 vs. 1.84 mm
  pose error, 0.88 vs. 1.12 % outliers). The velocity, which only the poses
  provide, limits the correction: the hand's 7 Hz tremor is invisible to
  30 fps poses. Integrating the accelerometer over a frame's readout would
  close that gap; double integration for the position itself would not (it
  drifts by about a millimetre within a few frames).
- **What the gyro gives regardless**: the turn rate for the gate from the
  first sample on, and an expected rotation for IPPE that survives a board
  occlusion.
- The calibration is in use from frame 61 (the first attempt with 2 s of
  poses) in every scenario but one.
- **The fitted mount is only slightly better than the platform's** (0.37–0.43°
  vs. 0.5°): the poses' remaining rolling-shutter error, which follows the
  motion, limits it. That is why the platform's mount is the default and a
  fit has to prove itself.

**These gains are an upper bound.** The simulated camera exposes its rows
exactly as the correction's model assumes, and the simulated gyro is ideal
unless a row breaks one assumption on purpose. The disturbance rows break
them one at a time:

| Scenario (brisk, gyro) | Pose pos RMS | median | p95 | > 1 mm | Gyro from |
|---|---|---|---|---|---|
| as above, no disturbance | 1.662 | 0.207 | 0.700 | 0.88 % | frame 61 |
| skew reported +10 % | 1.780 | 0.209 | 0.716 | 1.07 % | frame 121 |
| skew reported −10 % | 1.637 | 0.205 | 0.699 | 0.82 % | frame 61 |
| timestamp at the end of the readout | 1.662 | 0.207 | 0.700 | 0.88 % | frame 61 |
| exposure 2–16 ms per frame | 1.612 | 0.205 | 0.699 | 0.87 % | frame 61 |
| imperfect gyro (1 % scale, 0.02°/s/√s bias walk, 0.5 ms jitter) | 1.663 | 0.207 | 0.702 | 0.90 % | frame 61 |
| two-axis hand + every disturbance above | 1.707 | 0.218 | 0.717 | 1.09 % | frame 61 |

Timing and gyro errors leave an uncorrected reconstruction as it is, so
"brisk, uncorrected" (3.879 mm, 1.85 %) is their baseline. None of them
costs more than a fraction of the gain: a shifted timestamp is absorbed by
the fitted clock offset, a wrong skew scales the correction by the same
error, and the gyro's own errors are small against the poses'. Real phones
can still differ in ways the simulation does not model (motion blur, a
readout that is not linear in the row, the tremor spectrum); real recordings
decide.

## Settings

Config key `fusion` (JSON from the app, every field optional):

```json
{"fusion": {"gyro": true, "rolling_shutter": true,
            "calibration": {"min_pairs": 40, "max_offset_ms": 40, "max_residual_rel": 0.15,
                            "min_turn_deg": 0.5, "max_mount_sigma_deg": 0.25,
                            "max_mount_deviation_deg": 5, "interval_s": 2}}}
```

`rolling_shutter`: correct the rolling shutter, with the rate from the
calibrated gyro and from the board poses until then (or always, with
`gyro: false`). The app sends `gyro: false` when the camera's timestamps are
not on the sensor clock, and `camera.cam_from_imu` with the platform mount.

The HUD (scan mode) shows the gyro state and whether the frame was
corrected; header slots 58 (`GYRO_STATE`: 0 off, 1 calibrating, 2 calibrated
with the platform mount, 3 with a fitted mount) and 59 (`RS_CORRECTED`).

## Open points

- Real recordings: the benchmark's hand model is a guess. The real readout
  (Pixel 10 Pro main camera), the tremor spectrum and the gyro noise decide
  how much each part is worth. Recording real sessions (`bench::recording`,
  format version 2) is the next step.
- The scan gate's speed limits (120 mm/s, 25°/s) are unchanged. The
  correction would allow faster sweeps; raise them once real scans confirm it.
- The capture result of a frame often arrives after its image; its row
  timing then comes from the frame before, which differs only when the
  exposure changed.
