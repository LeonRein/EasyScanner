# Gyroscope fusion and rolling-shutter correction

The board pose from the ChArUco corners stays the only source of the camera
position. The phone's gyroscope is fused with it for four narrow jobs, each
measured with the synthetic benchmark ([`accuracy-benchmark.md`](accuracy-benchmark.md)).

| Job | Where | What it changes |
|---|---|---|
| Rolling-shutter correction | `algo/src/rolling.rs` | Corners and stripe points are moved to the frame's middle row before the pose is solved and the points are triangulated |
| Gyro self-calibration | `algo/src/imu.rs` | IMU→camera rotation, rate bias and clock offset are fitted to ordinary board poses; nothing is calibrated by hand |
| Turn rate for the scan gate | `algo/src/fusion.rs`, `engine.rs` | `max_turn_deg_s` is checked against the gyro (instant) instead of a 200 ms pose window |
| IPPE branch choice | `algo/src/ippe.rs`, `board.rs` | Near-frontal views take the branch nearest the last pose *turned on by the gyro*, also right after the board was out of view |

`PoseFusion` (`algo/src/fusion.rs`) owns the gyro samples, the calibration
and the recent poses, and answers these questions per frame. The engine and
the benchmark's reconstruction drive it the same way.

## Data flow

```
GyroStream (Kotlin, TYPE_GYROSCOPE at 200 Hz, own thread)
  └─ batch before each frame ─▶ nativePushGyro ─▶ PoseFusion::push_gyro
CameraController capture result: SENSOR_ROLLING_SHUTTER_SKEW, SENSOR_EXPOSURE_TIME
  └─ nativeProcess(…, timestamp, readout, exposure) ─▶ RowClock
Engine::process
  correction = fusion.correction(clock)          rate (gyro) + velocity (poses)
  expected   = fusion.expected_rotation(clock)   last pose ⊕ gyro
  pose       = solve(raw corners) → solve(corrected corners), IPPE branch by `expected`
  fusion.observe(clock, pose, trusted)           trusted = passes the gate's pose checks
  stripe point (its own row's camera frame) ─▶ middle-row frame ─▶ board frame
```

The gyro is used only when Camera2 reports a `REALTIME` timestamp source: then
frame and `SensorEvent` timestamps share `CLOCK_BOOTTIME` and the remaining
offset is a few milliseconds, which the calibration finds.

## Rolling-shutter model

Row `v` of a frame is exposed at `start + exposure/2 + readout·v/height`. Over
one readout (10–30 ms) the camera is taken to turn at a constant rate `ω`
(the gyro's mean over the readout) and move at a constant velocity `v` (the
slope of the camera centre over the last 100 ms of board poses). A point seen
`dt` after the middle row maps to the middle-row camera frame as
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

## Gyro self-calibration

Between two board frames 100 ms apart, the camera's rotation increment
`log(R₁·R₀ᵀ)` must equal the gyro's integrated increment, rotated into the
camera frame. `GyroCalibrator` collects the last ~8 s of trusted poses and,
every 2 s, fits:

- the **clock offset** by a grid search (±40 ms, then 0.25 ms steps),
- per offset, the **IMU→camera rotation** (Kabsch) and the **bias** (linear
  least squares), alternated.

The rotation needs turns about two axes. While the camera turns about one
only, the mounting the app derives from `SENSOR_ORIENTATION` (back camera:
x = (cos θ, −sin θ, 0), y = (−sin θ, −cos θ, 0), z = (0, 0, −1) in device
axes) is used and only bias and offset are fitted. A fit whose RMS
disagreement exceeds 0.2° is rejected; until one is accepted, the rate for
the correction comes from the board poses. The fitted offset is the one that
makes gyro and board agree: it also absorbs the few milliseconds by which
the poses lag before the gyro is in use (≤ 10 ms in the benchmark).

## What the benchmark says

`scan_bench sweep` (seed 1, 600 frames, 20 ms readout). Distances in mm;
"brisk" sweeps the aim at 0.75 Hz (~190 mm/s, ~35°/s) with corner noise
0.2 px and an analytic stripe with 0.3 px noise; "realistic" is every error
source at once with the rendered stripe:

| Scenario | Pose pos RMS | median | p95 | RMS | > 1 mm | Gyro in use from |
|---|---|---|---|---|---|---|
| realistic, uncorrected | 2.070 | 0.149 | 0.488 | 0.334 | 0.61 % | (frame 61) |
| realistic, corrected (gyro) | 1.263 | 0.098 | 0.381 | 0.288 | 0.43 % | frame 61 |
| brisk, uncorrected | 3.879 | 0.275 | 0.818 | 0.414 | 1.85 % | – |
| brisk, corrected from the poses | 1.844 | 0.213 | 0.731 | 0.353 | 1.12 % | – |
| brisk, corrected with the gyro | 1.773 | 0.211 | 0.717 | 0.348 | 1.04 % | frame 301 |

- **The correction pays off**: the pose error drops by 40–55 %; in the
  realistic scan the median point error falls from 0.15 to 0.10 mm and the
  points off by more than 1 mm from 0.61 to 0.43 %, in the brisk one the
  outliers from 1.85 to 1.04 %.
- **The gyro adds little over the board poses for the correction.** In the
  benchmark the camera's turn is smooth enough that the board poses predict
  it; the velocity, which only the poses provide, limits the correction. The
  gyro's rate matters with fast rotational shake that 30 fps poses cannot
  follow; a real hand will show whether that is the case.
- **What the gyro gives regardless**: the turn rate for the gate from the
  first frame on, and an expected rotation for IPPE that survives a board
  occlusion.
- At brisk speed the gyro calibrates late (frame 301): the poses are too
  sheared for a fit within 0.2° until the correction from the poses has
  settled. A calm first second of a scan calibrates it at once.
- The remaining pose error after correction (~1.3–1.8 mm) is the velocity
  estimate: the hand's 7 Hz tremor is invisible to 30 fps poses. Integrating
  the accelerometer over a frame's readout would close that gap; double
  integration for the position itself would not (it drifts by about a
  millimetre within a few frames).

## Settings

Config key `fusion` (JSON from the app, every field optional):

```json
{"fusion": {"gyro": true, "rolling_shutter": "gyro",
            "calibration": {"min_pairs": 40, "max_offset_ms": 40, "max_residual_deg": 0.2,
                            "min_excitation_deg": 0.5, "interval_s": 2}}}
```

`rolling_shutter`: `off`, `poses` (rate and velocity from the board poses) or
`gyro` (rate from the calibrated gyro, poses until then). The app sends
`gyro: false` when the camera's timestamps are not on the sensor clock, and
`camera.cam_from_imu` with the nominal mounting.

The HUD (scan mode) shows the gyro state and whether the frame was
corrected; header slots 58 (`GYRO_STATE`: 0 off, 1 calibrating, 2
calibrated) and 59 (`RS_CORRECTED`).

## Open points

- Real recordings: the benchmark's hand model is a guess. The real readout
  (Pixel 10 Pro main camera), the tremor spectrum and the gyro noise decide
  how much each part is worth. Recording real sessions (`bench::recording`)
  is the next step.
- The laser-plane calibration does not use the correction: its board views
  are held still.
- The scan gate's speed limits (120 mm/s, 25°/s) are unchanged. The
  correction would allow faster sweeps; raise them once real scans confirm it.
