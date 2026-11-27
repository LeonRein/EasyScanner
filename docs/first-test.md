# First test on the real scanner

A checklist for the first scan with the printed scanner: what to prepare,
what the app calibrates by itself, which steps need you, and what to bring
back so the pipeline can be tuned on real data.

| What | Who | When |
|---|---|---|
| Gyro mount (IMU→camera rotation) | app, automatic: from Android's sensor axes and `SENSOR_ORIENTATION`, refined against the board while you move | always |
| Gyro ↔ camera clock offset | app, automatic: fitted against the board poses | always |
| Rolling-shutter correction | app, automatic: from the gyro and the board poses | always |
| Board | you: pick the printed board, point the phone at it | once per board |
| Focus | you: lock it at the scanning distance | once, and after every focus change |
| Camera intrinsics | you move the phone, the app solves | once per focus |
| Laser plane (where the laser sheet lies) | you sweep the stripe over the empty board, the app fits | after every camera calibration and whenever the laser moves in its holder |

The laser is modelled as a flat sheet of light. Its shape is not
calibrated; the plane fit's RMS shows how flat it is.

## Before you start

- **Install the app**: the APK is an artifact of the newest green *Android
  build* run on `main` (Actions → the run → Artifacts →
  `easyscanner-debug-<commit>`). The artifact is a zip: unzip it on the
  computer and copy the APK to the phone (or `adb install -r <apk>`), then
  open it in Files and allow installs from that app; Play Protect may ask
  once. Builds from before the shared signing key need one uninstall first;
  from then on every build is signed with the same key, so a newer APK
  installs over an older one and keeps the calibrations.
- **Print a board**: `charuco/boards/charuco_A3_6x8_45mm.pdf` (or A4) at
  100 % ("Actual size"), check the 100 mm scale bar with a ruler, and glue
  the sheet flat onto something stiff. If the scale bar is off, enter the
  measured square and marker sizes in Settings → ChArUco board.
- **A reference object**: a ball of known diameter (a 40 mm table tennis
  ball, a steel ball) next to something with flat faces (a small box). The
  ball's radius is what the accuracy is scored on.

## Without the laser

1. **Board**: *Change board* and pick the printed board's preset (A3 or
   A4), then point the phone at it; *Board OK* becomes available once enough
   corners are found.
2. **Focus**: hold the phone at the scanning distance (25–40 cm) and lock
   the focus.
3. **Camera**: Start, then move the phone slowly around the board at varied
   tilts and distances until it is saved; the wizard then moves on by itself.
   The RMS stays in the HUD's camera line (`calibrated <size> rms <x>px`);
   below 0.5 px is good.

Worth reporting: the camera RMS, the frame rate and resolution the HUD
shows, and anything that looks wrong (screenshots help).

## With the laser

4. **Flash the laser switch** (`firmware/README.md`) and check it on the
   computer: the laser lights up a second after plugging in. Without the
   firmware the MOSFET module keeps the laser off.
5. **Mount the laser**: start with the 90° module (focus 250 mm, the
   thinnest line at scanning distance). Tilt it so the stripe crosses the
   middle of the image at about 30 cm, then tighten the knob. Every later
   change of the laser in its holder needs a new laser plane (step 7).
6. **Connect** the scanner's USB-C to the phone. If Android asks which app
   should open the USB device, dismiss it.
7. **Laser plane**: Start, then sweep the stripe across the *empty* board at
   varied tilts and distances until the plane is saved
   (`SAVED plane | rms <x> mm` in the laser step's messages). Note the RMS;
   the phone scan in the README had 0.35 mm, lower is better.
8. **Scan**: put the objects on the board, Start, and sweep the stripe over
   them slowly (the scan gate keeps frames below 120 mm/s and 25 °/s) while
   walking around them; keep the board in view. 30–60 s is plenty. Finish.
   Dim room light makes the red stripe easier to find. While scanning, the
   HUD shows the frames the gate kept and rejected (counted only while the
   stripe hits the scan volume) and the gyro state: it goes from
   *calibrating (turn the phone)* to *calibrated (…)* once the phone has
   turned a little; *off* means the camera does not share the gyro's clock
   and the gyro is not used. On a Pixel, stay on the main camera (camera 0).

## What to bring back

Finish copies the session to `Download/EasyScanner/<yyyyMMdd-HHmmss>/` on the phone
(visible from a computer over USB):

| File | What it is for |
|---|---|
| `scan.ply` | the cloud, Z up over the board: `scan_bench eval-ply scan.ply --radius <ball radius mm>` |
| `recording.jsonl` | every scanning frame's corners, pose, stripe pixels and timing, plus the gyro: replayed offline with `scan_bench eval` (see [accuracy-benchmark.md](accuracy-benchmark.md)) |
| `session.json`, `photos.jsonl`, `photos/` | the camera model and the posed photos |

`recording.jsonl` grows by about 20–40 MB per minute of scanning; it can be
switched off in Settings → Diagnostics → *Save raw scan data*. A
resolution switch continues the same recording. Also note the calibration numbers
(camera RMS, laser-plane RMS) and the ball's true diameter.
