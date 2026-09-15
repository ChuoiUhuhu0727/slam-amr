# Project Summary — What Was Built, What Was Measured, What Was Learned

A camera-based autonomous mobile robot for indoor search and rescue. This is the
single consolidated view: the results, the data behind them, and the engineering
insights that came out of building it.

Detail lives elsewhere and is not repeated here:
[`BENCHMARKS.md`](BENCHMARKS.md) (every measured number, by layer, plus the
external comparisons) · [`full report.md`](full%20report.md) (the formal written
report) · `README.md` (dated session logs).

---

## 1. What the robot does

It is placed in a corner of a known room and switched on. It drives a patrol
route on its own, watching the room's interior with two inward-angled cameras,
recognises the target it was trained to find, works out how far away that target
is from geometry alone, and reports where it is — as a live map, and as walking
directions from the spot the operator put the robot down.

No LiDAR. No SLAM. No external localisation. Two USD 20 camera modules, wheel
encoders and a gyroscope.

**Scale:** ~5,100 lines of Python across 24 files on the Jetson, ~1,650 lines of
C on the ESP32.

---

## 2. Headline results

| | Result |
|---|---|
| **Target position error, end to end, robot driving** | **1 cm / 16 cm / 4 cm** at ground truths of 0.31 m / 0.90 m / 1.04 m |
| Stereo distance accuracy alone, robot stationary | ~10 % over 0.3–1.0 m |
| Stereo calibration quality | 0.33 px reprojection error |
| Full mission run end to end | Yes — 2026-09-06 |

**The first row is the result the project should be judged on.** It is the only
number that includes every layer at once. It is also, deliberately, reported
separately from the second row — see insight 8.

---

## 3. The four layers

An autonomy stack has four layers. This project built all four and, more
unusually, joined them and measured the join.

### Layer 1 — Perception

**Built:** stereo camera calibration from scratch (checkerboard capture,
`stereoCalibrate`, `stereoRectify`, rectification maps); a YOLO target detector
trained and deployed on the Jetson; distance by disparity triangulation; a second
independent distance estimate by dense stereo (`StereoSGBM` →
`reprojectImageTo3D` → median of points inside the detection box); transformation
from camera-optical frame through `base_link` to world coordinates.

**Measured:**

| Quantity | Value |
|---|---|
| Reprojection error | 0.33 px |
| Calibrated baseline | 85.40 mm (physical: 83 mm) |
| Rectified focal length | 875.51 px |
| Distance accuracy (centroid, stationary) | 0.3→0.34 m · 0.5→0.54–0.59 m · 1.0→1.10 m · 1.3→1.30 m |
| Distance accuracy (dense stereo) | 0.3→0.28 m · 0.5→0.47 m · 1.0→0.89–0.91 m; unusable beyond 0.5 m |
| L/R brightness offset | 3.2 / 255 (~3 %), stable |
| L/R capture skew | 2.38 ms mean (0.48–4.92 ms) |
| Vision loop rate | 0.54 Hz achieved against a 5 Hz target |
| Dense stereo compute | ~1000–1050 ms/frame, 8k–24k valid points |
| Detector mAP / precision / recall | never formally evaluated |

### Layer 2 — State estimation

**Built:** dead-reckoning odometry computed on the MCU with midpoint
integration; gyro-fused heading (MPU6050 over I2C, magnetometer bridged through
the IMU's auxiliary bus); pose published at 20 Hz; a room coordinate frame with
a fixed offset from the odometry origin.

**Measured:** almost nothing. Wheel geometry (67 mm diameter, 100 mm wheelbase,
20 slots/rev → 10.5 mm per pulse) is known. Drift over a loop, straight-line
heading error and post-fix turn accuracy have **never been measured**.

### Layer 3 — Control

**Built:** three nested 20 Hz loops on the ESP32 — per-wheel PI speed control,
turn-rate correction, gyro heading lock — driving a TB6612FNG over 1 kHz PWM; a
go-to-goal waypoint controller on the Jetson with turn-in-place above ~50° of
heading error; proportional ramp-down on turns; a motor kill switch that pulls
the driver's standby pin.

**Measured:** gains are documented (Kp 3.0, Ki 0.2, Max-I 40, Khead 15, Klock
2.0), limits are documented (0.30 m/s, 1.5 rad/s, 10 cm arrival tolerance). Step
response, settling time and steady-state RPM error have **never been measured**.

### Layer 4 — Integration

**Built:** micro-ROS bridge over UART at 115200 between ESP32 and Jetson (`/odom`
in, `/cmd_vel` out, diagnostics and live PID gains alongside); a TF tree
(`odom → base_link → camera_optical`) feeding RViz2; a Flask + canvas operator
dashboard with live map, robot pose, both camera feeds, detection overlays,
runtime PID tuning and the kill switch.

**Measured:** the end-to-end result in §2. Patrol route 163 / 50 / 163 / 50 cm
in a 2.23 m × 1.0 m room.

---

## 4. Engineering insights

The transferable part. Each one came from a real failure on real hardware, and
each is backed by evidence in the repository.

### 4.1 Stereo depth error is geometry, not a bug

Distance is `Z = fx·B/d`, so `ΔZ = Z²/(fx·B)·Δd`. Error grows with the *square*
of distance. On this rig one pixel of disparity error costs:

| 0.30 m | 0.50 m | 1.00 m | 1.30 m | 1.50 m |
|---|---|---|---|---|
| 0.40 % | 0.67 % | 1.34 % | 1.74 % | 2.01 % |

**Two things this buys you.** First, any accuracy claim without a stated distance
is meaningless. Second — more useful — it tells a *real fault* from the *noise
floor*: a 10 % error at 1.0 m is ~7 px of disparity error, far above
quantisation, so it is a genuine systematic problem worth hunting. A 1 % error at
1.3 m is sub-pixel (~0.58 px), which is suspiciously good and deserves scrutiny
rather than celebration.

### 4.2 A model's training distribution is part of the geometry pipeline

The detector worked well in single-camera testing and collapsed once the stereo
pipeline went live. Not a model problem: it had been trained exclusively on raw,
distorted camera frames, and the stereo pipeline was feeding it *rectified*
frames — geometrically altered images outside its training distribution.

**The fix was to move the transform, not retrain.** Detection now runs on the raw
frame, and only the resulting box's corner points are mapped into rectified space
via `cv2.undistortPoints(..., R, P)` — the same per-camera rectification the image
remap already uses, applied to four points instead of two million pixels. The
model stays inside its training distribution; the disparity maths still gets
exact epipolar-aligned coordinates.

### 4.3 Passive dense stereo structurally fails on textureless objects

Dense stereo returned three discrete, repeating distance clusters (~0.4 m,
~1.5 m, ~2.4 m) for a *static* target at 1.3 m, while single-point triangulation
held steady at 1.30 m using the same detection box.

The cause is not tuning. Block matching compares small image patches; a patch of
smooth yellow plastic is indistinguishable from every other patch of smooth
yellow plastic. The only confidently matchable pixels were the textured floor and
wall at the box edges — so the matcher reported the *background's* distance.
Proven directly: the eroded centre region produced **zero** confident matches on
every failing frame while the full box produced hundreds, all at the edges. The
intuitive fix (enlarge the centre region) was tested live and made things
*noisier*, confirming the mechanism is missing texture, not an unlucky crop.

**What makes this an insight rather than a defeat:** NVIDIA's own production
stereo package (Isaac ROS ESS) documents the same limitation for textureless
surfaces, and the industry's answer is *hardware* — the Intel RealSense D435
reaches better than 2 % at 2 m from a narrower 50 mm baseline by projecting an
infrared pattern onto the scene, i.e. adding the texture that passive stereo
needs. The gap is a missing hardware capability, not a coding mistake.

### 4.4 Silent failures hide in the semantics of your middleware

`PointCloud2` messages never rendered in RViz2 while `Marker` messages in the
same frame rendered fine. No error anywhere.

The MCU was publishing `Odometry` with an all-zero header stamp. Copying that
stamp into the `odom → base_link` transform pinned it at "time 0" in the tf2
buffer. Markers were placed directly in the frame and needed no lookup, so they
worked. Point clouds must be transformed *at their own timestamp*, so every
lookup failed — silently.

**The generalisation:** when one consumer of a shared resource works and another
does not, the difference between what they *require* is where the bug is.

### 4.5 A sensor's resolution can defeat any controller gain

The wheel speed controller oscillated permanently. It was not a tuning problem:
RPM was measured by counting pulses on a 20-slot disc over a 50 ms window, which
quantises RPM into 60 RPM steps — and the target sat exactly between two
measurable levels, so the controller could never read "at target," only ±30 RPM.
No value of Kp or Ki can fix that.

**Fixed by changing the measurement, not the controller:** timing the microseconds
*between* pulses instead of counting pulses per window removed the quantisation
floor without lowering the loop rate.

### 4.6 One symptom, causes stacked in series

PWM saturated at 78/255. First cause: the commanded speed was genuinely low
(0.15 m/s only asks for ~43 RPM), so the controller was correctly hitting a small
target. Raising the speed limits then exposed a *second*, real bug — an
anti-windup clamp on the integral term so tight it could not close a steady-state
error under load. The first problem had been masking the second the entire time.

**Fixing the first cause is what reveals the second.** A fix that changes the
symptom without eliminating it is evidence, not failure.

### 4.7 Know which errors cancel

The mission reports the target as directions from the robot's start pose. That is
computed as `duck_world − START`, and `duck_world` is itself built as
`odom + START + stereo`. The `START` offset cancels exactly:

```
duck_from_start = (odom + START + stereo) − START = odom + stereo
```

So a *translational* placement error — putting the robot down 5 cm off — does not
affect the reported directions at all. A *rotational* error does, and is
uncorrectable, because the gyroscope zeroes its heading at whatever angle the
robot was set down at. At 1.7 m range, 5° of placement yaw displaces the reported
target by ~15 cm.

**The practical consequence is a better operating procedure:** placing the robot
*square* matters; placing it in exactly the right spot does not. Measure from the
side wall to the front and rear of the chassis and equalise, rather than judging
by eye.

### 4.8 Component accuracy and system accuracy are different claims

The stereo distance sensor alone is accurate to ~10 % over 0.3–1.0 m. The
complete system locates a target to within 1–4 cm end to end while driving.

Both numbers are true. They measure different things, and the second stacks four
error sources (stereo distance, bearing angle, odometry position and heading, and
the running average over sightings) with no way to attribute error between them
from three placements.

**Reporting them separately is the whole credibility of the work.** Quoting the
best data point from a four-point sweep — as "~1 % accurate" would have been —
is cherry-picking, and the sweep is in the repository for anyone to find.

### 4.9 When the maths gives a plausible-but-wrong number, check the connectors

Stereo calibration produced a baseline 22 % away from the physical measurement,
with excellent per-camera reprojection error. The first hypothesis was a model
problem — these are 160° lenses, perhaps the pinhole distortion model does not
fit. A fisheye model was tried and diverged.

The actual cause was a **loose CSI ribbon cable**, found only after re-running
`dmesg` and seeing an i2c probe failure. Reseating it and recalibrating gave
85.4 mm against 83 mm measured.

This is the project's most repeated pattern: CSI cable, encoder power, encoder
solder joint, GPIO wiring. **Almost every serious problem lived in the physical
and electrical layer, not in the algorithms.**

### 4.10 A successful build is not code running on the chip

`idf.py build` failing does not stop the separate flash step from re-uploading
whatever `.bin` was left in `build/` by the last *successful* build. Several
"live test" results were silently produced by stale firmware. A stale binary
fails in exactly the way "the code change had no effect" does.

### 4.11 Isolate the layer you are measuring

Measuring accuracy while the robot drives mixes stereo error and odometry drift
irreversibly — the resulting number cannot say which layer contributed. To
characterise the distance sensor, the robot must be stationary. Driving is only
for validating the system as a whole.

### 4.12 Code should not contradict its own stated reasoning

The live distance readout uses a rolling **median**, with a comment explaining
that median was chosen specifically to reject badly-localised detections. The
function producing the *final answer* uses a plain **mean** over every sighting
in the run, with no outlier rejection and no window — so one bad sighting biases
the result permanently. This is the leading suspect for the single 16 cm outlier
in the headline result. Found by reading the code to trace what a number was
made of.

### 4.13 The evidence gap is not where it feels like it should be

Perception — the layer that looks hardest — is by far the best measured, with
multiple ruler sweeps, a derived physical floor and camera characterisation data.
State estimation and control, the two layers everything else sits on top of, have
almost no numbers at all.

**The layer that gets measured is the one that felt interesting, not the one
carrying the most weight.** Worth checking for deliberately.

---

## 5. Evaluated and deliberately excluded

Both were brought up on real hardware and then excluded, with reasons recorded.
This is engineering judgement, not incompleteness.

| | Why excluded |
|---|---|
| **Nav2** | Its `RegulatedPurePursuitController` accepted goals and produced valid plans but silently never published a single `/cmd_vel`. Isolated to that plugin specifically by swapping in DWB as a differential test, which worked immediately. Root cause never found — DEBUG logging is compiled out of the apt binaries. A known, bounded room does not need live path planning. |
| **cuVSLAM / visual SLAM** | Unresolved pose-scale problem: positions diverged or settled into implausible bands. Multiple hypotheses tested and disproven (camera_info baseline patch made it worse and was reverted). Hardcoded waypoints plus gyro-fused odometry cover a known room. |
| **Isaac ROS stereo depth** | Requires the full Docker container and a rectified stereo launch that claims the cameras exclusively, so it cannot coexist with this node's own capture. Also ~1–1.5 Hz on this class of Jetson. |

---

## 6. Open items

Nothing here blocks the system; all are known and recorded.

| Item | Cost to close |
|---|---|
| Odometry drift over one loop never measured — **the error is already printed on arrival at the final waypoint** | Record it. Zero code. |
| Turn accuracy never re-measured since the ramp-down fix | ~15 min with the existing manual turn tester |
| Straight-line lateral drift never measured | ~15 min with a tape measure |
| Bearing (lateral angle) sign never empirically verified — flagged in the code since it was written | ~5 min: place the target to one known side, check which side the map marker lands on |
| `duck_estimate()` uses mean, not median (§4.12) | ~3 lines |
| Repeatability — every figure in this project is a single placement | ~30 min: five trials per distance, report mean ± σ |
| Dense-stereo disagreement gate never observed triggering live | one run |
| Detector never formally evaluated (no mAP) | a labelled holdout set |

---

## 7. Where this sits against published work

Full treatment in [`BENCHMARKS.md`](BENCHMARKS.md) §9.

- **Intel RealSense D435** — better than 2 % at 2 m, 50 mm baseline, active IR
  pattern projector. The fair comparison, because it uses the same metric type.
- **FoundationStereo** (NVIDIA, CVPR 2025 Oral, Best Paper Nomination; 1st on
  Middlebury and ETH3D; supports Jetson Orin) — the research ceiling and a real
  upgrade path. Fast-FoundationStereo reports >10× speed-up, relevant to the
  ~1 s/frame dense stereo here.
- **NVIDIA Isaac ROS ESS** — cited for its documented textureless-surface
  limitation, which independently confirms §4.3.

**A trap worth naming:** stereo papers report `bad-2.0` — the percentage of
*pixels* whose disparity is wrong by more than 2 px on a public dataset. That is
not the metric used here (percentage error in metric distance to one object). The
two are not directly comparable; the honest bridge is the table in §4.1.
