# Benchmarks — Every Measured Number

All measurements for this robot, by autonomy layer. Every value traces to a dated
live run on the real rig — nothing simulated, nothing estimated.

**Legend: M** = measured against ground truth · **C** = configured value, not a
result · **—** = never measured.

The *explanations* for why these numbers look the way they do live in
[`PROJECT_SUMMARY.md`](PROJECT_SUMMARY.md) §4. This file is the data.

---

## 1. What is being measured

> **Metric:** absolute error in the estimated straight-line distance to *one
> specific detected object*, as a percentage of tape-measured ground truth.
>
> **Protocol:** target static at a measured distance, read over many consecutive
> frames. Robot stationary unless stated otherwise.
>
> **Not measured:** dense per-pixel depth quality, disparity error on any public
> dataset, or lateral (x/y) position error.

This is *object-level metric depth* — deliberately the number the mission needs,
not the one stereo papers optimise. See §9 before comparing to any publication.

---

## 2. Hardware under test

| Item | Value | |
|---|---|---|
| Cameras | 2× IMX219 CSI, 160° FOV, rolling shutter | C |
| Resolution | 1280 × 720 (sensor mode 4) | C |
| Physical baseline | 83 mm (tape) | M |
| Calibrated baseline | **85.40 mm** (2.9 % off physical) | M |
| Rectified focal length | **fx = 875.51 px** | M |
| Reprojection error | **0.33 px** | M |
| fx × baseline | 74.77 | derived |
| Compute | Jetson Orin Nano Super, JetPack 6.x | C |
| Detector | YOLO (`best.pt`), run on **raw** un-rectified frames | C |
| Disparity floor / distance ceiling | 2 px / 5 m | C |

Passive stereo on ambient light — no IR projector, no structured light. This
matters; see §9.

---

## 3. The physical floor

`Z = fx·B/d` → `ΔZ = Z²/(fx·B)·Δd`. Cost of **one pixel** of disparity error:

| Distance | 0.30 m | 0.50 m | 0.75 m | 1.00 m | 1.30 m | 1.50 m | 2.00 m |
|---|---|---|---|---|---|---|---|
| Disparity | 249 px | 149 px | 100 px | 75 px | 57 px | 50 px | 37 px |
| Error | 1.2 mm | 3.3 mm | 7.5 mm | 13.4 mm | 22.6 mm | 30.1 mm | 53.5 mm |
| As % | **0.40 %** | **0.67 %** | **1.00 %** | **1.34 %** | **1.74 %** | **2.01 %** | **2.68 %** |

Use this to tell a real fault from the noise floor: a 10 % error at 1.0 m is
~7 px of disparity error (a genuine systematic problem); a 1 % error at 1.30 m is
sub-pixel (~0.58 px) and deserves scrutiny rather than celebration.

---

## 4. Layer 1 — Perception

### 4.1 Distance accuracy — centroid disparity

Single disparity from the horizontal offset between the target's box centroid in
the left vs right rectified frame. Median-smoothed over 8 readings for display.

| Ground truth | 2026-08-27 | 2026-09-04 (post photometric fix) | |
|---|---|---|---|
| 0.30 m | — | 0.34 m (**+13.3 %**, −29.3 px) | M |
| 0.50 m | 0.46 m (−8.0 %, ~13 px) | 0.54–0.59 m (**+9 to +18 %**, ~−12 px) | M |
| 1.00 m | 1.06 m (+6.0 %, ~4 px) | 1.10 m (**+10.0 %**, −6.8 px) | M |
| 1.30 m | — | 1.30–1.31 m (**~+1 %**, −0.4 px) | M |
| 1.50 m | 1.35 m (−10.0 %, ~6 px) | — | M |

**Headline: ~10 % over 0.3–1.0 m, reading consistently long.** The 1.30 m point is
one reading in a four-point sweep from the same session — quoting it alone as
"~1 % accurate" would be cherry-picking. A near-constant percentage error suggests
a scale problem, but the 1.30 m point breaks that pattern, so no single scale
factor explains the set. Unresolved.

At 1.5 m raw single-frame readings fluctuated **1.2–1.7 m**, which motivated the
rolling-median window.

### 4.2 Distance accuracy — dense stereo (point cloud)

`StereoSGBM` on the rectified pair → `reprojectImageTo3D` → median of valid points
inside the box, core region eroded 25 % per side. Rate-limited to ~1 Hz.

| Ground truth | 2026-08-29 (post core-region fix) | Before that fix | |
|---|---|---|---|
| 0.30 m | 0.28 m (**6.7 %**) | 0.43 m (43 % err) | M |
| 0.50 m | 0.47 m (**6.0 %**) | 0.50 m | M |
| 1.00 m | 0.89–0.91 m (**~10 %**) | 0.92–1.29 m (37 cm swing) | M |
| **Above 0.5 m** | **Unusable** — at 1.30 m returns three repeating clusters: 0.37–0.44 / 1.53–1.56 / 2.43–2.46 m | | M |

Cause: the target's smooth surface has no texture for block matching, so the only
confidently matchable pixels are textured background at the box edges. Proven —
the eroded core produced **zero** confident matches on every failing frame while
the full box produced 100+, all at the edges. Enlarging the core was tested live
and made readings *noisier*. Now gated against the centroid method at a **30 %**
disagreement threshold. Residual 5–11 % near-field bias remains open.

Box height shrinks with range (`≈ fx·h/Z`): 379 px @ 0.3 m → 227 @ 0.5 → 114 @ 1.0
→ 87 @ 1.3; after 25 % erosion the surviving core is 190 / 114 / 57 / 44 px.

### 4.3 Camera characterisation

| Quantity | Value | |
|---|---|---|
| L/R brightness | left 107.7, right 111.1 → **3.2/255 (~3 %)** gap, stable (std 0.28/0.29, 38 samples) | M |
| L/R capture skew | 2.38 ms mean (0.48–4.92 ms, 20 samples) | M |
| Worst-case theoretical skew | ~16.7 ms (one frame at 60 fps) | derived |
| Dense stereo compute | ~1000–1050 ms/frame, 8k–24k valid points | M |
| Vision loop rate | **0.54 Hz** achieved against a 5 Hz target — YOLO + SGBM bound | M |
| Detector mAP / precision / recall | never formally evaluated | — |

---

## 5. Layer 2 — State estimation ⚠️

**This layer has essentially no benchmarks**, and it sits underneath every
position the robot reports.

| Quantity | Value | |
|---|---|---|
| Odometry publish rate | 20 Hz | C |
| Wheel diameter / wheelbase | 67 mm / 100 mm | M |
| Encoder resolution | 20 slots/rev → 10.5 mm per pulse | C |
| IMU | MPU6050, gyro ±250 °/s (131 LSB/(°/s)) | C |
| Heading source | Gyro-fused, not encoder-differential | C |
| **Drift over one patrol loop** | **— never measured** | — |
| **Straight-line heading error over 1.63 m** | **— never measured** | — |
| **Turn accuracy since the ramp-down fix** | **— never measured** (pre-fix: commanded 90° landed ~144°) | — |

---

## 6. Layer 3 — Control ⚠️

Configuration documented; **closed-loop performance not measured.**

| Quantity | Value | |
|---|---|---|
| Control loop / PWM carrier | 20 Hz (50 ms) / 1 kHz LEDC | C |
| Max linear / angular speed | 0.30 m/s / 1.5 rad/s | C |
| Arrival tolerance / turn-in-place threshold / turn-done tolerance | 100 mm / 0.9 rad (~50°) / 2° | C |
| PID gains | Kp 3.0, Ki 0.2, Max-I 40 | C |
| Heading gains | Khead 15, max trim 20 RPM; Klock 2.0, max lock 0.6 | C |
| RPM measurement | Pulse-**interval** timing — replaced pulse counting, which quantised RPM into 60 RPM steps and caused a limit cycle | M |
| PWM headroom bugs | Ceiling was 78/255, then plateaued 100–130; both root-caused and removed | M |
| **Step response / settling / overshoot** | **— never measured** | — |
| **Steady-state RPM error vs target** | **— never measured** | — |

---

## 7. Layer 4 — Integration (end to end) ✅

The only measurement that includes every layer at once. **Robot driving**,
distance from its start pose to the target, against tape:

| Ground truth | Reported | Error | |
|---|---|---|---|
| 0.31 m | 0.30 m | **1 cm** (−3.2 %) | M |
| 0.90 m | 0.74 m | 16 cm (−17.8 %) | M |
| 1.04 m | 1.00 m | **4 cm** (−3.8 %) | M |

| | | |
|---|---|---|
| MCU ↔ host link | micro-ROS over UART, 115200 baud | C |
| Patrol route | 163 / 50 / 163 / 50 cm (426 cm total) | C |
| Room | 2.23 m × 1.0 m | M |
| Full mission run end to end | Yes, 2026-09-06 | M |

This stacks four error sources — stereo distance, bearing angle, odometry
position and heading, and the running mean over sightings — and three placements
cannot separate them. Strong evidence the *system* works; no evidence about any
component. The 16 cm outlier's leading suspect is `duck_estimate()` using a plain
mean with no outlier rejection.

---

## 8. Verified vs not verified

| Claim | Status |
|---|---|
| Centroid ~10 % over 0.3–1.0 m | ✅ 2026-09-04 |
| Centroid ~1 % at 1.30 m | ✅ measured, but a single point of a four-point sweep |
| Centroid ~1 % across the full range | ❌ **disproven** by §4.1 |
| Dense stereo 6–10 % below 0.5 m | ✅ 2026-08-29 |
| Dense stereo unreliable beyond 0.5 m | ✅ 2026-09-04 |
| End-to-end 1–4 cm while driving | ✅ 2026-09-06, three placements |
| Repeatability (mean ± σ) of anything above | ❌ every figure is a single placement |
| Odometry drift, straight-line error, turn accuracy | ❌ never measured |
| Bearing (lateral angle) sign | ❌ never empirically verified — flagged in code since written |
| Centroid gate (30 %) triggering correctly | ❌ deployed, never observed |
| Detector accuracy (mAP) | ❌ never evaluated |

---

## 9. Comparing against published work

**The metric-mismatch trap.** Stereo papers report **`bad-2.0`**: the percentage
of *pixels* in a dense disparity map whose disparity is off by more than 2 px on a
public dataset. That is not the metric here. Quoting "our 10 % vs their X %" would
be a methodological error. The honest bridge is §3: a method achieving 0.5 px
disparity accuracy would give ~0.9 % distance error *on this rig*.

| | This robot | Intel RealSense D435 |
|---|---|---|
| Baseline | 85.4 mm | ~50 mm |
| Depth method | Passive stereo, ambient light | Active IR stereo (**pattern projector**) |
| Useful range | ~0.3–1.5 m (measured) | 0.3–3 m (specified) |
| Accuracy | ~10 % at 0.3–1.0 m | **better than 2 % at 2 m** (datasheet) |

**Why the D435 is the fair comparison** — same metric type, and its IR projector
exists precisely to solve the textureless-surface failure in §4.2. The gap is a
missing *hardware* capability, not a coding mistake.

**FoundationStereo** (NVIDIA, CVPR 2025 Oral, Best Paper Nomination) — 1M synthetic
training pairs, zero-shot, **1st on both Middlebury and ETH3D**. The research
ceiling *and* a real upgrade path: it officially supports Jetson Orin with an
ONNX/TensorRT route, and Fast-FoundationStereo reports **>10× speed-up** at close
to the same accuracy — directly relevant to the ~1 s/frame in §4.3.

**NVIDIA Isaac ROS ESS** — NVIDIA's production stereo depth for this hardware
family documents the same limitation: disparity for *"highly reflective and
textureless surfaces is not reliably measured."* Independent third-party
confirmation of §4.2.

---

## 10. Where the evidence actually is

| Layer | Benchmarked? |
|---|---|
| Perception | **Well** — multiple sweeps, a derived physical floor, characterisation data |
| State estimation | **Barely** — no drift, heading or turn-accuracy number exists |
| Control | **Barely** — gains documented, no closed-loop response measured |
| Integration | **Once** — three placements, one session |

**The gap is not in the hard layer.** Perception — the part that looks hardest —
is the best measured. Estimation and control, the two layers everything else sits
on, have almost no numbers. That matters concretely: when the system reports a
16 cm error, there is no way to say how much came from stereo and how much from
odometry drift.

Three cheap fixes, each under 20 minutes:

1. **Loop drift** — the route returns to its own start, so the position error
   already printed on reaching the final waypoint *is* the measurement. Zero code.
2. **Turn accuracy** — the manual turn tester exists; command 90°, read the
   reported angle, repeat five times.
3. **Straight-line drift** — command 1.63 m forward, measure lateral deviation.

---

## Sources

**Internal:** `README.md` (2026-08-27 / 08-29 / 09-04 / 09-06 session entries),
`COMMIT_HISTORY.md`, `stereo_calibration.npz`, `motor_f1.c`, PRs #67 / #68.

**External:**
- [FoundationStereo: Zero-Shot Stereo Matching, CVPR 2025](https://arxiv.org/abs/2501.09898) · [project](https://nvlabs.github.io/FoundationStereo/) · [code](https://github.com/NVlabs/FoundationStereo)
- [Fast-FoundationStereo](https://nvlabs.github.io/Fast-FoundationStereo/)
- [NVIDIA Isaac ROS DNN Stereo Depth (ESS)](https://nvidia-isaac-ros.github.io/concepts/stereo_depth/ess/index.html)
- [Middlebury Stereo Evaluation v3](https://vision.middlebury.edu/stereo/eval3/) — `bad-2.0` definition
- [Intel RealSense D435 specifications](https://www.intel.com/content/www/us/en/products/sku/128255/intel-realsense-depth-camera-d435/specifications.html) — verify the exact datasheet line before quoting in a submitted report
