# System Benchmarks — All Four Layers

Every number this project has actually measured, organised by autonomy layer,
with an explicit marker for what is measured and what is only configured.

Legend: **M** = measured against ground truth · **C** = configured value, not a
result · **—** = never measured.

Perception detail lives in [DEPTH_ACCURACY_BENCHMARK.md](DEPTH_ACCURACY_BENCHMARK.md);
this file is the one-page view across the whole stack.

---

## Layer 1 — Perception

### Calibration

| Quantity | Value | |
|---|---|---|
| Stereo reprojection error | 0.33 px | M |
| Calibrated baseline | 85.40 mm (vs 83 mm measured, 2.9 % off) | M |
| Rectified focal length | 875.51 px | M |
| Resolution | 1280 × 720 | C |
| fx × baseline | 74.77 | derived |

### Depth resolution — the physical floor

Cost of **one pixel** of disparity error, derived from calibration:

| Distance | 0.30 m | 0.50 m | 1.00 m | 1.30 m | 1.50 m |
|---|---|---|---|---|---|
| Error | 0.40 % | 0.67 % | 1.34 % | 1.74 % | 2.01 % |

### Distance accuracy — centroid triangulation (robot stationary)

| Ground truth | 2026-08-27 | 2026-09-04 (post photometric fix) | |
|---|---|---|---|
| 0.30 m | — | 0.34 m (+13 %) | M |
| 0.50 m | 0.46 m (−8 %) | 0.54–0.59 m (+9 to +18 %) | M |
| 1.00 m | 1.06 m (+6 %) | 1.10 m (+10 %) | M |
| 1.30 m | — | 1.30 m (~1 %) | M |
| 1.50 m | 1.35 m (−10 %) | — | M |

**Headline: ~10 % over 0.3–1.0 m.** The 1.30 m point is one reading in a
four-point sweep and is not the system's typical accuracy.

### Distance accuracy — dense stereo / point cloud

| Range | Result | |
|---|---|---|
| 0.30 m | 0.28 m (6.7 %) | M |
| 0.50 m | 0.47 m (6 %) | M |
| 1.00 m | 0.89–0.91 m (~10 %) | M |
| Above 0.5 m | Unusable — at 1.3 m returns three clusters (0.37–0.44 / 1.53–1.56 / 2.43–2.46 m) | M |

Cause: the target's smooth surface has no texture for SGBM to match, so the
matcher locks onto textured background at the box edges. Now gated against the
centroid method at a 30 % disagreement threshold.

### Camera characterisation

| Quantity | Value | |
|---|---|---|
| L/R brightness offset | 3.2 / 255 (~3 %), stable (std 0.28 / 0.29) | M |
| L/R capture skew | 2.38 ms mean (0.48–4.92 ms) | M |
| Worst-case theoretical skew | ~16.7 ms (one frame at 60 fps) | derived |
| Point-cloud compute | ~1000–1050 ms/frame, 8k–24k valid points | M |
| Vision loop rate achieved | **0.54 Hz** (target 5 Hz) — YOLO + SGBM bound | M |

### Detector

| Quantity | Value | |
|---|---|---|
| Detection accuracy (mAP, precision/recall) | **—** never formally evaluated | — |
| Known failure mode | Rectified input collapses accuracy; must detect on raw frames | M |

---

## Layer 2 — State Estimation ⚠️

**This layer has essentially no benchmarks.** It is the weakest-evidenced part
of the system, and it sits underneath every position the robot reports.

| Quantity | Value | |
|---|---|---|
| Odometry publish rate | 20 Hz | C |
| Wheel diameter / wheelbase | 67 mm / 100 mm | M (tape) |
| Encoder resolution | 20 slots/rev → 10.5 mm per pulse | C |
| IMU | MPU6050, gyro ±250 °/s (131 LSB/(°/s)) | C |
| Heading source | Gyro-fused, not encoder-differential | C |
| **Drift over one patrol loop** | **— never measured** | — |
| **Straight-line heading error over 1.63 m** | **— never measured** | — |
| **Turn accuracy after the ramp-down fix** | **— never measured** (pre-fix: commanded 90° landed ~144°) | — |
| Drive-straight behaviour | Confirmed working qualitatively 2026-08-24, no number | — |

**The patrol route returns to its own start point, so the position error
reported on arrival is a free, direct measurement of loop drift.** It has never
been recorded. This is the single cheapest missing number in the project.

---

## Layer 3 — Control ⚠️

Configuration is fully documented; **closed-loop performance is not measured.**

| Quantity | Value | |
|---|---|---|
| Control loop rate | 20 Hz (50 ms) | C |
| PWM carrier | 1 kHz (LEDC) | C |
| Max linear / angular speed | 0.30 m/s / 1.5 rad/s | C |
| Waypoint arrival tolerance | 100 mm | C |
| Turn-in-place threshold | 0.9 rad (~50°) | C |
| Turn-done tolerance | 2° | C |
| PID gains | Kp 3.0, Ki 0.2, Max-I 40 | C |
| Heading gains | Khead 15, max trim 20 RPM; Klock 2.0, max lock 0.6 | C |
| **Step response / settling time / overshoot** | **— never measured** | — |
| **Steady-state RPM error vs target** | **— never measured** | — |
| RPM measurement method | Pulse-interval timing (replaced pulse counting, which quantised to 60 RPM steps) | M |
| PWM headroom bug found | Ceiling was 78/255, then 100–130; both root-caused and removed | M |

---

## Layer 4 — Integration (the end-to-end result)

| Quantity | Value | |
|---|---|---|
| **Target position error, robot driving** | **0.31→0.30 m (1 cm) · 0.90→0.74 m (16 cm) · 1.04→1.00 m (4 cm)** | M |
| MCU ↔ host link | micro-ROS over UART, 115200 baud | C |
| Patrol route | 163 / 50 / 163 / 50 cm, 426 cm total | C |
| Room | 2.23 m × 1.0 m | M (tape) |
| Full loop completed end to end | Yes, 2026-09-06 | M |
| Repeatability of any figure above | **— every number is a single placement** | — |

This is the number the project should be judged on. It stacks all four layers
and therefore attributes error to none of them.

---

## Summary: where the evidence actually is

| Layer | Benchmarked? |
|---|---|
| Perception | **Well** — multiple sweeps, a derived physical floor, characterisation data |
| State estimation | **Barely** — no drift, heading or turn-accuracy number exists |
| Control | **Barely** — gains documented, no closed-loop response measured |
| Integration | **Once** — three placements, one session |

**The gap is not in the hard layer.** Perception — the part that looks hardest —
is the best-measured. Estimation and control, the two layers everything else
sits on, have almost no numbers, and both are cheap to fix:

1. **Loop drift** — record the position error already printed when the robot
   reaches its final waypoint. Zero new code.
2. **Turn accuracy** — the manual turn tester already exists; command 90°, read
   the reported angle, repeat five times.
3. **Straight-line drift** — command 1.63 m forward, measure lateral deviation
   with a tape.

Each is under 20 minutes and each converts a "— never measured" row above into a
real number.
