# Duck-Distance Accuracy — Measured Results & External Benchmarks

Reference document for the stereo distance-estimation accuracy of this robot's
search-and-rescue pipeline. Written 2026-09-04, from measurements logged in
`README.md` and `COMMIT_HISTORY.md` — every number here traces to a dated live
run on the real rig, not a simulation or an estimate.

**Read the metric definition first.** The single most common way to misreport
this kind of result is to compare a number against a research paper that is
measuring something else entirely. See [§6](#6-comparing-against-external-work).

---

## 1. What is actually being measured

> **Metric:** absolute error in the estimated straight-line distance from the
> left camera's optical centre to *one specific detected object* (a rubber
> duck), expressed as a percentage of the tape-measured ground-truth distance.
>
> **Protocol:** duck placed static at a tape-measured distance, robot
> stationary, distance read off the live pipeline over many consecutive frames.
>
> **Not measured:** dense per-pixel depth quality, disparity error on any
> standard dataset, lateral (x/y) position error, or accuracy while the robot
> is moving.

This is an *object-level metric-depth* metric. It is deliberately the number
that matters for the mission — "how far away is the duck" — not the number
stereo-matching papers optimise for.

---

## 2. Hardware and configuration under test

| Item | Value | Source |
|---|---|---|
| Cameras | 2× IMX219 CSI, 160° FOV, rolling shutter | — |
| Resolution | 1280×720 (sensor mode 4) | `stereo_calibration.npz` |
| Physical baseline | 83 mm (tape-measured) | 2026-08-04 |
| Calibrated baseline | **85.40 mm** (2.4% off physical) | `stereo_calibration.npz` `baseline_m` |
| Rectified focal length | **fx = 875.51 px** | `cv2.stereoRectify` → `P1[0,0]` |
| Calibration reprojection error | ~0.33 px | 2026-08-04 |
| Compute | Jetson Orin Nano Super, JetPack 6.x | — |
| Detector | YOLO (`best.pt`), run on **raw** un-rectified frames | 2026-08-25 |
| Duck height (for size-sanity gate) | 0.13 m | 2026-08-24 |

No active IR projector, no structured light — passive stereo on ambient room
light only. This matters; see [§6.2](#62-intel-realsense-d435--the-fair-apples-to-apples-comparison).

---

## 3. The physical floor: what this rig *can* achieve

Before comparing to anything, it's worth knowing what the hardware allows.
Depth from disparity is

```
Z = fx · B / d          →          ΔZ = Z² / (fx · B) · Δd
```

With `fx · B = 74.77`, **one pixel of disparity error costs**:

| True distance | Disparity | Error from 1 px | as % of distance |
|---:|---:|---:|---:|
| 0.30 m | 249.2 px | 1.2 mm | **0.40 %** |
| 0.50 m | 149.5 px | 3.3 mm | **0.67 %** |
| 0.75 m | 99.7 px | 7.5 mm | **1.00 %** |
| 1.00 m | 74.8 px | 13.4 mm | **1.34 %** |
| 1.30 m | 57.5 px | 22.6 mm | **1.74 %** |
| 1.50 m | 49.8 px | 30.1 mm | **2.01 %** |
| 2.00 m | 37.4 px | 53.5 mm | **2.68 %** |

**Three consequences worth internalising:**

1. **The 1/x relationship is the whole story of this project's range problem.**
   Error growth with distance is not a bug to be fixed — it is the geometry.
   Any accuracy claim is meaningless without stating the distance it was
   measured at.
2. **A "1 % at 1.3 m" result requires sub-pixel disparity accuracy** (1 % of
   1.30 m = 13 mm ≈ **0.58 px**). That is a real achievement, and also a
   reason to be sceptical of a single-session measurement.
3. **A 6–10 % error at these ranges is 5–15 px of disparity error**, far above
   quantisation. Those were real systematic faults, not noise floor.

---

## 4. Measured results

Two independent methods run side by side on the same detection box.

### Method A — Centroid disparity
Single disparity value from the horizontal offset between the duck's box
centroid in the left vs right rectified frame. One number per frame, no
averaging. Median-smoothed over the last 8 readings for display.

### Method B — Point cloud (dense)
`cv2.StereoSGBM` on the full rectified pair → `cv2.reprojectImageTo3D` →
median of the valid 3D points inside the duck's box (core region, eroded 25 %
per side). Rate-limited to ~1 Hz.

---

### 4.1 Ruler benchmark — 2026-08-27 (Method A, before photometric fix)

| Ground truth | Measured | Error | Implied disparity error |
|---:|---:|---:|---:|
| 0.50 m | 0.46 m | **8.0 %** | ~13 px |
| 1.00 m | 1.06 m | **6.0 %** | ~4 px |
| 1.50 m | 1.35 m | **10.0 %** | ~6 px |

Raw single-frame readings at 1.5 m fluctuated **1.2–1.7 m**. Root-caused to the
1/x relationship above: at range, disparity is small, so a few px of box jitter
swings the computed distance hard. Motivated the rolling-median window and the
whole Method B effort.

### 4.2 Ruler benchmark — 2026-08-29 (Method B, after core-region fix)

| Ground truth | Measured | Error | Before the fix |
|---:|---:|---:|---:|
| 0.30 m | 0.28 m | **6.7 %** | 0.43 m (43 % err) |
| 0.50 m | 0.47 m | **6.0 %** | 0.50 m |
| 1.00 m | 0.89–0.91 m | **~10 %** | 0.92–1.29 m (37 cm swing) |

The fix: a loosely-fitted detection box let background pixels at the box edges
drag the median toward "farther." Eroding the box 25 % per side before taking
the median removed it. Residual **5–11 % bias logged as open** — cheap
hypotheses exhausted, would need a controlled repeat-trial experiment.

### 4.3 Range-instability finding — 2026-09-04 (both methods, at 1.3 m)

After the photometric L/R brightness normalisation (PR #67):

| Method | Reading at 1.30 m ground truth | Error |
|---|---|---|
| **A — centroid** | **1.30–1.32 m, every single tick** | **~1 %** |
| B — point cloud | three discrete clusters: 0.37–0.44 m / 1.53–1.56 m / 2.43–2.46 m | catastrophic |

Both used the *exact same detection box*. Method B was locking onto background,
not the duck: the duck's smooth, untextured surface gives SGBM too little to
match on, so the strongest match inside the box is often the textured floor or
wall behind it. Confirmed not to be a crop-size problem — both adaptive
core-region sizing and a fixed 25 % margin were tested live and both still
produced it.

**Shipped fix (PR #68):** gate Method B against Method A. If B's distance
disagrees with A's by more than 30 %, rescale B's position vector onto A's
distance — keeps B's lateral direction, discards a depth that probably came
from the wrong object.

---

## 5. Current status — what is and is not verified

| Claim | Status |
|---|---|
| Method A ≈ 1 % at 1.30 m | ✅ Measured live, many consecutive ticks, 2026-09-04 |
| Method A 6–10 % at 0.5–1.5 m | ✅ Measured — **but before the photometric fix** |
| Method A ≈ 1 % across the full 0.3–1.5 m range | ❌ **Never measured.** Only 1.3 m was re-run post-fix |
| Method B 6–10 % below 0.5 m | ✅ Measured 2026-08-29 |
| Method B unreliable beyond ~0.5 m | ✅ Measured 2026-09-04 |
| The centroid gate (PR #68) works | ❌ **Shipped but never live-verified** |
| Any accuracy while the robot is *moving* | ❌ Never measured |
| Duck position in room coordinates | ❌ Never measured — inherits odometry drift on top of all the above |

> **The honest headline is:** *"~1 % error at 1.30 m, single distance, single
> session, robot stationary."* Not *"~1 % accurate."* The difference is the
> whole credibility of the claim.

### 5.1 The one experiment that would fix this

Re-run the 2026-08-27 protocol (0.3 / 0.5 / 1.0 / 1.5 m) post-photometric-fix,
**with repeat trials** — say 5 placements per distance, logging every frame —
and report mean error *and* standard deviation per distance. That converts a
single anecdote into a curve that can be plotted against the theoretical floor
in [§3](#3-the-physical-floor-what-this-rig-can-achieve). It is a
~30-minute experiment and it is the single highest-value thing left to do on
this pipeline.

---

## 6. Comparing against external work

### 6.1 The metric-mismatch trap — read before quoting any paper

Stereo-matching research almost universally reports **`bad-2.0`**: the
*percentage of pixels* in a dense disparity map whose disparity is off by more
than 2 pixels, on a fixed public dataset.

That is **not** the metric in this document. Ours is *percent error in metric
distance to one object*. They are not comparable, and quoting "our 1 % vs their
bad-2.0 of X %" in a report would be a straightforward methodological error
that any reviewer would catch immediately.

Two things genuinely *are* comparable, and both are useful:

- **Against a commercial depth sensor's spec** — same metric type (metric depth
  error as % of range). This is the fair comparison.
- **Against a research model's *disparity* accuracy, converted through the
  table in §3** — e.g. a method achieving 0.5 px disparity accuracy would give
  ~0.9 % depth error at 1.3 m *on this rig*. This lets a paper's number be
  translated into our units honestly.

### 6.2 Intel RealSense D435 — the fair apples-to-apples comparison

The reference point for "is this good for a robot depth sensor."

| | This rig | RealSense D435 |
|---|---|---|
| Baseline | 85.4 mm | ~50 mm |
| Depth tech | Passive stereo, ambient light | Active IR stereo (**pattern projector**) |
| Ideal range | ~0.3–1.5 m (measured) | 0.3–3 m (spec) |
| Stated accuracy | ~1 % @ 1.3 m *(single session)* | **< 2 % @ 2 m** (datasheet) |

**This is the most useful comparison in the document**, for one reason beyond
the numbers: the D435's IR pattern projector exists *specifically* to solve the
failure this project hit on 2026-09-04. Projecting texture onto surfaces is how
you make a smooth, untextured object (a rubber duck) matchable by a dense
stereo algorithm. Method B's background-lock is not a bug in the SGBM
parameters — it is the known, structural limitation of passive dense stereo,
and the industry's answer to it is extra hardware, not better code.

That reframes the centroid gate from "a workaround" into "the correct
engineering response given passive-stereo hardware," which is a much stronger
thing to defend in a demo.

### 6.3 FoundationStereo — NVIDIA, CVPR 2025 (Oral, Best Paper Nomination)

*Wen, Trepte, Aribido, Kautz, Gallo, Birchfield — NVIDIA Labs.*
[Paper](https://arxiv.org/abs/2501.09898) ·
[Project page](https://nvlabs.github.io/FoundationStereo/) ·
[Code](https://github.com/NVlabs/FoundationStereo)

The current state of the art in zero-shot stereo matching, and the right
"research ceiling" reference for this project:

- Trained on **1M synthetic stereo pairs** with a self-curation pipeline, plus a
  side-tuning backbone that adapts monocular foundation-model priors to close
  the sim-to-real gap.
- **1st place on both the Middlebury and ETH3D leaderboards** (Middlebury's
  headline metric is `bad-2.0` — see the mismatch warning in §6.1).
- **Zero-shot** — no per-domain fine-tuning, which is exactly this project's
  situation (a room and a duck that appear in no training set).

Why this one specifically, beyond being NVIDIA and highly cited:

1. **It officially supports Jetson** (Jetson Orin listed as tested hardware, ONNX/TensorRT
   path added 2025-07), so it is a *real upgrade path*, not just a paper to cite.
2. **Fast-FoundationStereo** (2025-12) claims **>10× faster** than the base model
   at close to the same zero-shot accuracy — directly relevant, since Method B's
   SGBM currently costs ~1000–1050 ms/frame (~1 Hz) on this Jetson.
3. There is an **NVIDIA TAO commercial variant** on NGC, meaning a
   deployment-ready packaging exists.

**How to use it in a report, honestly:** as the ceiling and the future-work
direction — "our passive-stereo pipeline reaches ~1 % at 1.3 m on a single
object using classical triangulation; a learned dense-stereo foundation model
such as FoundationStereo represents the current SOTA and runs on this same
Jetson class, and is the natural next step for making the dense point-cloud
branch reliable at range." Do **not** claim a numeric win or loss against it —
different metric, different task.

### 6.4 NVIDIA Isaac ROS DNN Stereo Depth (ESS) — the deployed-on-Jetson baseline

[Docs](https://nvidia-isaac-ros.github.io/concepts/stereo_depth/ess/index.html) ·
[Repo](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_dnn_stereo_depth)

Worth naming because it is NVIDIA's *productised* stereo depth for exactly this
hardware, and this project deliberately chose not to use it (needs the full
Isaac ROS container, which claims the cameras exclusively).

NVIDIA's own documented limitations for ESS are a direct, citable
third-party confirmation of the 2026-09-04 finding: disparity for **"highly
reflective and textureless surfaces is not reliably measured."** A rubber duck
is a textureless surface. Being able to point at NVIDIA's own docs saying their
production DNN has the same failure mode is a strong defence of the design
decision made here.

---

## 7. Summary table for a report

| Method | Range | Error | Conditions | Verified |
|---|---|---|---|---|
| Centroid disparity | 1.30 m | **~1 %** | static, post-photometric-fix | single session |
| Centroid disparity | 0.5–1.5 m | 6–10 % | static, pre-photometric-fix | 2026-08-27 |
| Dense point cloud | 0.3–0.5 m | 6–10 % | static, core-region median | 2026-08-29 |
| Dense point cloud | > 0.5 m | unusable | background-lock on textureless duck | 2026-09-04 |
| Theoretical floor (1 px) | 1.30 m | 1.74 % | geometry of this rig | derived |
| *RealSense D435 (reference)* | *2.0 m* | *< 2 %* | *active IR stereo* | *datasheet* |

---

## Sources

Internal: `README.md` (2026-08-27, 2026-08-29, 2026-09-04 session entries),
`COMMIT_HISTORY.md`, `stereo_calibration.npz`, PRs #67 / #68.

External:
- [FoundationStereo: Zero-Shot Stereo Matching (CVPR 2025)](https://arxiv.org/abs/2501.09898) — [project page](https://nvlabs.github.io/FoundationStereo/), [code](https://github.com/NVlabs/FoundationStereo)
- [Fast-FoundationStereo](https://nvlabs.github.io/Fast-FoundationStereo/)
- [NVIDIA Isaac ROS DNN Stereo Depth (ESS)](https://nvidia-isaac-ros.github.io/concepts/stereo_depth/ess/index.html)
- [Middlebury Stereo Evaluation v3](https://vision.middlebury.edu/stereo/eval3/) — `bad-2.0` metric definition
- [Intel RealSense D435 product specifications](https://www.intel.com/content/www/us/en/products/sku/128255/intel-realsense-depth-camera-d435/specifications.html) — verify the exact datasheet accuracy line before quoting it in a submitted report
