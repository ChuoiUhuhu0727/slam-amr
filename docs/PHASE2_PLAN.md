# Phase 2 — GPU-Accelerated Perception and Localisation

**Duration:** 11 weeks (~2.5 months) · **Platform:** Jetson Orin Nano Super,
existing robot · **Status:** proposed

---

## The thesis

> Replace this robot's classical CPU perception and dead-reckoning localisation
> with NVIDIA's GPU-accelerated stack, and **measure what it actually buys** —
> in accuracy, in latency, and in failure modes.

**The reason this works as a project and not just as learning:** the baseline
already exists and is already measured. Most students cannot run a before/after
because they never recorded "before." This project has
[`BENCHMARKS.md`](BENCHMARKS.md) — ruler sweeps, a derived physical floor,
timing data. Phase 1 built the control group. Phase 2 is the treatment.

That reframes every week of work from "I learned Isaac ROS" into "I replaced X
with Y and here is the measured difference," which is a fundamentally stronger
claim.

## What problem each part actually solves

Both targets come from measured deficits, not from a technology wish-list:

| Measured deficit | Current value | Target |
|---|---|---|
| Vision loop rate (SGBM on CPU is ~95 % of it) | **0.54 Hz** against a 5 Hz target | ≥ 5 Hz |
| Dense stereo beyond 0.5 m | **Unusable** — locks onto background on textureless targets | usable to 1.5 m |
| Position correction | **None** — odometry drifts with nothing to correct it | bounded drift |

---

## Phase 0 — Lock the baseline (Week 1)

**Nothing can be compared to a baseline that has error bars nowhere.** This week
closes the open items already listed in `BENCHMARKS.md` §8.

- Odometry drift over one patrol loop — the error is *already printed* on
  reaching the final waypoint; record it across 5 runs
- Straight-line lateral drift over 1.63 m, 5 trials
- Turn accuracy since the ramp-down fix, 5 trials at 90°
- Distance accuracy with **repeat trials**: 5 placements per distance at
  0.3 / 0.5 / 1.0 / 1.3 m, reporting mean ± σ
- Per-stage timing breakdown of the vision loop (capture / remap / YOLO ×2 /
  SGBM / transform), so the 1,850 ms is attributed rather than assumed
- Verify the bearing sign empirically, finally

**Deliverable:** baseline report with error bars. Every later claim measures
against this.

**Why first:** it is also the cheapest week. If the project stalls later, this
week alone still closes six open items and improves the existing report.

---

## Phase 1 — Camera arbitration + GPU stereo (Weeks 2–4)

### 1a. The architectural blocker (Week 2)

Isaac ROS claims the cameras exclusively and cannot coexist with the mission
node's own capture. This is documented in the code as the reason Isaac ROS was
excluded — so it must be solved before anything else.

**The fix is the lesson:** make camera capture a proper ROS 2 publisher node
that everything else subscribes to, instead of a private `cv2.VideoCapture`
owned by the mission node. This is how real robot systems are structured, and
doing it on a system you already understand is worth more than reading about it.

**Deliverable:** mission node and Isaac ROS consuming the same camera stream
simultaneously. Measure the cost — republishing images through ROS 2 is not
free; quantify the added latency.

### 1b. Swap SGBM for Isaac ROS ESS (Weeks 3–4)

**Deliverable:** the same ruler benchmark as Phase 0, run against ESS instead of
SGBM. Report latency and accuracy side by side.

**The interesting question, not the obvious one:** NVIDIA's own ESS
documentation lists textureless surfaces as a known limitation. So this is a
real hypothesis test, not a guaranteed win — *does a learned dense method
actually solve the failure that block matching could not, or does it inherit
it?* Either answer is a result. A negative result here is genuinely publishable
at student level, because it is measured.

---

## Phase 2 — FoundationStereo via ONNX → TensorRT (Weeks 5–6)

Now TensorRT is worth learning, because here it changes an outcome rather than
shaving 30 ms off a 1,850 ms loop.

- Export FoundationStereo (or Fast-FoundationStereo) to ONNX, build a TensorRT
  engine for this Jetson, integrate
- Run the identical ruler benchmark a third time

**Deliverable:** a three-way comparison on one benchmark — classical SGBM vs
Isaac ROS ESS vs FoundationStereo — across accuracy, latency and failure mode on
textureless targets.

**This is the centrepiece of the whole project.** A three-way controlled
comparison on real hardware, against tape-measured ground truth, with a derived
theoretical floor to interpret it against, is a stronger artefact than any
single implementation.

---

## Phase 3 — Localisation (Weeks 7–9) · **stretch**

Closes the last structural gap: nothing corrects position drift.

Pick up cuVSLAM where it was left. Per the existing notes, TF, `camera_info` and
stereo synchronisation have all been ruled out by measurement; the remaining
symptom is tracking loss under real motion, and the untried step is a visual
check through RViz2 / `rqt_image_view`.

**Deliverable:** loop-closure drift with and without VSLAM, over the same patrol
route, 5 runs each.

> **Explicitly marked stretch.** cuVSLAM has resisted this project more than any
> other component, across months. Phases 0–2 must stand as a complete result
> without it. If Phase 3 fails, it is written up as a documented negative result
> with the evidence — which this project already does well for Nav2.

---

## Week 10 — Consolidation · Week 11 — Buffer

Week 10: fold results into `BENCHMARKS.md` and `FULL_REPORT.md`, record a demo
video, clean the repo.

Week 11 is **deliberately empty**. Something will overrun. A plan with no slack
is a plan that fails in week 9.

---

## Kill criteria — decide in advance, not in the moment

Written now, while not frustrated, so they can be applied later when frustrated.

| Situation | Rule |
|---|---|
| Any single blocker consumes **more than 4 days** | Log it, de-scope that phase, move to the next. The project has enough independent phases to survive losing one. |
| Phase 1a (camera arbitration) not working by end of Week 2 | Fall back: run ESS offline on recorded `ros2 bag` data instead of live. Loses the live demo, keeps the entire accuracy comparison. |
| ESS or FoundationStereo will not build on this JetPack | That is a **finding**, not a failure. Document the exact versions and errors — environment friction is the single most under-reported part of the NVIDIA robotics stack, and the writeup is genuinely useful. |
| Phase 3 stalls | Drop it. It is already marked stretch. |

**The general rule this project has already learned once:** a sub-problem that
does not block the current phase gets logged, not chased.

---

## What makes this employable, in one line

Not "used Isaac ROS." It is:

> *"I replaced a classical CPU stereo pipeline with two different GPU-accelerated
> alternatives on a Jetson, benchmarked all three against tape-measured ground
> truth with repeat trials, and can tell you which one to pick, at what range,
> and why."*

Very few graduates can say the second sentence. The difference is not the
technology — it is the measurement discipline this project already has.

---

## Open scoping question

**Isaac Sim / Isaac Lab is deliberately not in this plan.** It carries the
highest job-market value of anything in the NVIDIA robotics ecosystem, but the
Orin Nano cannot run it — it needs a desktop with an RTX GPU.

If such a machine is available, the strongest variant swaps Phase 3 for:

> **Phase 3′ — Simulate this robot in Isaac Sim, and measure sim-to-real gap.**
> Build the robot in sim, run the identical patrol, compare simulated odometry
> drift and detection performance against the real measurements from Phase 0.
> A quantified sim-to-real gap on a robot you also own physically is a rarer
> artefact than another perception benchmark.

If no RTX machine is available, keep Phase 3 as written and do not let this
block anything.
