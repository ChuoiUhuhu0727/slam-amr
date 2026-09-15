# slam-amr — Full Commit History (weekly report source)

Generated 2026-08-07 from `git log origin/main` (merge commits excluded, 89 commits total, 2026-06-25 to 2026-08-07).

Grouped by calendar week (Mon-Sun). Each entry: commit hash, timestamp, subject, then the full commit body if one exists (these bodies are this project's real devlog — root causes, decisions, what was ruled out — use them, not just subjects).

## Weeks at a glance

| Week | Date range | Commits |
|------|-----------|---------|
| [Week 1](#week-1-2026-06-22-to-2026-06-28) | 2026-06-22 to 2026-06-28 | 6 |
| [Week 2](#week-2-2026-06-29-to-2026-07-05) | 2026-06-29 to 2026-07-05 | 6 |
| [Week 3](#week-3-2026-07-13-to-2026-07-19) | 2026-07-13 to 2026-07-19 | 3 |
| [Week 4](#week-4-2026-07-20-to-2026-07-26) | 2026-07-20 to 2026-07-26 | 14 |
| [Week 5](#week-5-2026-07-27-to-2026-08-02) | 2026-07-27 to 2026-08-02 | 35 |
| [Week 6](#week-6-2026-08-03-to-2026-08-09) | 2026-08-03 to 2026-08-09 | 25 |

---

## Week 1 (2026-06-22 to 2026-06-28)

### 2026-06-25 14:47 — Day 1: project initialized, ROS2 Humble + micro-ROS agent running (`3118c64`)

### 2026-06-25 22:51 — Add micro-ROS hello world firmware (auto-reconnect) (`ac33e11`)

### 2026-06-25 23:05 — Add README (`bf616e5`)

### 2026-06-25 23:12 — Update README.md (`a37500a`)

### 2026-06-25 23:13 — Update README with system pipeline and repository structure (`e251cdd`)

Added a new section detailing the system pipeline and repository structure.

### 2026-06-25 23:14 — Update README.md (`d965c05`)

---

## Week 2 (2026-06-29 to 2026-07-05)

### 2026-06-29 11:44 — Add project spec (`f7c49e2`)

### 2026-06-29 12:11 — Update README with I/O, business context, pipeline diagram (`874b1be`)

### 2026-06-29 17:19 — Switch power source to powerbank, remove LiPo+LM2596 (`fbc735c`)

### 2026-07-01 22:00 — Add PID pipeline diagram (`1048354`)

### 2026-07-01 22:04 — Add PID pipeline diagram (`ace49ef`)

### 2026-07-01 22:05 — Update README.md (`8807c1a`)

---

## Week 3 (2026-07-13 to 2026-07-19)

### 2026-07-17 10:56 — Add ESP32 motor_f1 firmware, H-Bridge diagram, VSCode config (`dbdd211`)

### 2026-07-17 12:48 — Add hardware wiring diagrams to README, merge motor_f1 docs (`0b92fa7`)

Adds TB6612FNG H-bridge schematic and an ESP32<->driver<->motor<->encoder
wiring diagram (mermaid) reflecting real GPIO pin usage, with IMU shown
as planned-but-not-yet-wired. Consolidates esp32/motor_f1/README.md
(pin map, build/flash steps, gotchas) into the top-level README as a
single source of truth.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>

### 2026-07-17 12:59 — Replace wiring diagram with a table for readability (`4c13627`)

The mermaid flowchart had too many crossing edges to scan quickly.
A flat pin table covers the same ESP32<->TB6612FNG<->encoder<->IMU
mapping and is easier to read at a glance.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>

---

## Week 4 (2026-07-20 to 2026-07-26)

### 2026-07-20 18:14 — Add ROS2 topic interface and team work-split docs to README (`16058d5`)

Documents the ESP32/Jetson topic contract and splits ownership between
on-site hardware work and remote software/config work, so the remote
teammate has enough context to build independently.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>

### 2026-07-20 18:20 — Name Alex as the remote teammate in README work split (`362531c`)

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>

### 2026-07-21 17:13 — Add F2 encoder_task to motor_f1: ISR pulse counting + RPM calc (`0fac8ac`)

Merged into motor_f1 rather than a separate project since F3 (PID)
needs both motor and encoder together anyway. Read-then-reset of the
shared pulse counters is wrapped in a critical section to avoid losing
a pulse that arrives between the read and the reset.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>

### 2026-07-21 17:30 — Disable motor enable for isolated F2 encoder test (`c5c2610`)

Temporary: comment out STBY enable so motors stay off while
hand-spinning wheels to verify encoder RPM readout alone.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>

### 2026-07-21 17:36 — Fix wrong FreeRTOS spinlock macro in encoder_task (`3e7da05`)

portMUX_INITIALIZE_DEFAULT doesn't exist in ESP-IDF; the correct
static initializer is portMUX_INITIALIZER_UNLOCKED. Caught by the
compiler on first build attempt.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>

### 2026-07-21 17:57 — Document F2 encoder milestone: pin map, test procedure, gotchas (`51436c9`)

Confirmed working on real hardware. Bakes in two cheap habits going
forward: pipe monitor output to a timestamped log file, and note the
RPM quantization (steps of 3) so it isn't mistaken for PID noise later.
Also logs the plotting-script TODO for F3, deliberately deferred since
it isn't worth building until PID tuning needs to see response curves.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>

### 2026-07-21 19:43 — Add F3 pid_task: P-only velocity control, re-enable motor (`8ef442d`)

pid_step() computes error = target - actual, u = Kp * error, clamped
to the LEDC 8-bit duty range [0,255]. Hardcoded TARGET_RPM=30 and
Kp=3.0 as a starting point for tuning. Re-enables STBY (was disabled
for the isolated F2 encoder test) since F3 needs the motor running.
Ki/Kd deliberately deferred until Kp is tuned, per the Week 2 plan.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>

### 2026-07-22 00:03 — Document power architecture issue found during F3 + LiPo upgrade plan (`06d725f`)

The F3 crash-loop traced to motor VM riding on the same weak rail as
ESP32's own logic power (sourced from Jetson's current-limited USB
port). Also flags the Jetson's own 12V/1.5A supply as undersized
against its ~45W rated draw, relevant before Week 3 GPU workloads.
Records the planned LiPo + LTC3780 buck-boost shopping list for the
Jetson supply, separate from the still-pending VM rewire.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>

### 2026-07-22 00:05 — Add temporary PWM safety cap to unblock Kp tuning before VM rewire (`c78ebaf`)

MAX_SAFE_PWM=100 caps peak current draw so F3 can be tuned today
without repeating the brownout reset loop. Not a real fix — VM still
needs its own supply straight from the powerbank (see README). Raise
this cautiously once that's done, watching for resets.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>

### 2026-07-22 21:19 — Merge encoder+PID into one 20Hz control_task, add PWM slew-rate limit (`64d49be`)

Addresses code review feedback: separate encoder_task (1Hz) and
pid_task (100Hz) meant PID recomputed 100x/sec against an RPM value
up to 1s stale. Combining both into one loop removes that staleness
entirely. Also adds slew_limit() to cap how fast applied PWM can
change per cycle, reducing di/dt current transients on the still-weak
shared power rail (does not replace the pending VM rewire).

Trade-off: 20Hz sampling (50ms window) gives coarser RPM resolution
than the old 1Hz window (60 RPM/pulse-step vs. 3 RPM/pulse-step) —
accepted for faster feedback given the 20-slot encoder.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>

### 2026-07-23 21:06 — Document camera debugging finding + open question for next session (`10358b2`)

nvgstcapture-1.0 crashed at the H.264 encoder-linking stage, not the
camera itself; a minimal gst-launch-1.0 pipeline proved the IMX219
was working the whole time. Left as an open question to reason
through before writing Week 3 capture code.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>

### 2026-07-24 21:54 — Document VM power fix + add IMU I2C pin assignments (`a41792b`)

Motor VM now wires directly from the powerbank instead of passing
through the ESP32's 5V pin — confirmed on hardware (F3 ran through the
PWM ramp with no brownout/reset loop). Also records the MPU6050 I2C
pins (SDA -> GPIO26, SCL -> GPIO25), wired but not yet read in firmware.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>

### 2026-07-26 21:42 — Close out camera CSI debugging note: hardware confirmed working (`604d06d`)

nvgstcapture-1.0's H.264 encoder-linking stage was the actual fault,
not the IMX219/Argus camera. Locks in the isolation lesson and points
future capture code at gst-launch-1.0-style raw pipelines instead.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>

### 2026-07-26 21:42 — Add encoder stall-detection guard to control_task (`d6baa5b`)

Warns if PWM is clearly high enough to move a wheel but pulse count
stays 0 for 1s straight, so a dead/disconnected encoder shows up in
the log automatically instead of needing a human to notice RPM=0.0.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>

---

## Week 5 (2026-07-27 to 2026-08-02)

### 2026-07-27 13:47 — Add first CSI camera + YOLOv8n pretrained smoke test (`4abbcf3`)

Minimal script to validate the full pipeline (CSI capture via
GStreamer -> YOLOv8n pretrained inference -> draw boxes/labels/
confidence -> record to file) before any custom fine-tuning work.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>

### 2026-07-27 14:19 — Make first_test.py headless-safe (no cv2.imshow over SSH) (`561a381`)

Runs for a fixed duration (or Ctrl+C) and only writes the annotated
video to disk instead of live-displaying it, since the Jetson is run
over plain SSH with no X11 forwarding. Matches the actual validation
method being used: review the recorded video after the fact.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>

### 2026-07-27 15:24 — Lower confidence threshold to 0.1 for missed-detection debugging (`5cc7eb3`)

Testing whether a person sitting next to a correctly-detected chair
was truly missed by the model, or just filtered out by the 0.5
threshold.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>

### 2026-07-27 15:44 — Add dataset collection scripts: record raw video, then extract frames (`6b520cf`)

Solves the "can't trigger a photo while also moving the object"
problem - record continuously while moving the duck freely, extract
one frame every ~0.5s afterward, instead of timing individual shots.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>

### 2026-07-27 17:16 — Thin frame extraction interval to cut manual labeling volume (`f53fb7d`)

179 frames at 15-frame spacing was mostly near-duplicates for an easy
single-class task. Widening to 45 targets ~40-50 diverse frames instead.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>

### 2026-07-27 17:44 — Add fine-tune script for custom duck class (`c0388e2`)

Trains YOLOv8n from COCO-pretrained weights on the hand-labeled
duck_dataset (148 train / 16 val images) for 50 epochs.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>

### 2026-07-27 17:53 — Drop hardcoded absolute path from generated data.yaml (`2dbee28`)

The Windows-built path broke training on the Jetson. Ultralytics
resolves train/val relative to the yaml file's own location when no
"path:" key is given, which works regardless of which machine built
or trains the dataset.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>

### 2026-07-27 18:00 — Cut training to 25 epochs to fit CPU-only runtime on Jetson (`ba2d058`)

50 epochs at ~5-6 min/epoch on CPU-only PyTorch was ~4-5 hours;
halving it trades some margin for a manageable single-session runtime
on this simple, single-class task.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>

### 2026-07-27 18:46 — Add PWM floor to keep current draw above powerbank auto-shutoff (`b47ee54`)

The powerbank feeding VM has no-load auto-shutoff (meant for phone
charging), which was tripping whenever PID sagged PWM toward 0. That
power drop caused a brownout/erratic-RPM spiral (RPM readings up to
6540 - clearly ISR noise from the voltage glitch, not real pulses).
MIN_SAFE_PWM=40 is a starting guess, tune from live testing.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>

### 2026-07-27 19:52 — Document today's findings: VM powerbank issue, camera handling, repo layout (`bbd9073`)

- Power Architecture: new open issue where the VM-dedicated powerbank
  cuts off mid-run; documents why sharing the Jetson's own Anker PD
  bank was ruled out instead of attempted.
- Lessons Learned: camera/Jetson must stay stationary during dataset
  capture - handling it mid-recording breaks the CSI pipeline.
- Repository Structure: adds the new jetson/ tooling added today
  (object_detection, dataset_collection, training).

### 2026-07-27 20:05 — Point first_test.py at fine-tuned duck weights instead of COCO yolov8n (`d2daf51`)

Swaps in best.pt from train-4 (25-epoch duck fine-tune) via a path
relative to the script, and renames the output file so the prior
COCO smoke-test recording isn't overwritten.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>

### 2026-07-27 21:11 — Document first live-camera test of fine-tuned duck detector (`6207117`)

Real detections at 0.53-0.73 confidence, one false-positive TV box
at 0.2 - expected given the tiny/no-negative-examples training set.
Deferred fix (raise CONF_THRESHOLD) noted for later, not applied now.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>

### 2026-07-27 21:28 — Add encoder ISR debounce to filter motor-noise false pulses (`f9f6064`)

Hand-spun encoder gave clean 0-60 RPM; motor-running gave RPM spikes
up to 6300 at constant PWM, pointing to PWM-switching/brush EMI
coupling onto the signal line rather than a wiring fault. Rejects
edges under 3ms apart, far below the ~100ms real pulse interval at
TARGET_RPM=30 but above typical EMI ringing duration.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>

### 2026-07-27 21:31 — Widen encoder debounce window from 3ms to 15ms (`0d8ef93`)

3ms cut peak noise-inflated RPM from 6300 to 540 but right encoder
still noisy vs. left's clean 0/60 - PWM runs at 1kHz, so noise can
recur faster than a 3ms window rejects. 15ms still has ~3x margin
under the real pulse interval at TARGET_RPM=30 (~50ms).

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>

### 2026-07-27 21:35 — Lower MIN_SAFE_PWM floor from 40 to 20 (`47db24e`)

Right wheel's actual RPM sat well above TARGET_RPM=30 even at the
floor, so PID was floor-clamped and unable to slow it toward target.
Power confirmed stable since the recent hardware fix, so testing a
lower floor.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>

### 2026-07-27 21:38 — Widen control period to 200ms to fix RPM quantization limit cycle (`720a900`)

At the old 50ms window, 1 pulse = 60 RPM/step and TARGET_RPM=30 sat
exactly between two measurable levels - confirmed live as RPM
alternating 0/60 every cycle regardless of PWM, a pure measurement
resolution problem no gain tuning could fix. At 200ms, 1 pulse =
15 RPM/step, making 30 RPM (2 pulses) exactly representable.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>

### 2026-07-27 21:45 — Raise TARGET_RPM from 30 to 60 to avoid floor-clamping (`f3b806e`)

30 forced the right wheel's PID output to sit at MIN_SAFE_PWM (its
natural RPM at the floor already exceeded target) - no torque margin
for added hardware later. 60 sits inside the PWM range both wheels
already reach comfortably. Still a test value, not a calibrated
real-world speed (needs wheel diameter, deferred to F4 odometry).

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>

### 2026-07-27 21:46 — Document the F3 PID debugging chain: encoder noise, quantization, floor, target (`6f741e2`)

Four stacked issues found via live encoder data before any gain
tuning happened: ISR noise from motor EMI, RPM measurement
quantization colliding with the test target, a safety floor
fighting the controller, and a target value with no torque margin.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>

### 2026-07-29 12:16 — Add Ki (PI control) with anti-windup, deliberately skip Kd (`fc495d1`)

P-only data showed the right wheel settling ~15 RPM below
TARGET_RPM consistently - a textbook P-only steady-state error,
justifying Ki. Integral is clamped independently (anti-windup) to
prevent windup during saturation, e.g. today's left-wheel stall
where PWM sat at max with 0 RPM for 10+ seconds. Kd skipped: RPM is
still a coarse step signal, differentiating it would amplify jitter
rather than help - PI is the standard choice for velocity loops.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>

### 2026-07-29 12:16 — Document Ki addition and the unresolved left-encoder blip (`e8e6420`)

Completes today's F3 PID debugging chain entry with the PI
controller outcome and an open note on a one-off left-encoder
zero-RPM episode that wasn't fully root-caused.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>

### 2026-07-29 12:36 — Add wheel-sync trim to fix the confirmed right-drift (`a05c604`)

Live test showed the robot pulling right (would circle if left
running), even with per-wheel PI converging each wheel to its own
target RPM independently. Tracks cumulative pulses per wheel (never
reset) and trims each wheel's target RPM proportionally to the
distance gap between them, so a wheel that's fallen behind actually
catches up rather than just matching speed from now on. KSYNC=0.05,
trim clamped to +/-15 RPM. Scope note: assumes both wheels want the
same target (straight-line only) - will need gating once F5 adds
real /cmd_vel turning commands.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>

### 2026-07-29 12:45 — Lower MIN_SAFE_PWM floor from 20 to 10 as a diagnostic test (`d4f16c6`)

Right wheel stayed pinned at PWM=20 (RPM 150-195) during real ground
driving, defeating the sync-trim fix since PID couldn't push PWM any
lower. If RPM stays this high even at PWM=10, that rules out the
floor as the cause and points to a real mechanical/motor asymmetry
instead - watch for the powerbank cutting out at this lower floor.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>

### 2026-07-29 14:06 — Switch RPM measurement from pulse-counting to period-based (`4ff6d0a`)

With a working encoder connection, RPM was still jumping in fixed
15 RPM steps (0, 15, 30...) - counting whole pulses in a 200ms
window can only measure in units of 1 pulse/window. Fixed by timing
the interval between consecutive pulses instead (already had the
microsecond timestamps from the debounce logic) and computing RPM
from that - the standard period-measurement tachometer technique,
far finer resolution at low speed than pulse-counting. Added a
500ms staleness timeout so a stopped wheel reads 0 instead of the
last (increasingly stale) interval forever.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>

### 2026-07-29 14:19 — Fix build-breaking macro order bug (CONTROL_PERIOD_MS used before defined) (`4ac4812`)

pid_step() used CONTROL_PERIOD_MS for its dt calculation, but the
#define lived much further down the file (near control_task), after
the point of use - a real compile error in C, not just style.
Confirmed via a fresh build failure: 'CONTROL_PERIOD_MS' undeclared
(first use in this function). This means every build since the Ki
commit likely failed silently when re-flashing reused a stale old
binary from build/ instead of failing loudly - so recent live test
results (wheel sync, floor changes) may not reflect the code changes
they were meant to test. Moved the #define up near SLOTS_PER_REV,
before pid_step(). Re-verification of recent tests needed.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>

### 2026-07-29 14:56 — Add F4 odometry (x, y, theta) from encoder pulses (`5e4906b`)

Wheel diameter 6cm, wheelbase 10cm (measured). Each cycle: convert
this cycle's pulse counts to per-wheel distance, average = distance
moved by robot center, difference/wheelbase = heading change. Uses
midpoint integration (average of old and new heading) rather than
plain Euler, since a full turning cycle's distance attributed only
to the pre-turn heading would visibly bias x/y at 200ms/cycle.
Logged alongside existing RPM/PWM/sync fields for the F4 test
(push robot 1m, verify x =~ 1.0).

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>

### 2026-07-29 15:57 — Add F5: uros_task subscribing /cmd_vel, publishing /odom (`fb0dae1`)

Mirrors the proven esp32/microros_hello.c connect/retry pattern.
cmd_vel_callback latches linear.x/angular.z with a timestamp for the
watchdog (control_task treats a stale or absent command as 0, never
"keep driving blind"). Per-wheel target RPM now comes from /cmd_vel
via differential-drive inverse kinematics, replacing the hardcoded
TARGET_RPM. Wheel-sync trim is gated to angular.z ~= 0 (straight-line
only) so it stops fighting intentional turns - the gap flagged when
sync was first added. /odom publishes pose_x/y/theta (F4) as
position + quaternion, and per-cycle distance/dtheta as twist.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>

### 2026-07-29 16:02 — Document F3-F5 status, solder-joint root cause, and the stale-binary lesson (`827619a`)

Adds F3/F4/F5 sections with current status, and Lessons Learned
entries for: the bad solder joint that was the real cause of the
right-encoder weirdness (not floor/motor asymmetry), the period-
measurement RPM fix, and the macro-order compile bug that likely
invalidated some same-day test results via silent stale-binary
re-flashing.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>

### 2026-07-29 16:20 — Fix two F5 bugs found on first live test: PWM floor overriding stop, UART0 log/transport collision (`c37838f`)

1. MIN_SAFE_PWM was clamping output even when target_rpm==0, so the
   F5 safety watchdog's "stop on stale/missing /cmd_vel" never
   actually stopped the motors - confirmed live, robot drove ~2m
   unattended with no /cmd_vel ever sent. pid_step() now bypasses
   the floor entirely and returns 0 when target_rpm==0, also
   resetting the integral so a stop doesn't leave stale windup.

2. ESP_LOGI/ESP_LOGW and printf() calls share UART0 with the
   micro-ROS transport - confirmed live as garbled monitor output
   at the same moment the agent never saw a valid session-establish
   packet. Muted all console log output via esp_log_level_set once
   the transport is set, and removed the printf() calls in
   uros_task (including during the ping-agent phase, which was also
   corrupting the handshake).

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>

### 2026-07-29 16:51 — Bring in micro-ROS custom transport files for F5, hardcode UART0 pins (`aa2288c`)

Copied esp32_serial_transport.c/h + Kconfig.projbuild from the
component's example, added to main/CMakeLists.txt SRCS. Hardcoded
UART_TXD/RXD to GPIO1/3 (UART0 default pins) directly instead of via
Kconfig - the example's Kconfig.projbuild didn't declare those
options, and this project's pins are fixed/known, no need for
menuconfig-time configurability.

### 2026-07-29 17:01 — Fix F5 connect/disconnect loop: ping timeout was 0ms (`ab8f553`)

Confirmed live: agent successfully created the full session (client,
participant, subscriber, publisher, datareader, datawriter for
cmd_vel/odom), then tore it down ~1-2s later, repeating forever.
Cause: rmw_uros_ping_agent(0, 1) gives the response zero time to
arrive over the UART round-trip, so it reported failure almost every
cycle even with a healthy connection. Raised to 100ms.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>

### 2026-07-29 17:36 — Add reset-reason diagnostic topic to investigate mid-drive resets (`efc32eb`)

Odometry was observed resetting toward 0 repeatedly during a real
driving test, and the agent log showed the micro-ROS client_key
changing (new session) at the same time - evidence the ESP32 itself
reboots mid-drive, not just a connection drop. Since console logging
is muted (shares UART0 with the transport), there's no way to see a
reboot reason via idf.py monitor while /cmd_vel testing is active.
Reads esp_reset_reason() once at boot (before anything else runs)
and republishes it as a std_msgs/String on /esp32_diag over the
already-working micro-ROS link, so `ros2 topic echo /esp32_diag`
answers "was that a brownout?" without needing the serial console.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>

### 2026-07-29 17:54 — Document F5 build chain, confirmed cmd_vel->odom pipeline, open reset issue (`77c8458`)

Records the 6-step build debugging chain (dependency drift, missing
transport files, missing CMakeLists SRCS entry, missing Kconfig,
ping timeout) so a future setup doesn't rediscover each one blind.
Confirms the core F5 pipeline works live. Leaves an honest account
of the still-open mid-drive reset issue and what the next session
needs to do to diagnose it cleanly.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>

### 2026-07-31 14:27 — Extend /esp32_diag with live per-wheel RPM/PWM for hardware debugging (`e72ad17`)

idf.py monitor is unusable once uros_task's transport claims UART0 (all
console logging is muted to avoid corrupting the ROS serial stream), so
there was no way to see PID's live RPM/PWM readings during a /cmd_vel
test. /esp32_diag already existed for reset-reason diagnostics; reusing
it (rebuilt every publish cycle instead of once at connection time) to
also carry RPM L/R and PWM L/R avoids adding a whole new publisher.

### 2026-07-31 22:42 — Add duck detector training dataset, frames, and test outputs (`13e16cf`)

### 2026-08-01 20:48 — Document dual-camera CSI debugging: false port-defect conclusion, actual fix was a bad camera module (`8468d22`)

---

## Week 6 (2026-08-03 to 2026-08-09)

### 2026-08-04 14:03 — Add stereo pair capture script for camera calibration (`a610dff`)

### 2026-08-04 14:11 — Add stereo calibration script (`d4bb16e`)

### 2026-08-04 14:23 — Add fisheye-model stereo calibration to test pinhole-model baseline error (`3b7bd13`)

### 2026-08-04 14:31 — Add checkerboard corner detection visualizer for calibration sanity check (`3bfda44`)

### 2026-08-04 14:38 — Guide capture with a distance/angle shot plan, auto-wipe stale batches, warn against camera movement (`0ffa2f4`)

### 2026-08-04 14:54 — Fix dual-camera segfault: grab each camera sequentially instead of holding both sessions open concurrently (`6dbea63`)

### 2026-08-04 14:59 — Add settle delay between camera sessions to avoid Argus dmabuf race (`f01f799`)

### 2026-08-04 15:05 — Rewrite capture to shell out to gst-launch-1.0 per shot instead of holding cv2 Argus sessions in-process (`f6a285c`)

### 2026-08-04 15:06 — Fix shell syntax error: quote the caps string containing parentheses (`bd32666`)

### 2026-08-04 15:26 — Capture both cameras concurrently instead of sequentially, matching the already-proven-working dual-stream pattern (`f7a3be8`)

### 2026-08-04 15:35 — Check capture success by file validity not exit code; add small stagger between camera starts (`0cf2394`)

### 2026-08-04 19:31 — Add script to convert stereo calibration npz to ROS camera_info YAML files (`c3cf26a`)

### 2026-08-04 19:37 — Add combined launch file: 2x ArgusMonoNode + VisualSlamNode using real calibration for camera_info and inter-camera TF (`8a47bcc`)

### 2026-08-04 19:40 — Fix CALIB_PATH to point inside the mounted Isaac ROS workspace, not the host-only slam-amr checkout (`146cc68`)

### 2026-08-04 19:48 — Raise sync_matching_threshold_ms: independent (non-fsync'd) cameras need more slack than the 5ms Hawk-module default (`b20c032`)

### 2026-08-04 19:54 — Set camera mode=4 (1280x720) to match calibration resolution -- default mode 0 was full sensor res, rejected by visual_slam (`a9a828b`)

### 2026-08-04 21:31 — Raise image_jitter_threshold_ms to match actual ~9-10fps camera throughput (`70b2065`)

### 2026-08-04 21:53 — Document today's marathon session: CSI cable root cause for calibration error, Isaac ROS release-3.2 bring-up, first live cuVSLAM run with open pose-scale issue (`bc39e31`)

### 2026-08-04 22:28 — Fix stereo camera_info to encode baseline in right camera's P matrix (`92b3198`)

npz_to_camera_info_yaml.py hardcoded Tx=0.0 in projection_matrix for
both cameras, even though standard ROS stereo camera_info convention
requires the right camera's Tx = -fx * baseline. This zeroed-out
baseline is a candidate root cause for the vo_pose scale error seen
in cuVSLAM (open bug from 2026-08-04 session).

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>

### 2026-08-04 22:38 — Revert "Fix stereo camera_info to encode baseline in right camera's P matrix" (`1b5f8ab`)

This reverts commit 92b319851eaafafd014aa0948e20aa78ba085453.

### 2026-08-04 22:44 — Add rectified_images:=True SLAM pipeline variant to isolate the vo_pose scale bug (`605dea4`)

New files, existing raw-mode pipeline untouched:
- npz_to_camera_info_yaml_rectified.py: runs cv2.stereoRectify() to compute real
  R1/R2/P1/P2 (baseline correctly encoded in P2's Tx by OpenCV's own math, not
  hand-patched -- see the reverted PR #27 for why hand-patching Tx made things worse).
- visual_slam_argus_rectified.launch.py: adds an isaac_ros_image_proc RectifyNode per
  camera between ArgusMonoNode and VisualSlamNode, sets rectified_images:=True.

Deliberately keeps the same TF (from raw R/T) and same sync/jitter params as the raw-mode
launch file, so this is a single-variable comparison against the open pose-scale bug.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>

### 2026-08-05 22:50 — Update README with Week 2-4 progress and tonight's scale-bug investigation (`cd87685`)

- Roadmap: mark Week 2 done, Week 3 in-progress (pipeline runs, scale bug open),
  Week 4 done early (PID/cmd_vel landed alongside Week 2's F3/F5).
- Lessons Learned: document the Tx camera_info patch (tested, made things worse,
  reverted — PR #27/#28) and the rectified_images:=True comparison build (PR #29),
  which surfaced a deeper frozen-pose/NITROS-subscriber issue instead of answering
  the original scale question. Logs the next concrete test (known-distance move)
  and a fallback plan if it keeps dead-ending.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>

### 2026-08-06 21:44 — Document Isaac ROS env fix (missing isaac_ros_nitros, magic_enum CMake bug) and the vo_pose hand-motion-vs-wheel-motion investigation (`2ff5489`)

Environment: isaac_ros_nitros was never cloned into the workspace, and
building it hit a real upstream NVIDIA CMake bug (missing
find_package(magic_enum) in isaac_ros_gxf/gxf_isaac_messages) - both
fixed, pipeline runs clean again.

Investigation: ruled out TF baseline, camera_info Tx, and stereo sync
timing as the scale-bug cause via direct measurement. Found that any
hand contact with the rig reliably causes catastrophic, unrecoverable
tracking loss (not a code bug - an experimental-protocol hazard), while
genuinely hands-off /cmd_vel-driven motion stays bounded with a real
~3-5x apparent-vs-real scale discrepancy across two independent trials.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>

### 2026-08-07 03:28 — Add Nav2 closed-loop MVP (Week 3): controller/planner/behavior/bt_navigator on /odom (`df5bf2d`)

No map/AMCL (no map source yet, no lidar), no obstacle layer (Week 4 scope).
Includes odom_to_tf.py bridge node -- ESP32 firmware publishes /odom as a
topic only, not TF, which Nav2's costmaps/controller require directly.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>

### 2026-08-07 14:05 — Bring README up to date with Nav2 MVP and current motor power setup (`9621505`)

- Document jetson/nav2/ (previously not mentioned anywhere): scope,
  limitations, odom_to_tf.py TF-bridge gap, how to run/test.
- Fix repo structure tree: jetson/calibration/ and jetson/slam/ existed
  but were never listed; add jetson/nav2/ too.
- Fix ROS2 Topic Interface table and pipeline diagram, which described
  Nav2 as consuming /visual_slam/tracking/odometry + /map -- it actually
  runs on /odom (vo_pose scale bug is deferred, no lidar for AMCL anyway).
  Added a Status column instead of deleting the planned rows.
- Update Power Architecture / hardware / wiring tables: motor VM now runs
  off a dedicated Li-ion cell + boost converter, replacing the powerbank
  that had a mid-run cutoff bug (marks that issue resolved).
- Update Roadmap and Team & Work Split to reflect the Nav2 MVP landing
  ahead of its original Week 5 assignment, with a coordination note.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>

---
