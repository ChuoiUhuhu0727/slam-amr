# slam-amr

Autonomous Mobile Robot with Visual-Inertial SLAM on NVIDIA Jetson Orin Nano Super.

**Ngoc Giang — Fulbright University Vietnam — June–August 2026**

## Input / Output

**Input:** a goal position — `(x, y)` coordinate on a map, clicked in RViz2 on the Jetson

**Output:** the robot physically drives itself to that position and stops, correcting its path in real time

Everything in between (SLAM, Nav2, PID, odometry) is the pipeline making that happen automatically.

## Business Context

Factory logistics in Vietnam is still largely manual. Mid-size manufacturers (electronics, garments, F&B) move parts between stations by hand or with basic forklifts. Imported AMRs (MiR, Omron) cost $20,000–$50,000 per unit — out of reach for most.

This project demonstrates a camera-based AMR (no lidar) built on commodity hardware. Replacing lidar with a $20 camera module cuts hardware cost significantly while GPU-accelerated SLAM on the Jetson maintains navigation quality. The Week 6 semantic navigation capability — navigate to a *detected object*, not a hardcoded coordinate — is the feature that makes this commercially relevant: a robot that can find a labeled bin or pallet without pre-programming exact positions.

Target users: Vietnamese manufacturers, logistics companies, and robotics startups who need autonomous internal transport but cannot justify imported AMR pricing.

## Performance Targets

| Metric | Target |
|--------|--------|
| Localization drift | <5 cm per 5 m travel |
| Navigation success rate | ≥80% in mapped environment |
| SLAM update rate | ≥30 Hz |
| /cmd_vel → motor response | <20 ms |
| Motor control loop (ESP32) | 100 Hz |

## Hardware

| Component | Role |
|-----------|------|
| Jetson Orin Nano Super (67 TOPS, 8 GB) | SLAM inference, navigation planning |
| IMX219 CSI camera | Primary vision sensor |
| ESP32 | Real-time motor control, sensor bridge |
| MPU6050 IMU | Visual-inertial sensor fusion |
| TB6612FNG motor driver | Dual H-bridge PWM control |
| LM393 encoders x2 | Wheel velocity feedback |
| TT DC motors x2 | Differential drive |
| Powerbank 20000mAh | 5V 3A USB output — main power source |

### Power Architecture — Known Issue + Planned Fix (2026-07-22)

**Problem found during F3 (PID) testing:** motor power (VM on TB6612FNG) was wired through the ESP32's own 5V pin, which itself was powered off the Jetson's USB port. Jetson USB ports are current-limited and were never meant to supply motor-driver current. Once PID pushed PWM higher during startup (highest current draw is near-stall, i.e. near 0 RPM), the shared 5V rail sagged enough to brown out the ESP32, which then rebooted, re-enabled the motor immediately in `app_main`, sagged again — a self-sustaining reset loop.

Separately, the Jetson's own supply (powerbank → PD Trigger → 12V barrel jack) tops out at ~12V/1.5A (~18W), well under the ~45W (19V/2.37A) the stock adapter is rated for — fine at idle, but a real risk once Week 3 GPU/SLAM workloads start drawing more.

**Planned fix — dedicated LiPo + buck-boost for the Jetson, decoupling it from the ESP32/motor power path:**

| Item | Spec to match | Search terms |
|------|----------------|--------------|
| LiPo battery pack | 4S, 5000mAh, XT60 connector + balance connector | `4S 5000mAh XT60 lipo` |
| Balance charger | Supports 4S balance charging | `lipo balance charger 4S` |
| Buck-boost converter | Adjustable output, input range covering 3S–6S (~9–25V), ≥5A rated output — built around the `LTC3780` controller IC | `LTC3780 buck boost converter module 10A` |
| LiPo safety bag | Fireproof charging/storage bag | `lipo safe bag fireproof` |

**Before connecting to the Jetson:** power the buck-boost from the LiPo alone (Jetson disconnected), measure output with a multimeter, trim to exactly 19V, confirm polarity — only then connect to the barrel jack. Same verify-before-connect discipline as the camera cable fix.

**✅ Fixed 2026-07-24:** motor VM now has its own wire straight from a powerbank output, bypassing the ESP32 entirely. Confirmed on real hardware — F3 (PID) ran continuously through the PWM ramp with no brownout/reset loop. The Jetson's own LiPo/buck-boost supply (above) is still open — lower priority, becomes urgent before Week 3 GPU workloads.

**🔶 New issue found 2026-07-27, open:** the powerbank now dedicated to VM cuts its own output off mid-run during PID testing. Root cause not yet isolated (candidates: no-load auto-shutoff tripping when PWM sags low, or over-current protection tripping on motor inrush current when PID ramps PWM up quickly — these need different fixes, and which one it is wasn't confirmed before this session paused). A `MIN_SAFE_PWM` floor was added in `motor_f1.c` as a first attempt (ruling out the low-current theory) but did not resolve it. **Decided against sharing the Jetson's own Anker PD powerbank for VM too** — even a well-regulated multi-port bank shares one internal battery/BMS, so a motor inrush spike on one port risks sagging the other; this would reintroduce the exact ESP32-brownout failure mode from the 2026-07-22 bug above, but on the Jetson instead (much higher stakes). Plan: keep Jetson and motor VM on physically separate power sources (as today), find/verify a VM powerbank that doesn't exhibit this cutoff behavior under real motor load.

## Software Stack

| Layer | Technology |
|-------|-----------|
| OS | Ubuntu 22.04 (JetPack 6.2) |
| Middleware | ROS2 Humble |
| SLAM | Isaac ROS Visual SLAM (Elbrus, GPU-accelerated) |
| Navigation | Nav2 |
| MCU framework | ESP-IDF + FreeRTOS |
| MCU ROS bridge | micro-ROS for ESP-IDF |

## Motor Driver Schematic (TB6612FNG H-Bridge)

![](<images/H-Bridge in TB6612FNG.drawio.png>)

## Wiring Table (ESP32 ↔ TB6612FNG ↔ Motors ↔ Encoders ↔ IMU)

Motor driver path is wired and verified in `esp32/motor_f1` (F1 milestone). Encoders are wired (F2, in progress). IMU is wired but not read in firmware yet.

| ESP32 Pin | Connects To | Component | Purpose | Status |
|-----------|-------------|-----------|---------|--------|
| GPIO16 | PWMA | TB6612FNG | Motor A speed | ✅ wired |
| GPIO17 | PWMB | TB6612FNG | Motor B speed | ✅ wired |
| GPIO18 | AIN1 | TB6612FNG | Motor A direction | ✅ wired |
| GPIO19 | AIN2 | TB6612FNG | Motor A direction | ✅ wired |
| GPIO21 | BIN1 | TB6612FNG | Motor B direction | ✅ wired |
| GPIO22 | BIN2 | TB6612FNG | Motor B direction | ✅ wired |
| GPIO23 | STBY | TB6612FNG | Enable (HIGH = run) | ✅ wired |
| GPIO34 | OUT | LM393 Encoder L | Wheel L pulse count | ✅ wired |
| GPIO35 | OUT | LM393 Encoder R | Wheel R pulse count | ✅ wired |
| GPIO26 | SDA | MPU6050 IMU | I2C data line | 🔌 wired, not read in firmware yet |
| GPIO25 | SCL | MPU6050 IMU | I2C clock line | 🔌 wired, not read in firmware yet |
| 3V3 | VCC | TB6612FNG, both encoders, IMU | Logic power | ✅ wired |
| Powerbank (direct) | VM | TB6612FNG | Motor power | ✅ wired — direct from powerbank, no longer via ESP32 5V (fixed 2026-07-24) |
| GND | GND | All above + powerbank | Common ground | ✅ wired |

**Downstream of TB6612FNG:** AO1/AO2 → Motor L (red/black) · BO1/BO2 → Motor R (red/black)

## System Pipeline

```mermaid
flowchart TD
    A["RViz2 — click goal (x, y)"] --> B["Nav2\nPath Planner + Costmap"]
    C["IMX219 CSI Camera"] --> D["isaac_ros_visual_slam\nGPU-accelerated on Jetson"]
    D -->|"/visual_slam/tracking/odometry"| B
    D -->|"/map"| B
    B -->|"/cmd_vel\ngeometry_msgs/Twist"| F["micro-ROS Agent\nJetson"]
    F <-->|"UART 115200 baud"| G["ESP32 — FreeRTOS\n4 tasks"]
    G -->|"PWM + DIR"| H["TB6612FNG\nMotor Driver"]
    H --> I["TT Motor L + TT Motor R"]
    I --> J["LM393 Encoders x2"]
    J -->|"pulse count → RPM"| G
    G -->|"/odom nav_msgs/Odometry"| F
    G -->|"/imu sensor_msgs/Imu\nMPU6050 @ 200 Hz"| D
    F -->|"/odom + /imu"| B
```

## ESP32 Firmware (FreeRTOS)

4 tasks pinned to cores:
- `imu_task` — I2C MPU6050 @ 200 Hz → `/imu`
- `encoder_task` — GPIO ISR on LM393 → RPM per wheel
- `pid_task` — velocity PID @ 100 Hz → PWM + `/odom`
- `uros_task` — micro-ROS spin, subscribes `/cmd_vel`

### F1 — Basic Motor Spin (`esp32/motor_f1`)

First firmware milestone for the drivetrain: prove the **ESP32 → TB6612FNG → TT motor**
path works. Fixed 50% PWM, both motors forward. No encoder, no PID yet (those are F2 / F3).

**Pin map (ESP32 38-pin DevKit → TB6612FNG)**

| ESP32 | TB6612FNG | Purpose |
|-------|-----------|---------|
| GPIO16 | PWMA | Motor A speed |
| GPIO17 | PWMB | Motor B speed |
| GPIO18 | AIN1 | Motor A direction |
| GPIO19 | AIN2 | Motor A direction |
| GPIO21 | BIN1 | Motor B direction |
| GPIO22 | BIN2 | Motor B direction |
| GPIO23 | STBY | Enable (HIGH = run) |
| 3V3 | VCC | Logic power |
| GND | GND | Common ground (shared with powerbank) |

> Motor power (VM) no longer connects to the ESP32 — wired directly from the powerbank as of 2026-07-24. See [Power Architecture](#power-architecture--known-issue--planned-fix-2026-07-22) for the fix history.

Motor L: red → AO1, black → AO2 · Motor R: red → BO1, black → BO2

**Build & flash**

```bash
cd esp32/motor_f1
idf.py build
sudo chmod 666 /dev/ttyUSB0
python -m esptool --chip esp32 --no-stub -p /dev/ttyUSB0 -b 115200 \
  write_flash --flash_mode dio --flash_size 2MB --flash_freq 40m \
  0x1000  build/bootloader/bootloader.bin \
  0x8000  build/partition_table/partition-table.bin \
  0x10000 build/motor_f1.bin
```

**Gotchas hit during F1 (so we don't repeat them)**

1. **Non-standard crystal.** This board's XTAL is not the usual 40 MHz.
   A default build gave garbled serial + a boot loop. Fix is baked into
   `sdkconfig.defaults` (`CONFIG_XTAL_FREQ_AUTO=y`).
2. **Stub flashing fails** with `Failed to start stub`. Flash with
   `--no-stub` at `-b 115200` (see command above).
3. **Predict-then-measure debugging.** Every pin has an *expected* voltage
   you can work out before touching the meter. Two points on the same wire
   showing different voltages ⇒ a broken/cold solder joint between them.

### F2 — Encoder RPM Read (`esp32/motor_f1`, merged with F1)

Second firmware milestone: `encoder_task` counts LM393 pulses via GPIO ISR
and computes RPM per wheel every 1s. Merged into the same project as F1
(not a separate one) because F3 (PID) needs both motor and encoder together
anyway. **Confirmed working on real hardware 2026-07-21** — hand-spun each
wheel, RPM tracked correctly and independently per side, returned to 0 at rest.

**Pin map (ESP32 → LM393 encoders)**

| ESP32 | Encoder | Purpose |
|-------|---------|---------|
| GPIO34 | Left OUT | Pulse count (input-only pin, no internal pull-up needed) |
| GPIO35 | Right OUT | Pulse count (input-only pin, no internal pull-up needed) |

**Test procedure** — verify the encoder alone before trusting it under PID:
comment out the `gpio_set_level(STBY_PIN, 1)` line in `app_main` so the
motors stay off, flash, then hand-spin each wheel and check the RPM printed
over serial looks sane. Only once confirmed, uncomment and let F1 + F2 run
together.

**Build, flash & monitor** — pipe the monitor output to a timestamped file
so a milestone test run isn't lost to terminal scrollback:

```bash
cd esp32/motor_f1
idf.py build
sudo chmod 666 /dev/ttyUSB0
python -m esptool --chip esp32 --no-stub -p /dev/ttyUSB0 -b 115200 \
  write_flash --flash_mode dio --flash_size 2MB --flash_freq 40m \
  0x1000  build/bootloader/bootloader.bin \
  0x8000  build/partition_table/partition-table.bin \
  0x10000 build/motor_f1.bin
idf.py -p /dev/ttyUSB0 monitor | tee "test_f2_$(date +%Y%m%d_%H%M).log"
```

**Gotchas hit during F2 (so we don't repeat them)**

1. **Wrong FreeRTOS macro.** `portMUX_INITIALIZE_DEFAULT` doesn't exist —
   the correct static spinlock initializer is `portMUX_INITIALIZER_UNLOCKED`.
   Compiler catches this immediately, but noting it here to save the lookup.
2. **Read-then-reset race condition.** The shared pulse counter is written
   by the ISR and read+reset by `encoder_task` every second. Without
   protection, a pulse that arrives between the read and the reset gets
   silently lost (the reset unconditionally zeroes the counter, discarding
   whatever was there). Fixed by wrapping the read+reset in
   `portENTER_CRITICAL`/`portEXIT_CRITICAL`.
3. **RPM is quantized in steps of 3.** With 20 slots/rev and a 1-second
   sample window, one pulse = `(1/20) * 60 = 3` RPM — every value in the
   test log is a multiple of 3. This isn't a bug; it's the sensor's actual
   resolution. **Relevant for F3:** at low speed this granularity can look
   like noise/jitter to a PID loop — if Kp tuning looks unexpectedly twitchy
   at low RPM, check whether it's real oscillation or just this quantization
   before assuming the gains are wrong.

**TODO (still not done, still worth doing):** write a small Python script to
parse a saved monitor log and plot RPM/PWM/pose vs. time. F3 debugging today
confirmed the need (see "F3 PID debugging chain" and "F5" below) — reading
scrolling terminal numbers by eye was the actual bottleneck more than once.
Workaround used instead: `idf.py monitor`'s built-in `Ctrl+T Ctrl+L` log-to-file
(reliable over SSH, unlike piping through `tee` — see Lessons Learned), then
`scp` the file back for offline analysis. A real parser/plotter is still the
better fix, just not built yet.

### F3 — PID Velocity Control (`esp32/motor_f1`, merged with F1/F2)

**Status: done, 2026-07-27.** P+I control per wheel (Kd deliberately skipped —
see "F3 PID debugging chain" in Lessons Learned for why), anti-windup, a
cumulative-distance wheel-sync trim, and a real compile-breaking bug fix
(macro used before its `#define` — see Lessons Learned) all landed the same
day. Full blow-by-blow of the 4 stacked bugs found before any of this worked
is in Lessons Learned; short version: encoder ISR noise, RPM measurement
quantization (twice — once from the sampling window, once from pulse-counting
vs. period-measurement), a safety PWM floor fighting the controller, and a
target value with no torque margin. None of them were "the gains are wrong."

### F4 — Odometry (`esp32/motor_f1`)

**Status: done, 2026-07-27.** Computes `x`, `y`, `theta` from wheel pulses
each control cycle — per-wheel distance from pulse count × wheel
circumference, robot-center distance = average of the two wheels, heading
change = difference ÷ wheelbase. Uses midpoint integration (average of old
and new heading for the position update, not just the old heading) since a
turning robot's heading can shift enough within one 200ms cycle to visibly
bias x/y otherwise. Measured constants: wheel diameter 6cm, wheelbase 10cm.
Test procedure: push the robot ~1m in a straight line, confirm the logged
`x` reads close to 1.0 (and `y`/`theta` stay near 0).

### F5 — micro-ROS: `/cmd_vel` in, `/odom` out (`esp32/motor_f1`)

**Status: builds, connects, and the core `/cmd_vel` → drive → `/odom` pipeline is CONFIRMED working (2026-07-27) — but one real issue is still open (see below), don't call this fully done yet.**

`uros_task` mirrors the proven `esp32/microros_hello.c` connect/retry pattern
(Week 1): subscribes `geometry_msgs/Twist` on `/cmd_vel`, converts
`linear.x`/`angular.z` into per-wheel target RPM via differential-drive
inverse kinematics (replacing the old hardcoded `TARGET_RPM` test value),
and publishes `nav_msgs/Odometry` on `/odom` at 10Hz using F4's pose. Two
safety/correctness details worth knowing:

- **Watchdog:** if no `/cmd_vel` message has arrived within 1s (agent crash,
  USB unplugged, Jetson down), commanded velocity is forced to 0 — a lost
  connection must mean "stop", never "keep driving on the last command."
  A related real bug (`MIN_SAFE_PWM` overriding this and never letting the
  robot actually stop) was found and fixed live — see Lessons Learned.
- **Wheel-sync gating:** the F3 sync-trim logic (see above) is now gated to
  only run when `|angular.z| < 0.05` (straight-line motion) — an intentional
  turn deliberately makes the two wheels travel different distances, and
  without this gate sync would fight every turn command.

**Known interface conflict:** micro-ROS's custom transport and the normal
debug console (`idf.py monitor` / `ESP_LOGI`) both use UART0 — once
`uros_task`'s transport is active, the familiar `RPM L=... R=...` log lines
stop being readable there (confirmed live: sharing the wire actively
corrupts both streams, not just makes one unreadable). All console logging
is muted via `esp_log_level_set` once the transport is set. Debugging F5 is
via the Jetson side instead: `ros2 topic echo /odom`, `ros2 topic pub
/cmd_vel`, and a new `/esp32_diag` topic (see open issue below) — per the
ROS2 Topic Interface contract further down.

**Confirmed working end-to-end (2026-07-27):** `ros2 topic pub /cmd_vel` from
the Jetson made the robot actually drive, and `ros2 topic echo /odom` showed
real position/heading tracking the motion (e.g. x=0.097m, y=0.189m, ~38°
turn after a short drive). This is the core F5 deliverable and it works.

**OPEN ISSUE, not yet resolved — resume here next session:** during a
longer, real drive test (both wheels actually running, not just the earlier
stationary connectivity check), `/odom`'s position was observed climbing
then resetting back toward ~0 repeatedly, and the agent log showed the
micro-ROS `client_key` change (i.e. a new session) at least once during
testing — strong evidence the ESP32 itself is rebooting mid-drive, not just
dropping the micro-ROS link. A `/esp32_diag` topic was added (publishes
`esp_reset_reason()` read at boot, since normal console logging is muted)
specifically to answer "was that a brownout?" without needing the serial
monitor - **but the diagnostic test to actually check this hasn't been
completed cleanly yet** (two different physical disturbances - an
accidental cable unplug, and closing the agent terminal by mistake -
interrupted data collection both times). Also seen once: one wheel
spinning while the other stayed stopped, then swapping - possibly related,
possibly a separate symptom of the same underlying power issue.
**Next session: rerun the drive test with `/esp32_diag` open in a terminal
the whole time, nobody touching any cables, and watch whether it flips to
`BROWNOUT` at the same moment `/odom` resets.** If confirmed, the real fix
is almost certainly a power-delivery problem under combined 2-motor current
draw (see "Power Architecture" section above for the project's known power
topology) - not a firmware bug to keep patching blind.

**Setup needed to build this project fresh on a new checkout** (already done
on the current Jetson working copy, but not automatic - the micro-ROS
library itself is a large local build cache, intentionally NOT committed to
git): copy `micro_ros_espidf_component` from an already-working micro-ROS
project (e.g. `~/esp/microros_hello/components/`) into
`esp32/motor_f1/components/`, then `idf.py build`. Everything this project
specifically needs on top of that (the custom transport `.c`/`.h`, a minimal
`Kconfig.projbuild`, `main/CMakeLists.txt`'s `SRCS` list) is already
committed - see Lessons Learned for the exact chain of build errors this
took to work out, in case a future fresh setup hits the same ones.

## Roadmap

| Week | Deliverable |
|------|-------------|
| 1 ✅ | micro-ROS hello world — ESP32 publishes ROS2 topic on Jetson |
| 2 | Motor driver + encoder wiring, ESP32 publishes `/odom` |
| 3 | IMX219 → isaac_ros_visual_slam → trajectory in RViz2 |
| 4 | PID velocity control, `/cmd_vel` → accurate robot movement |
| 5 | Nav2 + full end-to-end: camera → SLAM → Nav2 → motors autonomous |
| 6 | Semantic navigation: TensorRT object detection → navigate to target |
| 7 | Stress test, metrics, GitHub, demo video |
| 8 | Buffer / stretch goals (waypoint patrol, return-to-dock, multi-session map) |

## ROS2 Topic Interface (contract between ESP32 stack and Jetson stack)

This is the boundary the two halves of the team build against. Either side can develop independently as long as message type and topic name match — the ESP32 side doesn't need to know how `/cmd_vel` was computed, and the Jetson side doesn't need to know how `/odom` was computed.

| Topic | Message Type | Publisher | Subscriber |
|-------|-------------|-----------|------------|
| `/camera/image_raw` | `sensor_msgs/Image` | argus_camera (Jetson) | visual_slam (Jetson) |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | argus_camera (Jetson) | visual_slam (Jetson) |
| `/imu` | `sensor_msgs/Imu` | ESP32 (micro-ROS) | visual_slam (Jetson) |
| `/odom` | `nav_msgs/Odometry` | ESP32 (micro-ROS) | Nav2 (Jetson) |
| `/visual_slam/tracking/odometry` | `nav_msgs/Odometry` | visual_slam (Jetson) | Nav2 (Jetson) |
| `/map` | `nav_msgs/OccupancyGrid` | visual_slam (Jetson) | Nav2 costmap (Jetson) |
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 (Jetson) | ESP32 (micro-ROS) |
| `/tf` | `tf2_msgs/TFMessage` | visual_slam + Nav2 (Jetson) | All nodes |
| `/goal_pose` | `geometry_msgs/PoseStamped` | RViz2 / mission (Jetson) | Nav2 (Jetson) |

## Team & Work Split

Two-person team. Split is drawn along one line: **does this task require physically touching the robot?** Alex (remote) cannot solder, reseat a cable, or hear a motor to tune PID — so anything requiring hands-on-hardware iteration stays with the on-site owner. Anything that is pure software/config, or can be developed and dry-run against logged/simulated data (e.g. a `ros2 bag` recording, or mock topic publishers), is fair game to build remotely and integrate later.

**Ngoc Giang (vịt) — on-site, owns the physical stack:**
- ESP32 firmware requiring real hardware feedback: `encoder_task`, `pid_task` (PID tuning needs to hear/see the real motor respond — cannot be tuned blind), odometry math, `uros_task`
- All hardware bring-up: soldering, wiring, camera mounting/calibration, IMU mounting
- On-device validation: carrying the robot to check SLAM trajectory (Week 3), physically measuring drift (Week 7)

**Alex (remote) — owns the software/config stack, buildable without the physical robot:**
- Isaac ROS Docker setup + `visual_slam` launch/config (Week 3) — can be built and dry-run against a sample rosbag or public IMX219 dataset before the real camera feed is ready
- Nav2 stack: YAML config, planner selection, costmap layers (Week 5) — develop against simulated `/odom` + `/map` data, tune for real once camera/SLAM (vịt's side) is live
- Semantic navigation: train/export detection model, write TensorRT inference node (Week 6) — training and most integration work doesn't need the physical robot, only final on-device deployment does
- Tooling: evaluation/metrics scripts (Week 7), RViz2 dashboard config, repo docs

**Handoff points:** the [ROS2 Topic Interface](#ros2-topic-interface-contract-between-esp32-stack-and-jetson-stack) table above is the contract — build against topic name + message type, not against the other person's implementation. When a physical milestone lands (e.g. `/odom` is real and flowing), that's the signal to switch from mock data to live integration testing together.

## Repository Structure

```
slam-amr/
├── esp32/
│   ├── microros_hello/
│   │   └── microros_hello.c        # Week 1: micro-ROS publisher with auto-reconnect
│   └── motor_f1/
│       └── main/motor_f1.c         # F1-F5: motor spin, encoder RPM, PI control, odometry, micro-ROS
├── jetson/
│   ├── object_detection/           # first_test.py: CSI cam -> YOLO -> boxes -> record
│   ├── dataset_collection/         # record video, extract frames, assemble YOLO dataset
│   └── training/                   # fine-tune YOLOv8n on a custom class (train_duck.py)
└── README.md
```

## PID Control Block Diagram

![](<images/PID pipeline.drawio.png>)

## Lessons Learned

**Camera debugging (2026-07-23, closed out 2026-07-26) — CSI/IMX219 confirmed working, not abandoned.**

`nvgstcapture-1.0` crashed (`Elements could not link encoder & parser`, core dump) before capturing anything. Teammate's fix: skip that tool entirely and drive `gst-launch-1.0` directly — `nvarguscamerasrc ! nvvidconv ! autovideosink` connected fine (`CONSUMER: Producer has connected`), and a `nvjpegenc` still-capture pipeline completed cleanly. So the IMX219/Argus camera itself was never broken — the crash was isolated to `nvgstcapture-1.0`'s H.264 encoder-linking step, a stage the SLAM pipeline doesn't even need (OpenCV/visual_slam wants raw frames, not encoded video).

**Lesson locked in:** when a tool throws an error, identify which pipeline stage it's actually about before assuming the loudest/earliest error names the root cause. A minimal, explicit `gst-launch-1.0` pipeline isolated the fault to one unnecessary stage instead of condemning the whole camera. Apply the same isolation habit to the next weird failure anywhere in the stack (capture / SLAM / serial / servo). For Week 2/3 capture code: use `gst-launch-1.0`-style raw pipelines (or OpenCV's GStreamer backend with an equivalent pipeline string) — never `nvgstcapture-1.0`.

**Dataset collection (2026-07-27) — keep the camera stationary, move the object instead.** While recording a custom-object dataset (`jetson/dataset_collection/record_video.py`), physically picking up and orbiting the Jetson around the target object caused capture to fail mid-recording (`GStreamer warning: ... nvarguscamerasrc0 reported: INVALID_SETTINGS`). Isolated with a clean before/after test: Jetson stationary + object moved worked reliably every time; Jetson handheld failed. Leading theory is the CSI ribbon cable flexing under handling (consistent with this camera's known physical fragility). Rule going forward: camera/Jetson stays fixed in place for any capture session; only the target object moves — this achieves the same viewpoint diversity without touching the fragile part of the setup.

**First live-camera test of the fine-tuned duck detector (2026-07-27) — works, one known false positive.** `jetson/object_detection/first_test.py` was pointed at `train-4/weights/best.pt` (25-epoch fine-tune, single "duck" class) instead of stock `yolov8n.pt`. Result: real duck detections scored 0.53–0.73 depending on camera angle; one frame also boxed a TV in the background at 0.2 confidence. Since the model only has one class, that box can only be a "duck" mislabel of the TV, not a different class leaking through — a false positive, not a wiring/weights-loading bug. Root cause is the expected one for a 148-image training set with almost no negative (no-duck) examples: the model hasn't learned much about what *isn't* a duck yet. Not fixed yet, deliberately deferred (current results judged "good enough" to move on) — the concrete fix when revisited is raising `CONF_THRESHOLD` in `first_test.py` (currently `0.1`, left over from low-confidence COCO debugging) to ~0.4–0.5 to filter this class of low-confidence noise, and/or adding negative examples to the training set if it recurs.

**F3 PID debugging chain (2026-07-27) — four stacked bugs found via live encoder data, in the order they were found:**

1. **Encoder pulse noise while the motor runs.** Hand-spinning the right wheel gave a clean 0–60 RPM signal, but running it under PWM gave RPM readings spiking as high as 6300 at a *constant* PWM — a motor can't change speed like that on its own. Root cause: PWM switching (1kHz) and DC-brush arcing coupling electrical noise onto the encoder signal line, which the ISR was counting as real slot transitions. Fixed in software with a time-based debounce in the ISR (`esp_timer_get_time()`, reject edges under some minimum interval) — 3ms wasn't enough (PWM period is 1ms, noise can recur faster than that), widened to 15ms, which is still >>3x below the real pulse interval at low target RPMs.
2. **RPM quantization limit cycle.** Even with clean pulses, RPM was only measurable in steps of 60 RPM/pulse (`SLOTS_PER_REV=20` over a 50ms control window), and the test target (30 RPM) sat exactly between two measurable levels — the controller could never read "at target," only ever ±30 RPM error, a pure resolution problem no amount of Kp/Ki tuning could remove. Fixed by widening `CONTROL_PERIOD_MS` from 50ms (20Hz) to 200ms (5Hz), giving 15 RPM/pulse resolution — trades loop speed for resolution, acceptable at these low target speeds.
3. **`MIN_SAFE_PWM` floor fighting the controller.** The right wheel's actual RPM at the PWM floor (40) already exceeded the 30 RPM target, so PID could never reduce PWM further — permanently floor-clamped regardless of gain. Confirmed power stays stable at a lower floor (20) since the earlier VM-wiring fix, so lowered the floor to unstick it.
4. **The target itself had no torque margin.** Even after fixes 1–3, hitting 30 RPM meant running both wheels at very low PWM (near the floor) — fine today, but with zero headroom for added weight later (sensors, battery, final chassis). `TARGET_RPM` was a placeholder value anyway (real `/cmd_vel` targets come in F5), so raised it to 60 — inside the PWM range (~20–40) both wheels already handle comfortably, without redefining what "real" robot speed should be (that calibration needs wheel diameter + a chosen cm/s target, correctly deferred to F4 odometry work, not invented now).

**Lesson locked in:** each of these looked like "the PID gains are wrong" at first glance, and none of them were — they were an encoder signal-integrity bug, a measurement-resolution bug, a safety-clamp side effect, and a target-selection question, found in that order only by looking at the raw live data (`idf.py monitor`) instead of jumping to gain-tuning. Don't tune Kp/Ki/Kd against data before ruling out these categories of bug first.

**F3 completed same day — Ki added (PI control), Kd deliberately skipped.** With bugs 1–4 above fixed, clean P-only data showed the right wheel settling consistently ~15 RPM below `TARGET_RPM` (30–60 RPM band vs. target 60) — a textbook P-only steady-state error, and trustworthy this time since RPM resolution is now 15 RPM/step. Added `KI=0.2` (same "start small" approach as Kp) with an anti-windup clamp on the integral term specifically — without it, the integral would have grown unbounded during the left-wheel stall seen the same session (PWM pinned at max, RPM=0, for 10+ seconds), then caused a large overshoot once the wheel freed up. Kd was deliberately left out: RPM is still a coarse step signal at 5Hz, and differentiating a stair-step signal amplifies jitter rather than smoothing anything ("derivative kick") — PI is the standard choice for velocity/RPM loops, Kd matters more for position control. Revisit only if PI alone proves insufficient once retested.

One unresolved side note from the same session, deliberately not chased further: the left encoder briefly reported a sustained 0 RPM (firmware's own stall-detection log fired) while the wheel was visually confirmed still spinning by hand — encoder LED checked immediately after and looked normal. Left as an open possibility of an intermittent connection (not re-confirmed) rather than a proven fault; revisit only if it recurs.

**Root cause of the right-encoder weirdness, found later the same day: a bad solder joint — not floor tuning, not motor asymmetry.** Chased the "right wheel runs away even at the lowest safe PWM" symptom through two rounds of floor-lowering (40→20→10) with no improvement, plus a wheel-sync trim that made the circling *worse*, before the actual signal came from a direct contradiction: the encoder reported RPM 90–210 while the wheel was visually confirmed barely turning. That mismatch (sensor says fast, eyes say slow/weak) is what pointed at a bad connection rather than a real mechanical/control problem — re-soldering fixed it, confirmed by both wheels running evenly afterward. **Lesson:** when a sensor reading and a direct physical observation disagree, trust the physical observation and go looking for a wiring fault — don't keep re-tuning software parameters (floor, gains, sync) against a signal that might be lying. This project has now hit the "loose connector under vibration" failure mode three separate times (CSI cable, encoder VCC, this one) — always the first hypothesis to test when something behaves fine at rest/lifted but not under real load/motion.

**RPM quantization, round 2: pulse-counting vs. period measurement.** After the solder fix, RPM was still visibly jumping in fixed 15 RPM steps (0, 15, 30, 45...) with no in-between values — counting whole pulses in a fixed 200ms window can only ever resolve `1 pulse/window` = 15 RPM, so anything between steps is invisible to that method. Fixed by timing the interval *between* consecutive pulses instead (reusing the microsecond timestamps already captured for ISR debouncing) and computing RPM from that — the standard "period measurement" tachometer technique, far finer resolution at low speed than "frequency measurement" (pulse-counting). Added a 500ms staleness timeout so a stopped wheel correctly reads 0 instead of reporting the last (increasingly stale) interval forever. `CONTROL_PERIOD_MS` (still 200ms) now only paces the control loop, not RPM resolution — could be sped back up independently if ever needed.

**Real bug, not style: a `#define` used before it was defined, silently invalidated a chunk of same-day testing.** `pid_step()` used `CONTROL_PERIOD_MS` for its integral `dt`, but that constant's `#define` lived much further down the file (near `control_task`) — a genuine C compile error (`'CONTROL_PERIOD_MS' undeclared`), not just an ordering nitpick. Found only when a teammate (Alex, working the physical setup for the first time) hit the error directly and pasted the *actual compiler output* rather than the build-invocation noise around it. Because the flash workflow builds with `idf.py build` then flashes `build/*.bin` as a separate manual step (the `--no-stub` esptool workaround for this board's crystal quirk), a failed build doesn't block the flash step from re-uploading whatever `.bin` was already sitting in `build/` from the last *successful* build — so several "live test" results after the Ki commit (wheel-sync trim, floor 20→10) may have silently been re-running old, pre-Ki/pre-sync firmware rather than the code being discussed. **Lesson:** after any `idf.py build`, actually check it printed success before trusting the next flash+test cycle — a stale binary fails silently and looks exactly like "the code change had no effect," which cost real debugging time here. Fixed by moving the `#define` up near `SLOTS_PER_REV`, before any function that uses it.

**F5 build chain (2026-07-27) — six real, separate build/runtime errors in a row before it worked, each a different layer.** Worth recording the full sequence so a future fresh setup (or another teammate) doesn't have to rediscover each one from scratch:

1. **Deleting and re-cloning the micro-ROS library cache exposed upstream dependency drift.** First fix attempt was a full clean rebuild (`rm -rf micro_ros_dev micro_ros_src`) after an unrelated bug turned out to be in our own C code, not the build - the clean rebuild was unnecessary and cost real time. The component's build re-`git clone`s many ROS2 core packages fresh from their default branches (not pinned commits), so a clean rebuild is not reproducible - it can pull whatever changed upstream since the last successful build. Hit a `ModuleNotFoundError: No module named 'importlib.resources.abc'` (a freshly-cloned `ament_package` assuming Python 3.11+, this venv is 3.10) - patched with a `try/except` import fallback - then immediately hit a *different* error (`ament_cmake` package config not found) one layer deeper. **Recovery, not another patch:** copied the already-built `micro_ros_dev`/`micro_ros_src` directories wholesale from the known-working `~/esp/microros_hello` project instead of re-cloning - sidesteps the whole moving-target problem. **Lesson: don't delete a working micro-ROS build cache "to be safe" unless the bug is actually inside it** - confirm that first, because rebuilding from scratch is not a reliable, repeatable operation with this component.
2. **Missing `esp32_serial_transport.c`/`.h`.** `motor_f1.c` included the header but nothing had copied the actual transport source into this project - it exists only inside the component's `examples/int32_publisher_custom_transport/main/` folder, not on the main include path by default. Copied both files into `motor_f1/main/`.
3. **New source file silently not compiled.** Copying a `.c` file into `main/` isn't enough - this project's `main/CMakeLists.txt` lists `SRCS` explicitly (`"motor_f1.c"` only), so the new file linked with `undefined reference` errors until added to that list by name.
4. **Missing `Kconfig.projbuild`.** The transport code reads `CONFIG_MICROROS_UART_TXD/RXD/RTS/CTS`, Kconfig options declared in a `Kconfig.projbuild` that also needed copying from the example - except the specific example folder used didn't declare those options at all (only app-stack/priority settings), a dead end.
5. **Resolved by hardcoding instead of chasing Kconfig.** Given the project's UART0 pins are fixed and already known (TXD=GPIO1, RXD=GPIO3, no flow control), replaced the four `CONFIG_MICROROS_UART_*` macro references with hardcoded values directly in `esp32_serial_transport.c` - simpler and more correct than hunting for the right Kconfig source for a board that was never going to need runtime-configurable pins anyway.
6. **`rmw_uros_ping_agent(0, 1)` - a zero-millisecond timeout.** After all of the above finally produced a working build+flash, the micro-ROS agent showed full session setup (participant/subscriber/publisher/datawriter all created) followed by an immediate teardown, repeating every ~2s. A 0ms ping timeout gives the response no time to arrive over the UART round-trip, so it read as agent-lost almost every cycle even with a healthy connection. Raised to 100ms - session then stayed established, confirmed no drops over 60+ seconds idle.

None of these six were logic bugs in the robot-control code itself (PID/odometry/kinematics all built correctly the first time) - they were entirely in getting an unfamiliar, fragile third-party build system to compile at all. **Lesson for next time a new component gets added to this project: budget real time for build-system archaeology separately from the actual feature logic**, and prefer copying a known-working reference setup wholesale over re-deriving one from partial documentation.

**F5 core pipeline (`/cmd_vel` → drive → `/odom`) confirmed working live same day** - see F5 section above. One correctness bug found in the same test: `MIN_SAFE_PWM` (the anti-brownout PWM floor from F3) was overriding the F5 safety watchdog's "stop on stale `/cmd_vel`" - the robot never actually reached PWM=0 even when commanded to stop, and drove ~2m unattended with no command ever sent. Fixed by making `pid_step()` bypass the floor entirely and return exactly 0 when `target_rpm==0`.

**Open issue, not resolved same day:** during sustained two-motor driving, `/odom` position was seen climbing then resetting toward 0 repeatedly, and the agent log showed the micro-ROS session's `client_key` change at least once mid-test - strong circumstantial evidence of the ESP32 rebooting under load, not just a dropped link. Added a `/esp32_diag` topic publishing `esp_reset_reason()` (read at boot) specifically to test the leading hypothesis (brownout under combined 2-motor current draw) without needing the serial console, which is unusable during active `/cmd_vel` testing (see UART0 conflict note above). **The actual diagnostic run wasn't completed cleanly** - two unrelated physical disturbances (an accidental cable unplug, and the agent terminal being closed by mistake) interrupted data collection both attempts. Picking this up cleanly (nobody touching cables, `/esp32_diag` open the whole time) is the top item for the next session.

**Dual-camera CSI bring-up (2026-07-31) — long detour into device-tree internals, root cause turned out to be a defective camera module, not the board.** With the 2nd IMX219 mounted (~8.3cm baseline), enabling `jetson-io.py`'s "Camera IMX219 Dual" overlay showed one camera (`imx219 9-0010`) consistently failing `imx219_board_setup: error during i2c read probe (-121)` while the other (`imx219 10-0010`) always bound fine - 100% reproducible across many reboots.

Ruled out, with direct evidence, in order: power (multimeter confirmed 3.3-3.4V steady-state), boot-time timing (identical failure across independent reboots, and forcing a manual re-probe ~4 minutes post-boot via sysfs `unbind`/`bind` still failed the same way), `reset-gpios` polarity (decompiled the `.dtbo`, flipped the flag, recompiled, reflashed - zero change), `mux-gpios` polarity (flipping this one broke *both* cameras, confirming the GPIO is real but not a simple invert - reverted), and a real bug some other Orin Nano Super users hit where the bootloader silently ignores the `OVERLAYS` directive (worked around by merging the overlay into the base `.dtb` via `fdtoverlay` and pointing `FDT` at the merged file directly - identical result, ruling that bug out for this board specifically since `OVERLAYS` was already being applied correctly here).

Two isolation tests - swapping which camera module sat in which port, and separately swapping which ribbon cable connected to which port - both showed the failure following the *port*, not either module or cable. That looked like conclusive proof of a defective board-side CSI connector, and was written up and posted to the NVIDIA Developer Forums as such.

**It was wrong.** Buying one replacement IMX219 module (identical Waveshare SKU) and swapping it in fixed both cameras immediately - both bound cleanly in `dmesg`, both captured valid frames, both showed live video. The 2-way swap test couldn't actually distinguish "the port is bad" from "both of the original modules happen to share the same defect" - it only rules out one side when there's a genuinely known-good third unit in the rotation. **Lesson locked in: a swap test between only two known units is weaker evidence than it feels like in the moment - it can't rule out "both units are bad" the way it can rule out a port/cable fault.** Don't post a board-defect conclusion (internally or publicly) off a 2-way swap alone; get a third known-good unit into the test first if at all possible.

Also worth remembering: mid-session, running a second AI assistant against the same live Jetson (uncoordinated with the first) to try the same class of fix produced a *new*, unrelated regression (`dmesg` went completely silent on `imx219` - worse than either prior state) purely from two tools editing the same boot config concurrently. Untangled via `history` and `md5sum` against known-good backups. Don't run two AI-assisted debugging sessions against the same live system state at once without one of them being the clear source of truth.
