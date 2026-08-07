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
| Powerbank 20000mAh (PD) | Jetson power via PD Trigger → 12V barrel jack — see [Power Architecture](#power-architecture--known-issue--planned-fix-2026-07-22), LiPo+buck-boost upgrade still pending |
| 1x 3.7V Li-ion cell + boost converter | Motor VM (TB6612FNG) — replaced the dedicated powerbank after its mid-run cutoff bug, confirmed running cleanly under real PID load. **TODO: record cell capacity + boost converter model/rating here.** |

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

**🔶 Issue found 2026-07-27, RESOLVED by switching power source (not by isolating the original root cause):** the powerbank then dedicated to VM cut its own output off mid-run during PID testing. Root cause was never conclusively isolated (candidates: no-load auto-shutoff tripping when PWM sags low, or over-current protection tripping on motor inrush current — a `MIN_SAFE_PWM` floor added as a first attempt ruled out the low-current theory but didn't resolve it). **Decided against sharing the Jetson's own Anker PD powerbank for VM too** — even a well-regulated multi-port bank shares one internal battery/BMS, so a motor inrush spike on one port risks sagging the other, reintroducing the exact ESP32-brownout failure mode from the 2026-07-22 bug but on the Jetson instead (much higher stakes).

**✅ Current fix:** motor VM now runs off a dedicated **1x 3.7V Li-ion cell + boost converter** instead of any powerbank. Confirmed running cleanly under real motor/PID load, no mid-run cutoff — Jetson and motor VM remain on physically separate power sources, as planned. The Jetson's own supply (powerbank → PD Trigger → barrel jack) is unchanged and still the lower-priority open item below.

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
| Li-ion cell (3.7V) → boost converter (direct) | VM | TB6612FNG | Motor power | ✅ wired — no longer via ESP32 5V (fixed 2026-07-24), no longer a powerbank either (switched off the cutoff-prone VM powerbank, see Power Architecture) |
| GND | GND | All above + powerbank | Common ground | ✅ wired |

**Downstream of TB6612FNG:** AO1/AO2 → Motor L (red/black) · BO1/BO2 → Motor R (red/black)

## System Pipeline

```mermaid
flowchart TD
    A["RViz2 — click goal (x, y)\nFixed Frame: odom"] --> B["Nav2\nPath Planner + Costmap\n(global_frame: odom, no map)"]
    C["IMX219 CSI Camera"] --> D["isaac_ros_visual_slam\nGPU-accelerated on Jetson"]
    D -.->|"/visual_slam/tracking/odometry\n(planned, blocked on vo_pose bug)"| B
    D -.->|"/map\n(planned, no mapping pipeline yet)"| B
    B -->|"/cmd_vel\ngeometry_msgs/Twist"| F["micro-ROS Agent\nJetson"]
    F <-->|"UART 115200 baud"| G["ESP32 — FreeRTOS\n4 tasks"]
    G -->|"PWM + DIR"| H["TB6612FNG\nMotor Driver"]
    H --> I["TT Motor L + TT Motor R"]
    I --> J["LM393 Encoders x2"]
    J -->|"pulse count → RPM"| G
    G -->|"/odom nav_msgs/Odometry"| F
    G -->|"/imu sensor_msgs/Imu\nMPU6050 @ 200 Hz"| D
    F -->|"/odom"| K["odom_to_tf.py\n(jetson/nav2/)"]
    K -->|"TF odom→base_link"| B
    F -->|"/odom"| B
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
| GND | GND | Common ground (shared with motor power source) |

> Motor power (VM) no longer connects to the ESP32 — wired directly to its own supply as of 2026-07-24, now a dedicated Li-ion cell + boost converter (see [Power Architecture](#power-architecture--known-issue--planned-fix-2026-07-22) for the full fix history, including the powerbank that was tried and replaced in between).

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

## Nav2 — Closed-Loop MVP (`jetson/nav2/`)

**Status: code written and merged (PR #32), not yet tested end-to-end on real hardware.** Goal: click a goal pose in RViz2, robot drives there via `/cmd_vel`. Runs **natively on the Jetson's ROS2 Humble** (`ros-humble-navigation2`), fully outside the Isaac ROS Docker container — Nav2 only depends on `/odom`, which is already published natively by the ESP32/micro-ROS side, so this whole stage is decoupled from the fragile Isaac ROS environment.

**Deliberate scope decisions for this MVP (see also the ROS2 Topic Interface status column above):**
- **`global_frame: odom`, no map, no AMCL.** There's no map to localize against (the `vo_pose`/cuVSLAM scale bug below is deliberately deferred, not fixed) and no lidar on this robot for AMCL's `LaserScan` input anyway, even if a map existed. **Known, accepted limitation:** odom drift is unbounded over time with nothing correcting it, so the longer the robot runs, the more the planner's idea of "where things are" silently diverges from reality. Fine for a short single-goal test drive; not for long/repeated runs without real localization.
- **No obstacle costmap layer** — only an inflation layer around the robot's own footprint. Semantic/obstacle detection is later-roadmap scope. **This MVP does not avoid obstacles.**
- **`odom_to_tf.py` bridge node, added because of a real gap found during design:** the ESP32 firmware (`esp32/motor_f1/main/motor_f1.c`) publishes `/odom` as a plain `nav_msgs/Odometry` topic only — it never broadcasts the `odom → base_link` TF. Nav2's costmaps and controller read robot pose from TF, not from the topic directly, so without this bridge Nav2 fails immediately on any goal with "Could not get robot pose." This is a small, separate Jetson-side ROS2 node (not a colcon package — run by file path like everything else in `jetson/`), deliberately not added to the ESP32 firmware itself to keep the firmware hardware-facing-only.

**Files:**
- `nav2_params.yaml` — `controller_server` (Regulated Pure Pursuit), `planner_server` (NavFn), `behavior_server` (spin/backup/wait recoveries), `bt_navigator`, local + global costmap (both rolling-window, `global_frame: odom`), `lifecycle_manager`. Speeds kept conservative (`desired_linear_vel: 0.15` m/s) relative to the ~0.3 m/s already proven safe on real hardware during the `vo_pose` `/cmd_vel` trials — raise once this MVP is proven end-to-end. `robot_radius: 0.10` is a conservative estimate from the measured wheel diameter/wheelbase (6cm / 10cm), not yet measured off the real chassis.
- `nav2_launch.py` — launches `odom_to_tf.py` + the 4 Nav2 lifecycle nodes together.
- `odom_to_tf.py` — the TF bridge described above.

**To run (not yet verified on hardware, expect at least one bring-up bug on first try):**
```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup   # if not already present
# bring up micro-ROS agent + ESP32 first so /odom is live
ros2 launch jetson/nav2/nav2_launch.py
```
Then in RViz2: set Fixed Frame to `odom`, add a TF display, use the "2D Goal Pose" tool to send a goal, confirm the robot drives toward it via `/cmd_vel`.

**Handoff note for whoever picks this up next (see Team & Work Split below):** the original work split assigned "Nav2 stack: YAML config, planner selection, costmap layers" to Alex for Week 5. This MVP was built ahead of that, directly against the `/odom` contract, to unblock a closed-loop demo sooner given the project's compressed timeline. Coordinate before extending it (e.g. adding the obstacle layer, tuning the controller) to avoid duplicate work.

## Roadmap

| Week | Deliverable |
|------|-------------|
| 1 ✅ | micro-ROS hello world — ESP32 publishes ROS2 topic on Jetson |
| 2 ✅ | Motor driver + encoder wiring, ESP32 publishes `/odom` |
| 3 🔄 | IMX219 → isaac_ros_visual_slam → trajectory in RViz2 — pipeline runs end-to-end, trajectory publishes; **open bug: `vo_pose` scale is wrong, deliberately deferred** (see Lessons Learned 2026-08-04/06-07) |
| 4 ✅* | PID velocity control, `/cmd_vel` → accurate robot movement — done ahead of schedule alongside Week 2's F3/F5 |
| 5 🔄* | Nav2 closed loop — **MVP built ahead of schedule** (see [Nav2 — Closed-Loop MVP](#nav2--closed-loop-mvp-jetsonnav2) above), routed on `/odom` instead of SLAM since Week 3's bug is unresolved; not yet hardware-tested, no obstacle avoidance yet |
| 6 | Semantic navigation: TensorRT object detection → navigate to target |
| 7 | Stress test, metrics, GitHub, demo video |
| 8 | Buffer / stretch goals (waypoint patrol, return-to-dock, multi-session map) |

\* Week 4's PID/`/cmd_vel` work landed early because motor control needed to be solid before odometry (F4/F5) could be tested — see Week 2 sections above. Week 5's Nav2 MVP landed early too, routed on `/odom` instead of waiting on the Week 3 SLAM bug — see the Nav2 section above for scope/limitations.

## ROS2 Topic Interface (contract between ESP32 stack and Jetson stack)

This is the boundary the two halves of the team build against. Either side can develop independently as long as message type and topic name match — the ESP32 side doesn't need to know how `/cmd_vel` was computed, and the Jetson side doesn't need to know how `/odom` was computed.

**Status column reflects what's actually wired today, not the original design intent** — the Nav2 MVP (`jetson/nav2/`) currently runs on `/odom` alone, not `/visual_slam/tracking/odometry` + `/map`, because there's no map source yet (the `vo_pose` scale bug, see Lessons Learned, is deliberately deferred) and no lidar for AMCL. The visual_slam→Nav2 rows below are the future/planned path once that's resolved, kept in the contract table so it's clear where they'll plug back in.

| Topic | Message Type | Publisher | Subscriber | Status |
|-------|-------------|-----------|------------|--------|
| `/camera/image_raw` | `sensor_msgs/Image` | argus_camera (Jetson) | visual_slam (Jetson) | ✅ live |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | argus_camera (Jetson) | visual_slam (Jetson) | ✅ live |
| `/imu` | `sensor_msgs/Imu` | ESP32 (micro-ROS) | visual_slam (Jetson) | 🔌 wired, not read in firmware yet |
| `/odom` | `nav_msgs/Odometry` | ESP32 (micro-ROS) | `odom_to_tf.py` → TF, and Nav2 costmaps (Jetson) | ✅ live, this is what Nav2 actually navigates on today |
| `/tf` (`odom`→`base_link`) | `tf2_msgs/TFMessage` | `jetson/nav2/odom_to_tf.py` (Jetson) | Nav2 costmaps + controller (Jetson) | ✅ live (added with the Nav2 MVP — ESP32 only ever published the `/odom` topic, never TF) |
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 (Jetson) | ESP32 (micro-ROS) | ✅ live |
| `/goal_pose` | `geometry_msgs/PoseStamped` | RViz2 "2D Goal Pose" (Jetson) | Nav2 `bt_navigator` (Jetson) | ✅ live |
| `/visual_slam/tracking/odometry` | `nav_msgs/Odometry` | visual_slam (Jetson) | Nav2 (Jetson) | ⏳ planned — blocked on the `vo_pose` scale bug |
| `/map` | `nav_msgs/OccupancyGrid` | visual_slam (Jetson) | Nav2 costmap (Jetson) | ⏳ planned — no map-building pipeline running yet |

## Team & Work Split

Two-person team. Split is drawn along one line: **does this task require physically touching the robot?** Alex (remote) cannot solder, reseat a cable, or hear a motor to tune PID — so anything requiring hands-on-hardware iteration stays with the on-site owner. Anything that is pure software/config, or can be developed and dry-run against logged/simulated data (e.g. a `ros2 bag` recording, or mock topic publishers), is fair game to build remotely and integrate later.

**Ngoc Giang (vịt) — on-site, owns the physical stack:**
- ESP32 firmware requiring real hardware feedback: `encoder_task`, `pid_task` (PID tuning needs to hear/see the real motor respond — cannot be tuned blind), odometry math, `uros_task`
- All hardware bring-up: soldering, wiring, camera mounting/calibration, IMU mounting
- On-device validation: carrying the robot to check SLAM trajectory (Week 3), physically measuring drift (Week 7)

**Alex (remote) — owns the software/config stack, buildable without the physical robot:**
- Isaac ROS Docker setup + `visual_slam` launch/config (Week 3) — can be built and dry-run against a sample rosbag or public IMX219 dataset before the real camera feed is ready
- Nav2 stack: YAML config, planner selection, costmap layers (Week 5) — **an MVP (`jetson/nav2/`) was built ahead of schedule to unblock a closed-loop demo, see [Nav2 — Closed-Loop MVP](#nav2--closed-loop-mvp-jetsonnav2) above; check there before starting new Nav2 work to avoid duplicating it.** Remaining open work: real hardware test/tuning, obstacle costmap layer, eventually swapping `/odom` for `/visual_slam/tracking/odometry` + a real map once the Week 3 SLAM bug is fixed
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
│   ├── calibration/                 # stereo calibration: capture pairs, calibrate, npz -> camera_info YAML
│   │   ├── capture_stereo_pairs.py
│   │   ├── stereo_calibrate.py             # pinhole model (the one in use)
│   │   ├── stereo_calibrate_fisheye.py     # tried, not used -- pinhole error was already good
│   │   ├── npz_to_camera_info_yaml.py            # raw mode (Tx=0, baseline carried by TF)
│   │   ├── npz_to_camera_info_yaml_rectified.py  # rectified mode (real cv2.stereoRectify P)
│   │   └── visualize_corners.py
│   ├── slam/                        # Isaac ROS visual_slam launch files -- run INSIDE the
│   │   │                            # isaac_ros-dev Docker container (~/workspaces/isaac_ros-dev
│   │   │                            # on the Jetson, not vendored into this repo -- copy these
│   │   │                            # launch files + stereo_calibration.npz in after every pull)
│   │   ├── visual_slam_argus.launch.py            # raw mode, the known-working baseline
│   │   └── visual_slam_argus_rectified.launch.py  # rectified mode, currently produces frozen pose (open bug)
│   ├── nav2/                        # Nav2 closed-loop MVP (Week 3) -- runs NATIVE on the
│   │   │                            # Jetson's ROS2 Humble, outside the Isaac ROS container
│   │   ├── nav2_params.yaml         # controller/planner/behavior/bt_navigator + costmaps, global_frame=odom
│   │   ├── nav2_launch.py           # launches the 4 lifecycle nodes + odom_to_tf bridge
│   │   └── odom_to_tf.py            # bridges ESP32's /odom topic into the odom->base_link TF Nav2 needs
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

**Marathon session (2026-08-04) — stereo calibration, Isaac ROS bring-up on the correct JetPack version, and a first (imperfect) live cuVSLAM run. Long session, several distinct root causes; recorded in order.**

**Part 1 — stereo calibration, baseline off by 22%, root cause was a loose CSI cable, not the math.** Captured 20 checkerboard pairs (`jetson/calibration/capture_stereo_pairs.py`), ran `cv2.stereoCalibrate` (`jetson/calibration/stereo_calibrate.py`): reprojection error was excellent (~0.33px) but computed baseline came out 0.1016m against a measured physical baseline of ~0.083m — a 22% miss too large to blame on measurement noise (re-measured checkerboard square at 24.5-25mm, physical baseline at 85mm, both close to the script's inputs). First hypothesis (this camera is a 160°-FOV IMX219, standard pinhole distortion model might not fit that wide an FOV) was tested via `cv2.fisheye` — that attempt diverged numerically (250px error on one camera) and, on reflection, was never well-supported anyway since the *pinhole* model's own per-camera reprojection error was already excellent for both cameras. **Actual root cause, found only after vịt mentioned mid-session that the camera rig had been physically bumped:** re-ran `sudo dmesg | grep imx219` and found one camera failing i2c probe with the exact `-121` error from the original CSI defect saga months earlier — a loose CSI ribbon cable connector, not a math or model problem. Reseated the cable, recaptured 26 pairs (new shot plan: close/normal/far/edge/tilt, in `capture_stereo_pairs.py`), reran calibration: baseline came out 0.0854m vs. 0.083m measured (2.4% off) — confirms the cable was the entire story. **Lesson, consistent with the project's now-repeated pattern (CSI cable, encoder VCC, encoder solder, GPIO16/18 wire):** when calibration/sensor math produces a plausible-looking but wrong number, check the physical connectors before trusting a model-level explanation, especially if anything was recently handled.

**Part 1b — dual-camera live capture inside one Python process was fundamentally unreliable; fixed by spawning `gst-launch-1.0` as a fresh subprocess per shot.** Holding two `cv2.VideoCapture` Argus sessions open in one process (sequential open/close, even with settle delays) reliably broke the second camera with `nvbuf_utils: dmabuf_fd -1` and eventually wedged `nvargus-daemon` entirely (`sudo systemctl restart nvargus-daemon` was needed repeatedly). Rewrote `capture_stereo_pairs.py` to shell out to `gst-launch-1.0` per single-frame capture instead — the same recipe already proven reliable for early single-camera verification. Each OS-level subprocess exits and fully releases Argus on completion, so there's nothing left to leak between shots.

**Part 2 — Isaac ROS: the apt-install path is dead for JetPack 6/Humble; needed the Docker dev-container workflow, and needed the right *release* branch, not just the right repo.** `sudo apt install ros-humble-isaac-ros-visual-slam` failed (`Unable to locate package`) — NVIDIA removed prebuilt Humble debs from their apt repo (confirmed via NVIDIA forum thread, not assumption). Pivoted to `isaac_ros_common`'s Docker dev-container (`./scripts/run_dev.sh`), but the repo's default `main` branch (`v4.4-0`) targets JetPack 7/ROS2 Jazyy/Jetson Thor — not this hardware. Checking out `release-3.2` (confirmed via NVIDIA forum as the release built for JetPack 6.2 + Humble) fixed it; verified concretely by checking which L4T apt repo the built container pulled packages from (`apt-cache policy | grep nvidia.com/jetson`) — `release-2.1` pulled from `r35.4` (JetPack 5, wrong), `release-3.2` pulled from `r36.4` (same L4T 36.x family as this Jetson's actual `r36.5` — right generation). **Lesson: for a fast-moving NVIDIA repo with many release branches, check out the specific release matching your JetPack version before assuming `main`/latest works — and verify the match with something concrete (which apt repo it actually pulls from), not just a doc claim.**

**Part 2b — chain of smaller build-environment bugs, each with a real fix, recorded so a fresh setup doesn't rediscover them:**
- `vpiConfig.cmake` not found even after installing `vpi2-dev` — the package's own `dpkg` metadata claimed it was installed, but the actual files were missing on disk (confirmed via `ls` after `dpkg -L` said they should exist) because `apt install` no-ops when it thinks a package is "already the newest version." Fixed with `apt-get install --reinstall`.
- CDI (Container Device Interface) GPU injection failed (`unresolvable CDI devices nvidia.com/gpu=all`) on the `release-3.2` container even though the exact same Jetson's Docker+GPU had been verified working earlier the same session via the older `--runtime nvidia` flag — different container tooling generations expect different GPU-passthrough mechanisms. Fixed with `sudo nvidia-ctk cdi generate --output=/etc/cdi/nvidia.json`.
- `colcon build --symlink-install` failed on a leftover `build/`/`install/` directory from an earlier attempt under the *wrong* Isaac ROS release (release-2.1) — switching git branches doesn't clean previously-built artifacts on the host, and colcon's symlink step collided with real files left behind. Fixed with `rm -rf build install log` before rebuilding.

**Part 3 — first live cuVSLAM run: two real config bugs found via the actual error text, not guessed; then a genuine open item (pose scale) logged for next session, not chased further at 22:00 after a 12+ hour session.**
- Built a custom `jetson/slam/visual_slam_argus.launch.py`: 2x `ArgusMonoNode` (from `isaac_ros_argus_camera`, NVIDIA's official Argus-based CSI driver — chosen over hand-rolling a GStreamer→ROS bridge specifically because this project had just spent hours fighting Argus session conflicts, and NVIDIA's own driver already handles multi-camera Argus sessions correctly) feeding directly into `isaac_ros_visual_slam`'s `VisualSlamNode` with `rectified_images:=False` (raw distorted images + accurate `camera_info` straight to cuVSLAM, which rectifies on GPU — no separate rectification node needed). `camera_info` YAML for each camera generated directly from `stereo_calibration.npz` (`jetson/calibration/npz_to_camera_info_yaml.py`); the left↔right camera TF is computed from the same calibration's R/T (not guessed), via a rotation-matrix-to-quaternion conversion in the launch file itself.
- **Bug found: `sync_matching_threshold_ms` default (5ms) assumes hardware-fsync'd stereo** (e.g. NVIDIA's own Hawk module). This rig is two independent `ArgusMonoNode` instances with no fsync line between them — every frame pair was being rejected as "not synchronized enough," so `visual_slam_node` never processed a single pair (confirmed via `/visual_slam/status` never publishing, while `ros2 topic hz` on the raw image topics showed images genuinely arriving). Raised to 50ms.
- **Bug found: `[ERROR] Image 0 dimensions (3280x2464) do not correspond to camera resolution (1280x720)`.** `ArgusMonoNode`'s `mode` parameter defaults to 0 (full sensor resolution); calibration was done at 1280x720, which is a *different* sensor mode (mode 4, same enumeration order GStreamer's `nvarguscamerasrc` reports). Set `mode: 4` explicitly on both camera nodes — cuVSLAM tracker then initialized successfully (`cuVSLAM tracker was successfully initialized`) and `/visual_slam/tracking/vo_pose` began publishing real pose data at a stable 30Hz.
- **Confirmed real image throughput was ~9-10Hz under the wrong (mode 0) resolution** via `ros2 topic hz` — after the mode fix, re-measuring showed genuine ~29-30Hz, so the initial "camera is just slow" theory for a separate jitter warning was correctly abandoned once re-measured rather than patched around blindly.
- **Open, unresolved: `vo_pose` position values are not physically plausible.** Fast hand-movement of the camera rig caused values to diverge into the thousands within seconds (confirmed this was fast-motion-induced tracking loss, not a pipeline bug, by cross-checking with vịt: movement was fast, scene had normal texture — classic motion-blur/large-inter-frame-displacement VSLAM failure, not a config error). A clean-restart + deliberately slow motion test did **not** explode, but settled into an oscillating band (~x=11-16, not the small centimeter-scale values seen in the very first post-launch reading of 0.04/-0.07/-0.03) — looks like a **scale error** rather than lost tracking.

**Part 4 (same day, continued) — one scale-error hypothesis tested and disproven: manually patching `camera_info`'s baseline made things worse, not better.** Standard ROS stereo convention says the right camera's projection matrix should carry `Tx = -fx * baseline`; `npz_to_camera_info_yaml.py` had it hardcoded to `0.0`. Patched it in (PR #27) and tested live — `vo_pose` got worse (x/y/z jumped to ~58-83 instead of the prior ~11-16-on-x-only band), not better. **Reverted (PR #28).** Read as evidence that cuVSLAM already derives baseline from the static TF between `left_camera_optical_frame`/`right_camera_optical_frame` (published in the launch file from the calibration's own R/T) — adding a second baseline source via `camera_info` didn't get ignored, it compounded. Raw-mode `camera_info` is back to `Tx=0`; don't re-attempt this exact patch without new evidence.

**Part 5 (same day, continued) — built a `rectified_images:=True` comparison path (PR #29) to isolate the bug, but hit a different, deeper problem instead of an answer.** New `jetson/calibration/npz_to_camera_info_yaml_rectified.py` runs real `cv2.stereoRectify()` (baseline correctly encoded in `P` by OpenCV's own math this time, not hand-patched) and new `jetson/slam/visual_slam_argus_rectified.launch.py` adds an `isaac_ros_image_proc` `RectifyNode` per camera between `ArgusMonoNode` and `VisualSlamNode` — deliberately keeping the same TF and sync params as the raw-mode launch file so only one variable changes. Required cloning + building `isaac_ros_image_pipeline` (`release-3.2`) fresh; hit a `BUILD_TESTING` CMake/gtest failure building it, fixed with `--cmake-args -DBUILD_TESTING=OFF` (the package's own unit tests aren't needed, just the `RectifyNode` library).

Live test result: `vo_pose` stayed frozen at exact `(0,0,0)` / identity quaternion the entire time despite confirmed real camera movement — not a scale error this time, looks like zero tracking. cuVSLAM did initialize successfully (same log line as raw mode) and `ros2 topic hz /visual_slam/image_0` confirmed `RectifyNode` was genuinely publishing at ~28-29Hz, so it's not a dead node. Tried to inspect whether the rectified images themselves are valid (not black/corrupt) by writing a small `rclpy` subscriber script — it timed out waiting for a frame, on *both* the rectified topic and the raw pre-rectify Argus topic, despite `ros2 topic hz` proving real data flow on both. Ruled out QoS (tried default and `qos_profile_sensor_data`; `ros2 topic info -v` showed a compatible existing publisher/subscriber pair with the correct `sensor_msgs/msg/Image` type). Since it reproduces on the raw Argus topic too, this isn't a `RectifyNode`-specific bug — it looks like a general property of `isaac_ros_argus_camera`'s NITROS-accelerated composable-node publishers (likely a zero-copy/type-adaptation transport that only fully negotiates with another NITROS-aware subscriber in-process) that a plain out-of-process `rclpy` script can't read, unrelated to the original SLAM bug.

**Next session, start here:** don't keep hand-rolling `rclpy` subscriber scripts to inspect NITROS-published image topics — use RViz2 or `rqt_image_view` (NITROS-aware tooling) to visually check `/visual_slam/image_0`, or read up on Isaac ROS/NITROS type-adaptation first. Separately, a cheaper and cleaner next test for the *original* raw-mode scale bug (not yet tried): move the camera a precisely known distance (tape measure, one clean slow motion) and read the exact `vo_pose` displacement ratio — a precise, repeatable ratio (e.g. exactly 10x or 100x) would be far more diagnostic than the current "oscillates somewhere around 11-16" and might point straight at a units bug. If this keeps dead-ending, the fallback is to accept "trajectory shape correct, absolute scale wrong" as Week 3's known limitation and fix it before Nav2 (Week 5) actually needs metric accuracy, rather than blocking on it indefinitely now.

All of today's tooling lives in `jetson/calibration/` (capture, stereo calibration, npz→camera_info conversion, corner-detection visualizer) and `jetson/slam/` (the combined launch file). The Isaac ROS workspace itself (`isaac_ros_common`, `isaac_ros_visual_slam`, `isaac_ros_argus_camera`, all pinned to `release-3.2`) lives outside this repo at `~/workspaces/isaac_ros-dev` on the Jetson — deliberately not vendored into `slam-amr`, since it's third-party source + a multi-GB Docker build context, not project code. The container does not mount `~/slam-amr`, so any file a launch file needs to read (the launch file itself, `stereo_calibration.npz`) must be copied into `~/workspaces/isaac_ros-dev/` after each `git pull` — there's no live symlink between the two trees.

**Marathon session (2026-08-06/07) — fixed a fresh Isaac ROS environment gap (missing `isaac_ros_nitros`, an upstream NVIDIA CMake bug), then ran a systematic hand-motion-vs-wheel-motion investigation into the `vo_pose` bug logged above. No code changes to this repo were needed tonight — everything below happened on the Jetson's separate `~/workspaces/isaac_ros-dev` workspace and via live `ros2` commands.**

**Part 1 — `visual_slam_argus.launch.py` failed to load at all with `dlopen error: libisaac_ros_nitros.so`/`libgxf_isaac_optimizer.so: cannot open shared object file`, on a workspace that had run fine on 2026-08-04.** Root cause chain, each step confirmed concretely (not assumed):
- `isaac_ros_nitros` turned out to be a genuinely separate upstream repo (`github.com/NVIDIA-ISAAC-ROS/isaac_ros_nitros`, release-3.2), not a subfolder of `isaac_ros_common` as assumed — never cloned into this workspace's `src/`. No `.repos` file existed to auto-fetch it via `vcs import`, so it needed a manual `git clone --branch release-3.2`. It bundles both real C++ source (the NITROS runtime) and precompiled GXF extension `.so` binaries (incl. `gxf_isaac_optimizer`) distributed via git-lfs (~1GB, auto-pulled on clone since git-lfs was already installed).
- Building it hit a real bug in NVIDIA's own repo, confirmed via their GitHub issue tracker (other users hit the identical error): `isaac_ros_gxf/CMakeLists.txt` and `isaac_ros_gxf_extensions/gxf_isaac_messages/CMakeLists.txt` both reference a `magic_enum::magic_enum` CMake target without ever calling `find_package(magic_enum)` — doesn't fail at configure time (a `set_property(... INTERFACE_LINK_LIBRARIES ...)` call doesn't validate the target exists), only fails later at a *different* package's link step with a confusing `cannot find -lmagic_enum::magic_enum` linker error.
- Fix chain: (1) build+install `magic_enum` from source (header-only, not available via apt/rosdep on this image) — `git clone https://github.com/Neargye/magic_enum.git && cmake && make && sudo make install`; (2) manual symlink `sudo ln -s /usr/local/include/magic_enum/magic_enum.hpp /usr/local/include/magic_enum.hpp` since the current upstream `magic_enum` nests headers under a subdirectory but NVIDIA's GXF code does a bare `#include "magic_enum.hpp"` written for an older flat-layout release; (3) `sed -i '/ament_auto_find_build_dependencies()/a find_package(magic_enum REQUIRED)'` into both of the two CMakeLists.txt files above. Rebuilt clean, 34 packages, 0 failures. Also used `--packages-skip isaac_ros_image_pipeline gxf_isaac_image_flip` to dodge a separate, unrelated `magic_enum` build failure in the deprioritized rectified-mode experiment repo — not needed for the raw-mode path this session cared about.

**Part 2 — three concrete hypotheses for the `vo_pose` scale bug tested by direct measurement and ruled out, narrowing the search space:**
- **TF baseline** — `ros2 run tf2_ros tf2_echo left_camera_optical_frame right_camera_optical_frame` while raw-mode was live: translation `(0.085074, ...)` m, matches the 0.0854m calibration baseline almost exactly. Correct.
- **`camera_info`'s `Tx`** — confirmed by reading `npz_to_camera_info_yaml.py`: `Tx=0` for both cameras is intentional (baseline is meant to be carried entirely by TF in raw mode), not a bug.
- **Stereo camera sync timing** — compared `header.stamp` on `/visual_slam/camera_info_0` vs `_1` (plain, non-NITROS messages, safe to `ros2 topic echo` directly) across ~30 matched frame pairs: a stable **~3.28ms** offset, far under the 50ms tolerance and far too small to explain the bug.

**Part 3 — the real breakthrough: any physical disturbance to the camera rig (not a scale/calibration bug) reliably wrecks tracking; genuinely undisturbed wheel-driven motion doesn't.**
- Tried the classic reproduction first — pointing the camera at the checkerboard (leftover from calibration) reliably lost tracking even under slow motion. Root cause: a checkerboard is a near-worst-case VO scene (repetitive near-identical corners → ambiguous feature matching; one flat plane filling the frame → no parallax for stereo depth). **This also raises the possibility that the original 2026-08-04 "scale error" entry above was itself partly a checkerboard-scene artifact**, since that test happened right after calibration work — not confirmed either way, worth re-checking.
- Re-tested on a genuinely rich, well-lit scene (a real workshop view, confirmed by photo) — **still lost tracking on every hand-manipulated push**, ruling out "bad scene" as the sole cause. `cuVSLAM tracker` does **not self-recover** from a lost-tracking state once it happens — it free-runs from the broken (frozen or wildly drifting) pose until the pipeline is relaunched. Confirmed repeatedly, including once when a robot briefly wobbled and had to be caught by hand mid-test, and once when it collided with an obstacle — both caused the identical catastrophic-divergence signature.
- Switched to genuinely hands-off motion: drove the robot via `ros2 topic pub /cmd_vel` (ESP32/micro-ROS `/cmd_vel`→`/odom` path, confirmed working since Week 2) instead of hand-pushing, comparing real displacement (`/odom`, already trusted) against `/visual_slam/tracking/vo_pose` for the identical motion. **Two clean (no hand contact from relaunch through test) trials, same `0.3 m/s × 5s` command, in the good scene:**
  - Trial A: real `/odom` displacement ≈15.2cm, `vo_pose` displacement ≈77.8cm → ratio ≈**5.1x**.
  - Trial B: real `/odom` displacement ≈32.0cm, `vo_pose` displacement ≈95.3cm → ratio ≈**3.0x**.
  - Neither trial diverged catastrophically — both stayed bounded, unlike every single hand-touched trial. **A key procedural lesson learned twice the hard way: relaunch the pipeline *after* the robot is already sitting in its final test position, hands off — relaunching first and then carrying/repositioning the rig re-contaminates the very next reading before any `/cmd_vel` is even sent.**
- **Bottom line, next session starts here:** hand/collision disturbance → catastrophic, unrecoverable-without-relaunch tracking loss (an experimental-protocol hazard, not a code bug — avoid touching the rig during any live test). Clean wheel-only motion → tracking stays alive with a real, bounded **~3-5x apparent-vs-real scale discrepancy**, reproducible across independent trials. Next step: several more `/cmd_vel` trials (longer distances to reduce relative noise, always relaunch-before-touch, ideally logging full trajectories rather than single before/after points) to pin down whether that ratio is a stable constant (→ likely a fixable calibration/scale issue, e.g. in `camera_info`'s `fx`/`fy`) or still noisy at these short distances — before touching any camera_info/TF/calibration code.
