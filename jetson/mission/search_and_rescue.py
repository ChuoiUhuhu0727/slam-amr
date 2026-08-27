"""Option 1 search-and-rescue demo: robot patrols the room perimeter (hardcoded
waypoints, go-to-goal control using /odom -- gyro-fused heading, wheel-encoder
position), running duck detection the whole time. Reports which grid cell the
duck was in once the loop completes.

Also runs a live web dashboard (Flask, background thread) showing the room
map, robot position/heading, duck sightings, and both camera feeds with the
current detection box drawn on each -- for watching/debugging a live run
instead of only reading terminal logs. Reachable from any device on the same
network at http://<jetson-hostname>.local:8080 (this Jetson's mDNS hostname is
chuoi.local, already set up from earlier SSH work).

Deliberately does NOT use Nav2, VSLAM, or the Isaac ROS stereo depth pipeline:
- Nav2/VSLAM: skipped because cuVSLAM still has an unresolved ~3-5x pose scale
  bug (see project memory), and Nav2's costmap/planner stack has never been
  tested end-to-end on real hardware. A small known/bounded room doesn't need
  either -- hardcoded waypoints + /odom (already tested, gyro-fused heading)
  cover it.
- Isaac ROS stereo depth (/stereo/points2): needs the full Isaac ROS Docker
  container + rectified stereo launch just to get points, and produces a full
  dense point cloud we don't need. Instead, this does its OWN lightweight
  stereo just for the one object we care about: run detection on both
  cameras' frames, use the pixel shift (disparity) of the duck's box between
  the two frames + the known 8.3cm camera separation to triangulate
  real-world distance. Needs only plain OpenCV + the existing calibration
  file -- no Isaac ROS / ROS image pipeline dependency. Replaces the earlier
  monocular distance-from-known-duck-height approach (kept as
  search_and_rescue_monocular_backup.py) which needed the duck's real height
  hardcoded -- this doesn't need to know the duck's size at all.

- IMPORTANT (found 2026-08-25): detection runs on the RAW frame (rotation-
  corrected only, NOT undistorted/rectified), not the rectified frame used
  for display and stereo math. best.pt was trained exclusively on raw,
  distorted single-camera frames (see jetson/dataset_collection/
  record_video.py -- writes cap.read() straight to disk, no undistort).
  Feeding it the fully undistorted+cropped rectified frame instead put it
  outside its training distribution -- confirmed as the cause of a real
  live-deploy accuracy collapse (worked great in first_test.py/monocular
  testing, "totally trash" once this stereo rig's rectify-then-detect
  pipeline went live). Fix: detect on the raw frame (matches training
  exactly, zero accuracy risk), then map just the resulting box's corner
  points into rectified pixel space via cv2.undistortPoints(..., R, P) --
  the same per-camera R/P stereoRectify() already produces for the
  map_left_x/y / map_right_x/y remap, just applied to points instead of the
  whole image. That keeps disparity/distance math (which needs rectified,
  epipolar-aligned coordinates) exact while detection itself never sees a
  geometrically altered frame. See _detect_tick / _undistort_point.

Physical setup this assumes (confirmed with vịt 2026-08-25): both cameras
still exactly 8.3cm apart, same height as each other, rigidly mounted as one
unit -- ONLY the whole rig's heading changed (rotated 45deg right, same as
the old single-camera mount angle) so it looks toward the room center while
perimeter-hugging. Because the two cameras' position relative to EACH OTHER
didn't change, the existing stereo_calibration.npz (from the original
forward-facing mount) is still valid -- no recalibration needed, only the
existing CAMERA_BEARING_OFFSET_RAD (rig-to-chassis angle) applies, same as
before.

Run modes:
  python3 search_and_rescue.py             # full run: navigate + detect
  python3 search_and_rescue.py --nav-only  # navigation loop only, no camera/model
                                            # (use this FIRST to measure real
                                            # odometry drift before trusting
                                            # detection coordinates on top of it)

Needs `pip3 install flask` in addition to the packages first_test.py already needs
(ultralytics, numpy, opencv).

Prerequisite: micro-ROS agent already connected (ESP32 publishing /odom,
subscribed to /cmd_vel) -- this node doesn't manage that.
"""
import argparse
import json
import math
import statistics
import subprocess
import threading
import time
from collections import deque
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

# ---------------------------------------------------------------------------
# CONFIG -- confirm/measure these before trusting the output
# ---------------------------------------------------------------------------

# Room origin = robot's start position, facing +x down one wall (confirmed
# with vịt 2026-08-24). Room extent -- x-axis is the direction the robot
# initially faces (down one wall), y-axis the other. Confirmed with vịt
# 2026-08-26 for this specific room: robot's +x direction runs along the
# 1m wall.
ROOM_WIDTH_M = 1.0    # x-axis extent
ROOM_LENGTH_M = 2.23  # y-axis extent

# Inset from the walls for the patrol path (physical clearance while
# perimeter-hugging) -- same 0.3m used in the original 2x2m room. On this
# room's 1m-wide axis that's still the same absolute 0.3m wall clearance
# as before (just a visually narrower 0.4m gap between the two inset
# lines) -- should still be safe based on that room's testing, but worth a
# physical eyeball check before the real run given how tight this one is.
WAYPOINT_INSET_M = 0.3

# Loop direction matters: the camera rig is mounted looking toward the
# robot's RIGHT (CAMERA_BEARING_OFFSET_RAD below). For it to look toward
# the room's interior (not out past the wall) the whole loop, the interior
# must stay on the robot's right side throughout -- which only happens
# walking the loop CLOCKWISE (right turn at every corner). Fixed
# 2026-08-26: this order starts with one in-place left turn (normal,
# already-supported behavior) toward the far wall first, then turns right
# at every corner from there -- the previous order turned left at every
# corner (counter-clockwise), which pointed the camera outward the entire
# patrol, never toward the duck.
WAYPOINTS = [
    (WAYPOINT_INSET_M, WAYPOINT_INSET_M),
    (WAYPOINT_INSET_M, ROOM_LENGTH_M - WAYPOINT_INSET_M),
    (ROOM_WIDTH_M - WAYPOINT_INSET_M, ROOM_LENGTH_M - WAYPOINT_INSET_M),
    (ROOM_WIDTH_M - WAYPOINT_INSET_M, WAYPOINT_INSET_M),
    (WAYPOINT_INSET_M, WAYPOINT_INSET_M),  # back to start -- also how we measure real loop drift
]

GRID_CELL_M = 0.5  # grid reporting resolution, same in both directions -> 2x4 grid here

# How far the camera RIG (both cameras, rigidly mounted together) is
# physically rotated relative to the chassis' forward direction (base_link
# +x), radians. While hugging the perimeter the robot's forward direction
# points ALONG the wall, not toward room center -- a straight-ahead-mounted
# rig would mostly look down the wall and rarely catch the duck. Angling it
# inward fixes this. Positive = rotated toward the robot's left (REP103
# CCW+). Confirmed: rig mounted 45deg toward the robot's RIGHT -> negative.
CAMERA_BEARING_OFFSET_RAD = -math.radians(45)

# Safety floor for stereo disparity (pixels). Real disparity shrinks as the
# duck gets farther away; near-zero or negative disparity means either the
# duck is implausibly far, or the two cameras' detections don't actually
# match the same object (a bad L/R pairing) -- reject rather than divide by
# a tiny/negative number and report a garbage distance.
STEREO_MIN_DISPARITY_PX = 2.0

# Sanity ceiling on computed distance (m). Room is 2x2m (diagonal ~2.83m) --
# anything past this is treated as a bad stereo match, not a real reading.
STEREO_MAX_DISTANCE_M = 5.0

# Rolling-median smoothing window for the live distance readout (2026-08-27,
# see ruler benchmark: raw single-frame distance swung 1.2-1.7m at a 1.5m
# ground truth). Root cause: distance = fx*baseline/disparity is a 1/x
# relationship -- at longer range disparity is already small, so the same
# few-px bounding-box jitter swings the computed distance much harder than
# it does up close. Median (not mean) specifically to reject the occasional
# badly-localized box outright rather than let it drag an average around.
# 5-8 frames chosen over the 10-20 a commercial camera might use internally
# -- this loop only runs ~TARGET_DETECT_HZ=5Hz, so 20 frames would be 4s of
# lag; the mission's robot moves slowly during the demo so 5-8 frames
# (~1-1.6s) trades off jitter-rejection against staying responsive.
STEREO_SMOOTHING_WINDOW = 8

GOAL_TOLERANCE_M = 0.10       # "close enough" to a waypoint
# Raised 2026-08-28 (was 0.35 rad / ~20deg): this threshold predates the
# IMU-based heading PID on the ESP32 - back when heading correction while
# driving was weak/noisy (encoder-only), stopping to pure-rotate below even
# a small error was the safer choice. Now that heading correction while
# actually driving is solid, a small error is better absorbed by that
# continuous correction than by stopping - confirmed live as the cause of
# the "stutters and stops to correct" symptom: every 100ms control tick,
# ANY heading error over the old 20deg threshold triggered a full stop.
# Raised to ~50deg - still stops for a genuinely large misalignment (a new
# leg needing close to a 90deg turn), but lets smaller ongoing deviations
# get corrected while still moving forward, not by halting.
TURN_IN_PLACE_THRESHOLD = 0.9  # rad (~50deg) -- above this, stop and turn first
# Raised 2026-08-26: PWM maxing at ~78/255 during live tests was traced to
# these caps, not a weak PID -- target_rpm = v / WHEEL_CIRCUMFERENCE_M * 60,
# so 0.15 m/s only ever asked for ~43 RPM, and the tiny WHEELBASE_M (0.10m)
# meant 0.8 rad/s in-place turns only asked for ~11 RPM per wheel. The
# firmware PID was correctly hitting those (small) targets at low PWM --
# retuning Kp/Ki would not have changed the ceiling, only convergence speed.
# Verify live after this change: ESP32 diag panel's RPM should now track
# the higher targets, not just PWM going up -- if RPM lags far behind PWM at
# these new values, THAT is real evidence of an undertuned/underpowered PID
# worth revisiting (it wasn't, at the old low targets).
MAX_LINEAR_SPEED = 0.30       # m/s -- still cautious for a 2.23m-long room
MAX_ANGULAR_SPEED = 1.5       # rad/s -- wheelbase is small, turns need more
                               # angular.z than you'd expect for real wheel RPM
KP_HEADING = 1.5

CONTROL_PERIOD_S = 0.1   # 10 Hz control loop, navigation only -- camera grab +
                         # detection run on their own thread (see _vision_loop),
                         # paced naturally by how long YOLO takes, not this timer.

# PID tuning box (2026-08-27) -- the ESP32's Kp/Ki/max-integral-contribution
# used to be #define constants, needing a reflash to change. They're now
# live-tunable over a new /pid_gains topic (see motor_f1.c pid_gains_callback)
# so vịt can iterate against the real robot from the dashboard. Persisted HERE
# on the Jetson (not the ESP32's flash/NVS) -- simpler, and matches the actual
# need: the ESP32 resets to these firmware defaults on every reboot, and this
# node re-publishes the saved gains on PID_GAINS_REPUBLISH_PERIOD_S so an
# ESP32 reboot/reconnect gets corrected back within a couple seconds without
# needing a manual click every time.
PID_GAINS_PATH = Path(__file__).resolve().parent / "pid_gains.json"
PID_GAINS_REPUBLISH_PERIOD_S = 2.0
# Must match motor_f1.c's g_kp/g_ki/g_max_i_contribution/g_kheading/g_trim_left/
# g_trim_right defaults -- these are only the fallback used before any dashboard
# save has ever happened (or if pid_gains.json is missing/corrupt).
DEFAULT_PID_GAINS = {'kp': 3.0, 'ki': 0.2, 'max_i': 40.0, 'khead': 15.0, 'trim_left': 0.0, 'trim_right': 0.0}


def load_pid_gains():
    try:
        with open(PID_GAINS_PATH) as f:
            data = json.load(f)
        return {
            'kp': float(data['kp']), 'ki': float(data['ki']),
            'max_i': float(data['max_i']), 'khead': float(data['khead']),
            'trim_left': float(data['trim_left']), 'trim_right': float(data['trim_right']),
        }
    except (FileNotFoundError, KeyError, ValueError, TypeError, json.JSONDecodeError):
        return dict(DEFAULT_PID_GAINS)


def save_pid_gains(gains):
    with open(PID_GAINS_PATH, 'w') as f:
        json.dump(gains, f)

WEIGHTS_PATH = Path(__file__).resolve().parents[1] / "training/runs/detect/train-5/weights/best.pt"
CALIB_PATH = Path(__file__).resolve().parents[2] / "stereo_calibration.npz"
CONF_THRESHOLD = 0.3
# Tried shrinking the image the model looks at (640 default -> 480 -> 384)
# to cut CPU cost, since detection runs on CPU here (torch/CUDA driver
# mismatch, separate pre-existing issue, not fixed). REVERTED 2026-08-26 --
# vịt caught a real accuracy regression from this (a wildly oversized,
# low-confidence box on the RIGHT camera -- the one already flagged as
# harder to detect on due to its color-cast difference from the left
# camera). Today's actual goal is validating distance ACCURACY, which
# depends entirely on getting a correct box -- not worth trading that for
# frame rate right now. DETECT_IMGSZ = None means "don't override, let
# ultralytics use its own default," exactly matching yesterday's working
# single-camera behavior. Revisit speed only after accuracy is confirmed
# sound, as its own separate, deliberate trade-off -- not bundled in again
# by accident.
DETECT_IMGSZ = None

# Caps the vision loop's rate -- NOTE this can only make it SLOWER (adds a
# pause if a loop finishes early), it can't make YOLO itself faster than
# whatever it naturally takes. Real benefit: (a) a steady, predictable
# cadence instead of a jittery flat-out one -- looks better for a live demo
# -- and (b) stops the vision thread from hogging CPU with zero breathing
# room for the dashboard/steering threads. If YOLO is already naturally
# slower than this, the cap is a no-op -- watch the terminal's "vision loop"
# log line to see the REAL achieved rate before assuming this changed
# anything.
TARGET_DETECT_HZ = 5.0


def csi_pipeline(sensor_id: int) -> str:
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        "video/x-raw(memory:NVMM), width=1280, height=720, framerate=30/1 ! "
        "nvvidconv ! "
        "video/x-raw, format=BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=BGR ! appsink drop=1"
    )


# Which physical camera plays the calibration's "camera1/left" role (K1/D1)
# vs "camera2/right" role (K2/D2) -- must match whichever sensor-id was
# captured as the LEFT file during capture_stereo_pairs.py, since that's
# what stereo_calibration.npz's K1/D1 (left) and K2/D2 (right) were fit to.
# This is about ELECTRICAL port pairing to calibration data, not physical
# geometry -- swapping the whole rig's orientation on the chassis (the
# 45deg mount) never breaks this. It WOULD break if the two camera modules'
# physical CSI connections got swapped/re-plugged since calibration (easy
# to do by accident when rebuilding the mount) -- vịt found live evidence
# of exactly that 2026-08-25 (disparity consistently negative even for
# far-away detections, not just close-range noise). Flipped here to match.
LEFT_SENSOR_ID = 1
RIGHT_SENSOR_ID = 0

WEB_HOST = "0.0.0.0"  # bind all interfaces so other devices on the wifi can reach it
WEB_PORT = 8080


def yaw_from_quaternion(q):
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_angle(a):
    """Keep an angle in (-pi, pi] -- without this, heading error near the
    +pi/-pi boundary makes the robot spin the long way around."""
    return math.atan2(math.sin(a), math.cos(a))


class SearchAndRescue(Node):
    def __init__(self, nav_only: bool):
        super().__init__('search_and_rescue')
        self.nav_only = nav_only

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.have_odom = False
        self.esp32_diag = ''  # raw text from /esp32_diag (RPM/PWM/reset/I2C from
                               # the firmware's own PID loop) -- shown as-is on
                               # the dashboard for troubleshooting, no parsing

        self.waypoint_idx = 0
        self.done = False
        self.started = False  # gated by the dashboard's Start button -- camera/
                               # detection run regardless, only driving waits

        self.duck_sightings = []  # list of (world_x, world_y)
        self.report = None        # filled in once the loop finishes

        self.latest_frame_left = None       # for the dashboard video feed (rectified)
        self.latest_frame_right = None
        self.latest_frame_left_raw = None   # rotation-corrected but NOT undistorted/rectified --
        self.latest_frame_right_raw = None  # this is what YOLO actually sees, matching training data
        self.latest_detection_left = None   # {'x1','y1','x2','y2','conf','t'} for the overlay box
        self.latest_detection_right = None
        self.latest_stereo_reading = None   # {'distance_m','bearing_deg','disparity_px','t'} -- for
                                             # a live "distance right now" readout, useful for a
                                             # ruler accuracy check without reading terminal logs
        self.stereo_distance_history = deque(maxlen=STEREO_SMOOTHING_WINDOW)  # raw distance_m
                                             # readings, most recent last -- rolling median of this
                                             # is the smoothed distance (see STEREO_SMOOTHING_WINDOW
                                             # comment). Kept separate from latest_stereo_reading
                                             # (raw) deliberately -- both shown on the dashboard so
                                             # the raw jitter is still visible, not hidden by smoothing.

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pid_pub = self.create_publisher(String, '/pid_gains', 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(String, '/esp32_diag', self.on_diag, 10)
        self.control_timer = self.create_timer(CONTROL_PERIOD_S, self.control_tick)

        self.pid_gains = load_pid_gains()
        self._publish_pid_gains()  # push once immediately at boot, in case the
                                    # ESP32 is already connected and waiting
        self.create_timer(PID_GAINS_REPUBLISH_PERIOD_S, self._publish_pid_gains)

        if not nav_only:
            self._init_vision()
        else:
            self.get_logger().info("--nav-only: skipping camera/model, navigation loop only")

        self._start_web_server()

    def _init_vision(self):
        import cv2
        from ultralytics import YOLO

        calib = np.load(str(CALIB_PATH))
        K1, D1, K2, D2 = calib["K1"], calib["D1"], calib["K2"], calib["D2"]
        R, T = calib["R"], calib["T"]
        width, height = int(calib["image_size"][0]), int(calib["image_size"][1])
        size = (width, height)

        baseline_m = float(calib["baseline_m"])
        if not (0.05 < baseline_m < 0.15):
            # Sanity check against the known ~0.083m mount -- if this is off,
            # something's wrong with the calibration file, fail loudly now
            # rather than silently reporting garbage distances all mission.
            raise RuntimeError(
                f"Calibration baseline_m={baseline_m:.4f} is way off the known "
                "~0.083m mount -- wrong/stale calibration file, refusing to start."
            )

        # Real stereo rectification (same math as the earlier abandoned
        # npz_to_camera_info_yaml_rectified.py, just applied directly in
        # OpenCV here instead of feeding it to Isaac ROS). CALIB_ZERO_DISPARITY
        # aligns principal points; alpha=0 crops to the valid-pixel region only.
        # This rig-rotation doesn't touch R/T (camera-to-camera geometry is
        # unchanged), so this is the same R/T that was already trusted before.
        R1, R2, P1, P2, _, _, _ = cv2.stereoRectify(
            K1, D1, K2, D2, size, R, T,
            flags=cv2.CALIB_ZERO_DISPARITY, alpha=0,
        )
        self.map_left_x, self.map_left_y = cv2.initUndistortRectifyMap(
            K1, D1, R1, P1, size, cv2.CV_32FC1)
        self.map_right_x, self.map_right_y = cv2.initUndistortRectifyMap(
            K2, D2, R2, P2, size, cv2.CV_32FC1)

        # Kept per-camera (not just the map_*_x/y pixel grids above) so
        # _undistort_point can run the same undistort+rectify transform on a
        # single detected box corner instead of a whole image -- see
        # _detect_tick, which detects on the RAW frame and only needs the
        # resulting box's coordinates in rectified space, not a rectified
        # image to detect on.
        self.K1, self.D1, self.R1, self.P1 = K1, D1, R1, P1
        self.K2, self.D2, self.R2, self.P2 = K2, D2, R2, P2

        # After rectification both cameras share the same focal length by
        # construction -- P1[0,0] and P2[0,0] should match closely.
        self.fx_rect = float(P1[0, 0])
        self.cx_rect = float(P1[0, 2])
        fx_rect_right = float(P2[0, 0])
        if abs(self.fx_rect - fx_rect_right) > 1.0:
            raise RuntimeError(
                f"Rectified fx mismatch L={self.fx_rect:.1f} R={fx_rect_right:.1f} "
                "-- stereoRectify output looks wrong, refusing to start."
            )
        self.baseline_m = baseline_m

        self.model = YOLO(str(WEIGHTS_PATH))
        self.cap_left = cv2.VideoCapture(csi_pipeline(LEFT_SENSOR_ID), cv2.CAP_GSTREAMER)
        if not self.cap_left.isOpened():
            raise RuntimeError(f"Could not open LEFT CSI camera (sensor-id={LEFT_SENSOR_ID})")
        self.cap_right = cv2.VideoCapture(csi_pipeline(RIGHT_SENSOR_ID), cv2.CAP_GSTREAMER)
        if not self.cap_right.isOpened():
            raise RuntimeError(f"Could not open RIGHT CSI camera (sensor-id={RIGHT_SENSOR_ID})")

        self.get_logger().info(
            f"Stereo vision ready: fx_rect={self.fx_rect:.1f}, cx_rect={self.cx_rect:.1f}, "
            f"baseline={self.baseline_m*100:.2f}cm"
        )

        # Camera grab + detection run on their own thread, completely decoupled
        # from the 10Hz ROS control timer -- previously this ran inline inside
        # control_tick, so a slow (CPU-only, no usable GPU on this Jetson) YOLO
        # call blocked frame grabs AND cmd_vel updates too, which was the real
        # source of the reported lag, not a network/dashboard issue. Results
        # are written to plain instance attributes; CPython's GIL makes single
        # reads/writes of these safe enough across threads for this use case
        # (no lock needed -- nothing here does a multi-step read-modify-write
        # on shared state, only whole-object replacement).
        self.vision_loop_count = 0
        threading.Thread(target=self._vision_loop, daemon=True).start()

    def _vision_loop(self):
        target_period_s = 1.0 / TARGET_DETECT_HZ
        while rclpy.ok():
            iter_start = time.time()
            self.vision_loop_count += 1
            self._grab_frame()
            self._detect_tick()
            elapsed = time.time() - iter_start

            if self.vision_loop_count % 20 == 0:
                # Real achieved rate, not the target -- if this is already
                # below TARGET_DETECT_HZ, the cap above isn't doing anything.
                self.get_logger().info(
                    f"Vision loop: {1.0/elapsed:.2f}Hz this iteration "
                    f"(target {TARGET_DETECT_HZ}Hz, {'capped' if elapsed < target_period_s else 'uncapped -- YOLO is the bottleneck'})"
                )

            remaining = target_period_s - elapsed
            if remaining > 0:
                time.sleep(remaining)

    # -- odometry -----------------------------------------------------------

    def on_odom(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.theta = yaw_from_quaternion(msg.pose.pose.orientation)
        self.have_odom = True

    def on_diag(self, msg: String):
        self.esp32_diag = msg.data

    # -- PID tuning -----------------------------------------------------------

    def _publish_pid_gains(self):
        msg = String()
        g = self.pid_gains
        msg.data = (f"KP={g['kp']:.4f},KI={g['ki']:.4f},MAXI={g['max_i']:.4f},KHEAD={g['khead']:.4f},"
                    f"TRIML={g['trim_left']:.4f},TRIMR={g['trim_right']:.4f}")
        self.pid_pub.publish(msg)

    def set_pid_gains(self, kp: float, ki: float, max_i: float, khead: float,
                       trim_left: float, trim_right: float):
        self.pid_gains = {'kp': kp, 'ki': ki, 'max_i': max_i, 'khead': khead,
                           'trim_left': trim_left, 'trim_right': trim_right}
        save_pid_gains(self.pid_gains)
        self._publish_pid_gains()

    # -- main loop ------------------------------------------------------------

    def control_tick(self):
        if not self.have_odom:
            return  # wait for first /odom message before doing anything

        # Camera grab + detection run on their own background thread (see
        # _vision_loop) -- this callback is navigation-only now, so a slow
        # detection frame can never delay a cmd_vel update.
        if self.done or not self.started:
            self.cmd_pub.publish(Twist())  # stay stopped -- either finished, or waiting for Start
            return

        self._navigate_tick()

    def _navigate_tick(self):
        goal_x, goal_y = WAYPOINTS[self.waypoint_idx]
        dx = goal_x - self.x
        dy = goal_y - self.y
        distance = math.hypot(dx, dy)

        if distance < GOAL_TOLERANCE_M:
            self.get_logger().info(
                f"Reached waypoint {self.waypoint_idx} ({goal_x:.2f},{goal_y:.2f}) "
                f"-- actual pos ({self.x:.2f},{self.y:.2f})"
            )
            self.waypoint_idx += 1
            if self.waypoint_idx >= len(WAYPOINTS):
                self.done = True
                self.cmd_pub.publish(Twist())
                self._report()
            return

        angle_to_goal = math.atan2(dy, dx)
        heading_error = wrap_angle(angle_to_goal - self.theta)

        cmd = Twist()
        if abs(heading_error) > TURN_IN_PLACE_THRESHOLD:
            # facing the wrong way -- stop and turn first, don't drive sideways into it
            cmd.linear.x = 0.0
            cmd.angular.z = max(-MAX_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, KP_HEADING * heading_error))
        else:
            cmd.linear.x = MAX_LINEAR_SPEED
            cmd.angular.z = max(-MAX_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, KP_HEADING * heading_error))
        self.cmd_pub.publish(cmd)

    def _grab_frame(self):
        import cv2

        ok_l, frame_l = self.cap_left.read()
        ok_r, frame_r = self.cap_right.read()
        if ok_l and ok_r:
            # Rig was physically remounted rolled 180deg 2026-08-26 (cable-
            # length reason) -- undo that here, on the RAW frame, before
            # anything else touches it. A 180deg roll about the lens's own
            # optical axis is optically equivalent to capturing normally and
            # rotating the picture afterward (true because the calibration's
            # distortion model is radially symmetric around the image
            # center) -- so correcting it here means the existing
            # rectification maps, disparity math, and bearing math all just
            # keep working unchanged, exactly as if the rig had never rolled.
            frame_l = cv2.rotate(frame_l, cv2.ROTATE_180)
            frame_r = cv2.rotate(frame_r, cv2.ROTATE_180)
            # Keep this raw (rotation-corrected only) pair for detection --
            # best.pt was trained on raw CSI frames, never on undistorted/
            # rectified ones, so YOLO must see this, not the remap() output
            # below. See _detect_tick / module docstring.
            self.latest_frame_left_raw = frame_l
            self.latest_frame_right_raw = frame_r
            self.latest_frame_left = cv2.remap(frame_l, self.map_left_x, self.map_left_y, cv2.INTER_LINEAR)
            self.latest_frame_right = cv2.remap(frame_r, self.map_right_x, self.map_right_y, cv2.INTER_LINEAR)
        elif self.vision_loop_count % 50 == 0:  # throttled -- don't spam if it keeps failing
            self.get_logger().warn(
                f"Camera frame grab failing (left ok={ok_l}, right ok={ok_r})"
            )

    def _best_box(self, frame):
        """Run detection on one RAW (rotation-corrected only, NOT undistorted/
        rectified) frame -- matches what best.pt was trained on, see module
        docstring. Returns the highest-confidence box as (x1,y1,x2,y2,conf)
        in that same raw frame's pixel coordinates, or None if nothing above
        threshold."""
        kwargs = {'imgsz': DETECT_IMGSZ} if DETECT_IMGSZ is not None else {}
        results = self.model(frame, conf=CONF_THRESHOLD, verbose=False, **kwargs)
        boxes = results[0].boxes
        if boxes is None or len(boxes) == 0:
            return None
        box = boxes[boxes.conf.argmax()]
        x1, y1, x2, y2 = box.xyxy[0].tolist()
        conf = float(box.conf[0])
        return (x1, y1, x2, y2, conf)

    def _undistort_point(self, x, y, K, D, R, P):
        """Map one raw-frame pixel coordinate into rectified-frame pixel
        coordinates -- the point-wise equivalent of the cv2.remap() applied
        to whole frames via map_left_x/y / map_right_x/y (same K/D/R/P,
        just via cv2.undistortPoints instead of a per-pixel grid). Lets
        detection stay on the raw frame while stereo math (which needs
        rectified, epipolar-aligned coordinates) still gets what it needs."""
        import cv2
        pt = np.array([[[x, y]]], dtype=np.float64)
        out = cv2.undistortPoints(pt, K, D, R=R, P=P)
        return float(out[0, 0, 0]), float(out[0, 0, 1])

    def _detect_tick(self):
        # Detection runs on the RAW (rotation-corrected only) frames -- see
        # module docstring / _best_box. Stereo math below needs rectified,
        # epipolar-aligned coordinates, so each box's corners get mapped
        # into rectified space via _undistort_point right after detection,
        # once, rather than detecting on an already-rectified frame.
        frame_l_raw = self.latest_frame_left_raw
        frame_r_raw = self.latest_frame_right_raw
        if frame_l_raw is None or frame_r_raw is None:
            return

        box_l = self._best_box(frame_l_raw)
        self.latest_detection_left = None
        self.latest_detection_right = None
        if not box_l:
            # Nothing to pair even if the right camera happens to see
            # something -- skip that second, equally expensive YOLO call
            # entirely. This is the common case during patrol (duck not
            # currently in view), so it roughly halves average detection
            # cost. Means the right feed won't show its own box unless the
            # left camera ALSO currently sees the duck -- an acceptable
            # trade since a lone-camera sighting was never usable anyway
            # (see below).
            return

        x1, y1, x2, y2, conf = box_l
        lx1, ly1 = self._undistort_point(x1, y1, self.K1, self.D1, self.R1, self.P1)
        lx2, ly2 = self._undistort_point(x2, y2, self.K1, self.D1, self.R1, self.P1)
        self.latest_detection_left = {'x1': lx1, 'y1': ly1, 'x2': lx2, 'y2': ly2, 'conf': conf, 't': time.time()}

        box_r = self._best_box(frame_r_raw)
        if not box_r:
            # Duck only visible in the left camera -- per plan, don't guess.
            # Skip this sighting, wait for a tick where both agree.
            return
        x1, y1, x2, y2, conf = box_r
        rx1, ry1 = self._undistort_point(x1, y1, self.K2, self.D2, self.R2, self.P2)
        rx2, ry2 = self._undistort_point(x2, y2, self.K2, self.D2, self.R2, self.P2)
        self.latest_detection_right = {'x1': rx1, 'y1': ry1, 'x2': rx2, 'y2': ry2, 'conf': conf, 't': time.time()}

        # Both boxes are now in rectified pixel space (same space the
        # fx_rect/cx_rect/baseline math below was already written for) --
        # everything past this point is unchanged from before the raw-frame
        # detection fix.
        center_x_l = (lx1 + lx2) / 2.0
        center_x_r = (rx1 + rx2) / 2.0

        disparity_px = center_x_l - center_x_r
        if disparity_px < STEREO_MIN_DISPARITY_PX:
            self.get_logger().warn(
                f"Duck seen in both cameras but disparity={disparity_px:.1f}px too small/negative "
                "(implausibly far, or L/R boxes don't match the same object) -- skipping sighting"
            )
            return

        distance = (self.fx_rect * self.baseline_m) / disparity_px
        if distance > STEREO_MAX_DISTANCE_M:
            self.get_logger().warn(
                f"Duck stereo distance={distance:.2f}m exceeds room-size sanity ceiling -- skipping sighting"
            )
            return

        # Pixel offset from image center -> bearing angle, using the LEFT
        # (reference) rectified camera, same convention as the old monocular
        # code. Camera rig has ~0 x/y offset from base_link, just faces a
        # fixed direction relative to the chassis -- no extra translation
        # needed, only the CAMERA_BEARING_OFFSET_RAD rotation below.
        # SIGN NOT YET EMPIRICALLY VERIFIED: image-right (+dx) should mean
        # "duck is to the robot's right" = negative yaw offset (REP103: CCW
        # positive). Test with the duck placed to one known side first: if
        # the reported y comes out on the wrong side, flip this sign.
        dx_px = center_x_l - self.cx_rect
        bearing = -math.atan2(dx_px, self.fx_rect)

        self.latest_stereo_reading = {
            'distance_m': distance,
            'bearing_deg': math.degrees(bearing),
            'disparity_px': disparity_px,
            't': time.time(),
        }
        self.stereo_distance_history.append(distance)

        world_x = self.x + distance * math.cos(self.theta + CAMERA_BEARING_OFFSET_RAD + bearing)
        world_y = self.y + distance * math.sin(self.theta + CAMERA_BEARING_OFFSET_RAD + bearing)

        self.duck_sightings.append((world_x, world_y))
        self.get_logger().info(
            f"Duck seen (stereo): disparity={disparity_px:.1f}px dist={distance:.2f}m "
            f"bearing={math.degrees(bearing):.1f}deg -> estimated world ({world_x:.2f}, {world_y:.2f})"
        )

    def duck_estimate(self):
        """Running average of every duck sighting so far -- a single-duck
        assumption by design (no multi-target clustering). Used live by the
        dashboard (updates continuously, not just at the end) and by the
        final report (same number, just also bucketed into a grid cell)."""
        if not self.duck_sightings:
            return None
        avg_x = sum(p[0] for p in self.duck_sightings) / len(self.duck_sightings)
        avg_y = sum(p[1] for p in self.duck_sightings) / len(self.duck_sightings)
        return {'x': avg_x, 'y': avg_y, 'sightings': len(self.duck_sightings)}

    def _report(self):
        estimate = self.duck_estimate()
        if estimate is None:
            self.get_logger().info("=== REPORT: loop complete, no duck sighted ===")
            self.report = {'sightings': 0}
            return

        avg_x, avg_y = estimate['x'], estimate['y']
        cell_x = max(0, min(int(ROOM_WIDTH_M / GRID_CELL_M) - 1, int(avg_x // GRID_CELL_M)))
        cell_y = max(0, min(int(ROOM_LENGTH_M / GRID_CELL_M) - 1, int(avg_y // GRID_CELL_M)))

        self.get_logger().info(
            f"=== REPORT: {estimate['sightings']} sighting(s), "
            f"averaged position ({avg_x:.2f}, {avg_y:.2f}) -> grid cell ({cell_x}, {cell_y}) ==="
        )
        self.report = {
            'sightings': estimate['sightings'],
            'avg_x': avg_x, 'avg_y': avg_y,
            'cell_x': cell_x, 'cell_y': cell_y,
        }

    # -- web dashboard --------------------------------------------------------

    def _start_web_server(self):
        import logging
        from flask import Flask, Response, jsonify, request

        # Flask's dev server logs every single request by default (including
        # the dashboard's own /state poll every 400ms) -- floods the terminal
        # and buries the actual ROS2/detection logs. Silence it; real errors
        # still raise exceptions and aren't hidden by this.
        logging.getLogger('werkzeug').setLevel(logging.ERROR)

        app = Flask(__name__)
        node = self

        @app.route('/')
        def index():
            return HTML_PAGE

        @app.route('/state')
        def state():
            return jsonify({
                'x': node.x, 'y': node.y, 'theta_deg': math.degrees(node.theta),
                'have_odom': node.have_odom,
                'esp32_diag': node.esp32_diag,
                'waypoint_idx': node.waypoint_idx,
                'waypoints': WAYPOINTS,
                'room_width': ROOM_WIDTH_M,
                'room_length': ROOM_LENGTH_M,
                'grid_cell': GRID_CELL_M,
                'done': node.done,
                'started': node.started,
                'duck_sightings': node.duck_sightings,
                'duck_estimate': node.duck_estimate(),
                'latest_stereo': node.latest_stereo_reading,
                'stereo_distance_smoothed_m': (
                    statistics.median(node.stereo_distance_history)
                    if node.stereo_distance_history else None
                ),
                'stereo_smoothing_window': STEREO_SMOOTHING_WINDOW,
                'report': node.report,
                'has_camera': not node.nav_only,
                'target_waypoint': WAYPOINTS[node.waypoint_idx] if node.waypoint_idx < len(WAYPOINTS) else None,
                'pid_gains': node.pid_gains,
            })

        @app.route('/pid_gains', methods=['POST'])
        def post_pid_gains():
            try:
                data = request.get_json(force=True)
                kp = float(data['kp'])
                ki = float(data['ki'])
                max_i = float(data['max_i'])
                khead = float(data['khead'])
                trim_left = float(data['trim_left'])
                trim_right = float(data['trim_right'])
            except (TypeError, ValueError, KeyError):
                return jsonify({'ok': False, 'error': 'expected JSON {kp, ki, max_i, khead, trim_left, trim_right} as numbers'}), 400
            node.set_pid_gains(kp, ki, max_i, khead, trim_left, trim_right)
            node.get_logger().info(f"PID gains updated via dashboard: Kp={kp} Ki={ki} MaxI={max_i} Khead={khead} "
                                    f"TrimL={trim_left} TrimR={trim_right}")
            return jsonify({'ok': True, 'pid_gains': node.pid_gains})

        @app.route('/video_feed_left')
        def video_feed_left():
            if node.nav_only:
                return Response("Camera not active in --nav-only mode", mimetype='text/plain')
            return Response(_mjpeg_generator(node, 'left'), mimetype='multipart/x-mixed-replace; boundary=frame')

        @app.route('/video_feed_right')
        def video_feed_right():
            if node.nav_only:
                return Response("Camera not active in --nav-only mode", mimetype='text/plain')
            return Response(_mjpeg_generator(node, 'right'), mimetype='multipart/x-mixed-replace; boundary=frame')

        @app.route('/start', methods=['POST'])
        def start():
            node.started = True
            node.get_logger().info("Patrol STARTED (via dashboard)")
            return jsonify({'started': True})

        @app.route('/stop', methods=['POST'])
        def stop():
            node.started = False
            node.cmd_pub.publish(Twist())
            node.get_logger().info("Patrol STOPPED (via dashboard)")
            return jsonify({'started': False})

        @app.route('/reset', methods=['POST'])
        def reset():
            # Resets the MISSION state only (waypoint progress, duck sightings,
            # start/done flags) -- NOT the robot's actual (x,y,theta), which
            # comes from the ESP32's own odometry and only zeroes when the
            # ESP32 itself reboots. If you physically move the robot back to
            # the start corner between test runs, the position number won't
            # go back to 0 just from clicking this -- that needs an ESP32
            # reset (unplug/replug, or the board's reset button).
            node.started = False
            node.done = False
            node.waypoint_idx = 0
            node.duck_sightings = []
            node.latest_detection_left = None
            node.latest_detection_right = None
            node.latest_stereo_reading = None
            node.stereo_distance_history.clear()
            node.report = None
            node.cmd_pub.publish(Twist())
            node.get_logger().info("Dashboard: RESET (mission state cleared)")
            return jsonify({'reset': True})

        thread = threading.Thread(
            target=lambda: app.run(host=WEB_HOST, port=WEB_PORT, debug=False, use_reloader=False, threaded=True),
            daemon=True,
        )
        thread.start()
        self.get_logger().info(f"Web dashboard on http://<jetson-hostname>.local:{WEB_PORT}  (e.g. http://chuoi.local:{WEB_PORT})")


def _mjpeg_generator(node, side):
    import cv2
    node.get_logger().info(f"Dashboard: {side} video client connected")
    try:
        while True:
            frame = node.latest_frame_left if side == 'left' else node.latest_frame_right
            det = node.latest_detection_left if side == 'left' else node.latest_detection_right
            if frame is None:
                # No real frame yet -- send a placeholder instead of nothing, so the
                # browser actually renders something instead of spinning forever
                # waiting for the first byte. If you see this image, that camera's
                # pipeline itself isn't producing frames (check the "Camera frame
                # grab failing" warning in the terminal); if you never see even
                # this, the problem is the HTTP stream, not the camera.
                display = np.zeros((360, 640, 3), dtype=np.uint8)
                cv2.putText(display, f"waiting for {side} camera frame...", (20, 180),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)
            else:
                display = frame.copy()
                if det and (time.time() - det['t']) < 1.0:
                    p1 = (int(det['x1']), int(det['y1']))
                    p2 = (int(det['x2']), int(det['y2']))
                    cv2.rectangle(display, p1, p2, (0, 255, 0), 2)
                    cv2.putText(display, f"duck {det['conf']:.2f}", (p1[0], max(0, p1[1] - 8)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            ok, buf = cv2.imencode('.jpg', display)
            if ok:
                yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + buf.tobytes() + b'\r\n')
            time.sleep(0.1)
    finally:
        node.get_logger().info(f"Dashboard: {side} video client disconnected")


HTML_PAGE = """<!doctype html>
<html>
<head>
<meta charset="utf-8">
<title>Search and Rescue -- live</title>
<style>
  :root { color-scheme: dark; }
  body {
    background: #12151a; color: #e6e6e6;
    font-family: -apple-system, Segoe UI, Roboto, sans-serif;
    margin: 0; padding: 20px;
  }
  h1 { font-size: 1.2rem; font-weight: 600; margin: 0 0 16px; color: #9fd3ff; }
  .column { display: flex; flex-direction: column; gap: 16px; }
  .layout { display: grid; grid-template-columns: minmax(300px, 460px) minmax(300px, 1fr); gap: 16px; align-items: start; }
  @media (max-width: 860px) { .layout { grid-template-columns: 1fr; } }
  .panel {
    background: #1b1f27; border: 1px solid #2a2f3a; border-radius: 10px;
    padding: 14px;
  }
  .panel-title {
    font-size: 0.72rem; text-transform: uppercase; letter-spacing: 0.04em;
    color: #6b7688; margin-bottom: 10px; font-weight: 600;
  }
  canvas { background: #0e1116; border-radius: 6px; display: block; max-width: 100%; }
  .video-row { display: flex; gap: 10px; flex-wrap: wrap; }
  .video-cell { flex: 1 1 200px; }
  .video-cell img { width: 100%; border-radius: 6px; background: #000; display: block; }
  .video-cell .label { font-size: 0.7rem; color: #8a94a6; margin-bottom: 4px; }
  .status-line { font-family: ui-monospace, Menlo, Consolas, monospace; font-size: 0.85rem; line-height: 1.7; }
  .status-line b { color: #9fd3ff; }
  .stat-highlight {
    font-family: ui-monospace, Menlo, Consolas, monospace; font-size: 1.05rem;
    padding: 8px 10px; margin-top: 8px; border-radius: 8px;
    background: #10201c; border: 1px solid #1c4d2b; color: #9fe6ae;
  }
  .stat-highlight.stale { background: #201c10; border-color: #4a3f18; color: #8a94a6; }
  .badge {
    display: inline-block; padding: 2px 8px; border-radius: 999px; font-size: 0.75rem;
    background: #2a2f3a; margin-left: 6px;
  }
  .badge.done { background: #1c4d2b; color: #9fe6ae; }
  .badge.waiting { background: #4a3f18; color: #ffe08a; }
  .legend { font-size: 0.72rem; color: #8a94a6; margin-top: 8px; line-height: 1.5; }
  .report {
    margin-top: 10px; padding: 10px; border-radius: 8px; background: #2a2410;
    border: 1px solid #4a3f18; color: #ffe08a; font-family: ui-monospace, monospace; font-size: 0.85rem;
  }
  .controls { margin-bottom: 16px; }
  button {
    font-size: 0.95rem; font-weight: 600; padding: 10px 20px; border-radius: 8px;
    border: none; cursor: pointer; margin-right: 10px;
  }
  #startBtn { background: #1c4d2b; color: #9fe6ae; }
  #startBtn:disabled { background: #23282f; color: #555; cursor: not-allowed; }
  #stopBtn { background: #5a1c1c; color: #ffb3b3; }
  #stopBtn:disabled { background: #23282f; color: #555; cursor: not-allowed; }
  #resetBtn { background: #2a2f3a; color: #c8ccd4; }
  .pid-row { display: flex; gap: 12px; flex-wrap: wrap; align-items: flex-end; }
  .pid-row label { font-size: 0.72rem; color: #8a94a6; display: flex; flex-direction: column; gap: 4px; }
  .pid-row input {
    background: #0e1116; color: #e6e6e6; border: 1px solid #2a2f3a; border-radius: 6px;
    padding: 6px 8px; font-size: 0.9rem; width: 80px;
  }
  #pidPushBtn { background: #1c3a4d; color: #9fd3ff; padding: 8px 16px; }
</style>
</head>
<body>
  <h1>Search &amp; Rescue -- live dashboard</h1>
  <div class="controls">
    <button id="startBtn" onclick="sendCmd('/start')">Start Patrol</button>
    <button id="stopBtn" onclick="sendCmd('/stop')">Stop</button>
    <button id="resetBtn" onclick="sendCmd('/reset')">Reset All</button>
  </div>
  <div class="layout">
    <div class="column">
      <div class="panel">
        <div class="panel-title">Room map</div>
        <canvas id="map" width="150" height="335"></canvas>
        <div class="legend">
          solid box = room walls &middot; dashed box = patrol path &middot;
          <span style="color:#ff5b5b;">&#9679;</span> = current best duck estimate &middot;
          <span style="color:#ffd76b;">&#9679;</span> = individual sightings &middot;
          <span style="color:#9fd3ff;">&#9654;</span> = robot &middot;
          line = current target
        </div>
      </div>
      <div class="panel">
        <div class="panel-title">Robot / mission status</div>
        <div class="status-line" id="navStatus">connecting...</div>
      </div>
      <div class="panel">
        <div class="panel-title">ESP32 diagnostics (PID / encoder)</div>
        <div class="status-line" id="diagStatus" style="font-size:0.78rem;">connecting...</div>
      </div>
      <div class="panel">
        <div class="panel-title">PID tuning (saved on this Jetson, pushed to ESP32 live)</div>
        <div class="pid-row">
          <label>Kp <input id="pidKp" type="number" step="0.1"></label>
          <label>Ki <input id="pidKi" type="number" step="0.05"></label>
          <label>Max I contribution <input id="pidMaxI" type="number" step="5"></label>
          <label>Heading Kp (gyro) <input id="pidKhead" type="number" step="1"></label>
          <button id="pidPushBtn" onclick="pushPidGains()">Push to robot</button>
        </div>
        <div class="pid-row">
          <label>Trim left (PWM) <input id="pidTrimL" type="number" step="1"></label>
          <label>Trim right (PWM) <input id="pidTrimR" type="number" step="1"></label>
        </div>
        <div class="legend" id="pidStatus">
          re-pushed automatically every 2s, so an ESP32 reboot picks the saved values back up on its own.
          Trim = a flat PWM offset added on top of the PID output, for balancing a mechanically weaker motor -
          only applies while actually driving, never during a stop.
        </div>
      </div>
    </div>
    <div class="column">
      <div class="panel">
        <div class="panel-title">Camera feeds (rectified)</div>
        <div class="video-row">
          <div class="video-cell">
            <div class="label">LEFT camera</div>
            <img id="video-left" src="/video_feed_left">
          </div>
          <div class="video-cell">
            <div class="label">RIGHT camera</div>
            <img id="video-right" src="/video_feed_right">
          </div>
        </div>
      </div>
      <div class="panel">
        <div class="panel-title">Duck detection</div>
        <div class="status-line" id="duckStatus">connecting...</div>
        <div id="stereoBox"></div>
        <div id="reportBox"></div>
      </div>
    </div>
  </div>

<script>
const canvas = document.getElementById('map');
const ctx = canvas.getContext('2d');

const PX_PER_M = 150;  // fixed scale, same on both axes -- keeps circles round
                       // and distances visually comparable regardless of the
                       // room's aspect ratio, unlike sizing to a fixed square.

function draw(state) {
  // Canvas itself sized to the room's real proportions (setting width/height
  // clears it, which is fine -- redrawn fully every call anyway). Works for
  // any room shape without editing this page, not just square ones.
  canvas.width = Math.round(state.room_width * PX_PER_M);
  canvas.height = Math.round(state.room_length * PX_PER_M);
  const toScreen = (x, y) => [x * PX_PER_M, canvas.height - y * PX_PER_M];

  ctx.clearRect(0, 0, canvas.width, canvas.height);

  // grid
  ctx.strokeStyle = '#232833';
  ctx.lineWidth = 1;
  for (let g = 0; g <= state.room_width + 1e-6; g += state.grid_cell) {
    let [gx, ] = toScreen(g, 0);
    ctx.beginPath(); ctx.moveTo(gx, 0); ctx.lineTo(gx, canvas.height); ctx.stroke();
  }
  for (let g = 0; g <= state.room_length + 1e-6; g += state.grid_cell) {
    let [, gy] = toScreen(0, g);
    ctx.beginPath(); ctx.moveTo(0, gy); ctx.lineTo(canvas.width, gy); ctx.stroke();
  }

  // room boundary
  ctx.strokeStyle = '#4a5568';
  ctx.lineWidth = 2;
  ctx.strokeRect(0, 0, canvas.width, canvas.height);

  // waypoint path
  ctx.strokeStyle = '#3a5a7a';
  ctx.setLineDash([6, 5]);
  ctx.lineWidth = 2;
  ctx.beginPath();
  state.waypoints.forEach((wp, i) => {
    const [sx, sy] = toScreen(wp[0], wp[1]);
    if (i === 0) ctx.moveTo(sx, sy); else ctx.lineTo(sx, sy);
  });
  ctx.stroke();
  ctx.setLineDash([]);

  // waypoint markers
  state.waypoints.forEach((wp) => {
    const [sx, sy] = toScreen(wp[0], wp[1]);
    ctx.fillStyle = '#3a5a7a';
    ctx.beginPath(); ctx.arc(sx, sy, 4, 0, 7); ctx.fill();
  });

  // raw duck sightings -- faint, small: a cloud of noisy individual readings,
  // not meant to be read precisely on their own, just context for the estimate
  state.duck_sightings.forEach((p) => {
    const [sx, sy] = toScreen(p[0], p[1]);
    ctx.fillStyle = 'rgba(255, 215, 107, 0.35)';
    ctx.beginPath(); ctx.arc(sx, sy, 3, 0, 7); ctx.fill();
  });

  // running average of all sightings so far -- THE answer, updates live the
  // whole time (not just once the loop finishes)
  if (state.duck_estimate) {
    const [sx, sy] = toScreen(state.duck_estimate.x, state.duck_estimate.y);
    ctx.fillStyle = '#ff5b5b';
    ctx.beginPath(); ctx.arc(sx, sy, 6, 0, 7); ctx.fill();
    ctx.strokeStyle = '#ff5b5b'; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.arc(sx, sy, 11, 0, 7); ctx.stroke();
  }

  // line from robot to the waypoint it's currently driving toward
  if (state.target_waypoint && !state.done) {
    const [tx, ty] = toScreen(state.target_waypoint[0], state.target_waypoint[1]);
    const [ox, oy] = toScreen(state.x, state.y);
    ctx.strokeStyle = 'rgba(159, 211, 255, 0.5)';
    ctx.lineWidth = 1.5;
    ctx.setLineDash([3, 4]);
    ctx.beginPath(); ctx.moveTo(ox, oy); ctx.lineTo(tx, ty); ctx.stroke();
    ctx.setLineDash([]);
    ctx.fillStyle = '#9fd3ff';
    ctx.beginPath(); ctx.arc(tx, ty, 6, 0, 7); ctx.fill();
  }

  // robot (triangle pointing in facing direction)
  const [rx, ry] = toScreen(state.x, state.y);
  const theta = state.theta_deg * Math.PI / 180;
  ctx.save();
  ctx.translate(rx, ry);
  ctx.rotate(-theta);  // canvas y is flipped vs. room y, so flip rotation too
  ctx.fillStyle = state.done ? '#9fe6ae' : '#9fd3ff';
  ctx.beginPath();
  ctx.moveTo(10, 0); ctx.lineTo(-6, 6); ctx.lineTo(-6, -6); ctx.closePath();
  ctx.fill();
  ctx.restore();
}

function sendCmd(path) {
  fetch(path, { method: 'POST' }).catch(() => {});
}

let pidFieldsInitialized = false;  // only fill the inputs once from the server --
                                    // otherwise the 400ms poll would overwrite
                                    // whatever vịt is mid-typing

function pushPidGains() {
  const kp = parseFloat(document.getElementById('pidKp').value);
  const ki = parseFloat(document.getElementById('pidKi').value);
  const max_i = parseFloat(document.getElementById('pidMaxI').value);
  const khead = parseFloat(document.getElementById('pidKhead').value);
  const trim_left = parseFloat(document.getElementById('pidTrimL').value);
  const trim_right = parseFloat(document.getElementById('pidTrimR').value);
  if (!isFinite(kp) || !isFinite(ki) || !isFinite(max_i) || !isFinite(khead) ||
      !isFinite(trim_left) || !isFinite(trim_right)) {
    document.getElementById('pidStatus').innerText = 'enter valid numbers in all six fields first';
    return;
  }
  fetch('/pid_gains', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ kp, ki, max_i, khead, trim_left, trim_right }),
  }).then(r => r.json()).then(res => {
    document.getElementById('pidStatus').innerText = res.ok
      ? `saved + pushed at ${new Date().toLocaleTimeString()} -- watch the diag panel above for KP/KI/MAXI/KHEAD/TRIML/TRIMR to confirm the ESP32 picked it up`
      : (res.error || 'push failed');
  }).catch(() => {
    document.getElementById('pidStatus').innerText = 'push failed (dashboard unreachable?)';
  });
}

function poll() {
  fetch('/state').then(r => r.json()).then(state => {
    draw(state);

    let statusBadge;
    if (state.done) statusBadge = '<span class="badge done">LOOP DONE</span>';
    else if (state.started) statusBadge = '<span class="badge">patrolling</span>';
    else statusBadge = '<span class="badge waiting">waiting for Start</span>';

    const odomBadge = state.have_odom
      ? '<span class="badge done">odom OK</span>'
      : '<span class="badge" style="background:#5a1c1c;color:#ffb3b3;">NO /odom -- check micro-ROS agent</span>';

    document.getElementById('navStatus').innerHTML =
      `pos: <b>(${state.x.toFixed(2)}, ${state.y.toFixed(2)})</b> ` +
      `heading: <b>${state.theta_deg.toFixed(1)}&deg;</b> ${odomBadge}<br>` +
      `waypoint: <b>${state.waypoint_idx}/${state.waypoints.length}</b> ${statusBadge}`;

    document.getElementById('diagStatus').innerText =
      state.esp32_diag || '(nothing received yet from /esp32_diag)';

    if (!pidFieldsInitialized && state.pid_gains) {
      document.getElementById('pidKp').value = state.pid_gains.kp;
      document.getElementById('pidKi').value = state.pid_gains.ki;
      document.getElementById('pidMaxI').value = state.pid_gains.max_i;
      document.getElementById('pidKhead').value = state.pid_gains.khead;
      document.getElementById('pidTrimL').value = state.pid_gains.trim_left;
      document.getElementById('pidTrimR').value = state.pid_gains.trim_right;
      pidFieldsInitialized = true;
    }

    const stereoFresh = state.latest_stereo && (Date.now() / 1000 - state.latest_stereo.t) < 2.0;
    document.getElementById('duckStatus').innerHTML =
      `camera: <b>${state.has_camera ? 'on' : 'off (--nav-only)'}</b><br>` +
      `duck sightings recorded: <b>${state.duck_sightings.length}</b>`;

    const stereoBox = document.getElementById('stereoBox');
    if (stereoFresh) {
      const smoothed = state.stereo_distance_smoothed_m;
      const smoothedTxt = smoothed !== null
        ? ` / <b>${smoothed.toFixed(2)}m</b> smoothed (median of last ${state.stereo_smoothing_window})`
        : '';
      stereoBox.innerHTML = `<div class="stat-highlight">distance to duck now: ` +
        `<b>${state.latest_stereo.distance_m.toFixed(2)}m</b> raw${smoothedTxt} ` +
        `@ ${state.latest_stereo.bearing_deg.toFixed(1)}&deg; ` +
        `<span style="color:#8a94a6;">(disparity ${state.latest_stereo.disparity_px.toFixed(0)}px)</span></div>`;
    } else {
      stereoBox.innerHTML = `<div class="stat-highlight stale">no current stereo lock ` +
        `(duck not seen by both cameras right now)</div>`;
    }

    document.getElementById('startBtn').disabled = state.started || state.done;
    document.getElementById('stopBtn').disabled = !state.started;

    const reportBox = document.getElementById('reportBox');
    if (state.report) {
      if (state.report.sightings > 0) {
        reportBox.innerHTML = `<div class="report">REPORT<br>duck at (${state.report.avg_x.toFixed(2)}, ${state.report.avg_y.toFixed(2)})<br>grid cell (${state.report.cell_x}, ${state.report.cell_y})<br>from ${state.report.sightings} sighting(s)</div>`;
      } else {
        reportBox.innerHTML = `<div class="report">REPORT<br>loop complete, no duck sighted</div>`;
      }
    } else {
      reportBox.innerHTML = '';
    }
  }).catch(() => {
    document.getElementById('navStatus').innerText = 'connection lost, retrying...';
  });
}

setInterval(poll, 400);
poll();
</script>
</body>
</html>
"""


def _print_git_info():
    """Prints which branch/commit is actually running, first thing on startup --
    added 2026-08-27 after a real incident where the Jetson had been left
    checked out on a teammate's feature branch (motor/PID work), and the
    resulting "why doesn't the dashboard show distance" investigation cost
    real time before anyone thought to check `git status` by hand. Two
    people share this one physical Jetson and switch branches on it for
    hardware-dependent testing -- that's normal and fine, but which branch
    is active needs to be visible without remembering to check, not
    something you only discover after debugging symptoms."""
    repo_dir = Path(__file__).resolve().parents[2]
    try:
        branch = subprocess.check_output(
            ['git', 'rev-parse', '--abbrev-ref', 'HEAD'], cwd=repo_dir, stderr=subprocess.DEVNULL
        ).decode().strip()
        commit = subprocess.check_output(
            ['git', 'rev-parse', '--short', 'HEAD'], cwd=repo_dir, stderr=subprocess.DEVNULL
        ).decode().strip()
        dirty = subprocess.run(
            ['git', 'diff', '--quiet'], cwd=repo_dir, stderr=subprocess.DEVNULL
        ).returncode != 0
        print(f"[git] running from branch '{branch}' @ {commit}{' (uncommitted local changes)' if dirty else ''}")
    except Exception as e:
        print(f"[git] could not determine git branch/commit: {e}")


def main():
    _print_git_info()
    parser = argparse.ArgumentParser()
    parser.add_argument('--nav-only', action='store_true')
    args = parser.parse_args()

    rclpy.init()
    node = SearchAndRescue(nav_only=args.nav_only)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
