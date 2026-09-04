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
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from visualization_msgs.msg import Marker

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

# Duck's real physical height, measured 2026-08-24 (same constant the old
# monocular backup script used as DUCK_HEIGHT_M -- see
# search_and_rescue_monocular_backup.py).
DUCK_HEIGHT_M = 0.13

# Size-sanity gate (2026-08-27): given distance + box height in pixels, the
# implied real-world height (pinhole projection: real = px * distance / f)
# should land near DUCK_HEIGHT_M. Catches the class of bad reading seen live
# where disparity/distance come out wildly wrong (e.g. 0.13-0.15m readings
# from ~500-590px disparity, right in the known-unreliable <13cm zone) --
# those imply a real height nowhere near an actual duck's. Root-causes the
# false-positive/bad-match problem without touching the model or retraining
# (see project memory: retraining risks whack-a-mole regressions; this is a
# pure additive filter, trivially disabled by setting ENABLE_DUCK_SIZE_CHECK
# to False if it turns out to reject good sightings instead).
# Tolerance deliberately wide and asymmetric, not a tight fit: a partially-
# occluded detection (e.g. only the duck's head boxed, seen live 2026-08-27)
# under-reports height, so the floor is generous (0.4x) to avoid rejecting
# real partial detections; ceiling (2.5x) is looser than the floor because
# an oversized box from a bad L/R pairing has no such legitimate case to
# protect against. NOT validated against real logged data yet (the box
# height wasn't in the log format that surfaced this idea) -- the
# 'implied_height_m' field added to the log line below is specifically so
# the next live run can be checked before trusting this gate blindly.
ENABLE_DUCK_SIZE_CHECK = True
DUCK_SIZE_MIN_RATIO = 0.4
DUCK_SIZE_MAX_RATIO = 2.5

# Point-cloud 3D position estimate (2026-08-27 night, experimental -- built
# while vịt was away, NOT yet tested against real hardware since Jetson
# network access was down this session. See PR description before trusting
# this live.) Runs a SECOND, independent distance/position estimate
# alongside the existing single-disparity centroid method above, specifically
# to compare accuracy between the two (vịt's ask: get error down toward
# 1-2%, which the single-point centroid method structurally can't reach --
# see project memory ruler benchmark, 6-10% error, worsening with range
# because distance=fx*baseline/disparity is 1/x and a single box-centroid
# disparity has no averaging to reject jitter). This method instead runs
# dense stereo matching (cv2.StereoSGBM) on the FULL rectified frame pair
# (same self.latest_frame_left/right the dashboard already shows), reprojects
# to a real 3D point cloud (cv2.reprojectImageTo3D), and takes the MEDIAN of
# just the points falling inside the duck's already-detected box -- many
# points averaged (robust to per-pixel noise) instead of one disparity value
# from two box centroids (fragile to either box being off by a few px).
#
# Deliberately NOT using the existing Isaac ROS stereo_image_proc pipeline
# (jetson/slam/stereo_depth_argus.launch.py) for this even though it already
# does dense stereo -- that needs the full Isaac ROS Docker container +
# rectified visual_slam launch running, which claims the cameras exclusively
# and can't run alongside this script's own camera capture (see module
# docstring's existing "Deliberately does NOT use ... Isaac ROS stereo
# depth" reasoning, same logic extends here). Runs full-frame (not cropped
# to the box before matching) specifically to avoid a subtle correctness
# bug: cropping the disparity input by rows shifts the image's effective
# principal point, which cv2.reprojectImageTo3D's Q matrix doesn't know
# about unless Q itself is adjusted to match -- easy to get quietly wrong.
# Cropping the OUTPUT point cloud to the box instead (after reprojecting
# the full frame) needs no such correction. Isaac's own stereo_image_proc
# equivalent was independently measured at ~1-1.5Hz on this same class of
# Jetson CPU workload (see stereo_depth_argus.launch.py's 2026-08-11 note)
# -- POINTCLOUD_ESTIMATE_PERIOD_S below is set to roughly match that
# precedent rather than guessing blind, but the real achieved rate is
# logged every time this runs so it can be corrected against real data.
ENABLE_POINTCLOUD_ESTIMATE = True
POINTCLOUD_ESTIMATE_PERIOD_S = 1.0     # rate limit -- dense stereo is much
                                        # more expensive than the existing
                                        # centroid method, don't run it every
                                        # vision-loop tick
POINTCLOUD_MIN_VALID_POINTS = 20       # reject the reading if fewer than
                                        # this many pixels inside the box
                                        # produced a valid (finite, in-range)
                                        # depth -- mirrors the existing
                                        # disparity/distance/size sanity
                                        # gates above rather than trusting a
                                        # near-empty region's median

# Sanity gate vs. the centroid method (2026-09-04). Live-diagnosed at range
# (1.0-1.3m, see COMMIT_HISTORY.md): core/full-box can lock onto background
# right at the box edges instead of the duck (the duck's own smooth surface
# gives SGBM too little texture to match confidently there) -- both
# adaptive core-region sizing AND the plain 25% fixed margin were tested
# live and both still produce this, since it's a texture problem, not a
# crop-size problem. The centroid method (single disparity across the whole
# box, no dense per-pixel matching) was accurate at every distance tested
# this session (~1% error at 1.3m). If this method's distance disagrees
# with centroid by more than this fraction, rescale this reading's
# magnitude onto centroid's distance -- keeps this method's direction
# (lateral position within the box is far less likely to be wrong than its
# depth) but not a magnitude that likely came from the wrong object.
POINTCLOUD_CENTROID_MAX_DISAGREEMENT_FRAC = 0.30

# Diagnostic-only inset fraction (2026-08-29) -- see the "edge bleeding"
# hypothesis comment in _estimate_pointcloud_position. Shrinks each side of
# the box by this fraction before computing a SECOND, comparison-only
# median, testing whether background pixels near the box edges are
# dragging the full-box distance farther than the truth. Does not change
# the actual reported distance_m.
POINTCLOUD_EROSION_MARGIN_FRAC = 0.25

# How many recent sightings' SPHERE markers to keep on screen at once
# (2026-08-28) -- each sighting used to overwrite id=0, so RViz2 only ever
# showed the single latest median point, with no way to visually judge
# repeatability/precision across readings (vịt's actual ask: use RViz to
# see WHERE accuracy is weak, not just read one number). Cycling through
# this many distinct marker ids instead makes recent sightings persist
# simultaneously as a visible scatter -- tight cluster = stable/trustworthy,
# spread out = real per-reading noise, exactly the kind of pattern a single
# webpage number can't show.
POINTCLOUD_MARKER_HISTORY = 20

# Camera mount offset from base_link -- copied verbatim from
# jetson/slam/visual_slam-adjacent jetson/slam/stereo_depth_argus.launch.py
# (measured there 2026-08-11, tape measure against the real chassis: camera
# sits ~directly over the wheel axle, centered left-right, 14cm up). NOT the
# rotation from that file (BASE_TO_LEFT_CAM_QUAT) -- this mission's rig has
# since been yawed 45deg (CAMERA_BEARING_OFFSET_RAD), so the rotation is
# derived fresh in _estimate_pointcloud_position instead of reused. This
# XYZ offset is carried over on the assumption that yawing the rig about its
# own mount point doesn't change its height/centering -- plausible but NOT
# independently re-measured for this specific rig.
BASE_TO_LEFT_CAM_XYZ = (0.0, 0.0, 0.14)

# Fixed optical-frame axis-convention rotation (X=right,Y=down,Z=forward ->
# x=forward,y=left,z=up), as a quaternion -- same one stereo_depth_argus.
# launch.py's BASE_TO_LEFT_CAM_QUAT encodes for an UNYAWED mount. Kept
# separate here (not reused as a single fixed quaternion) because this
# mission's rig IS yawed (CAMERA_BEARING_OFFSET_RAD) -- see _quat_mul below,
# composed fresh at runtime instead of hand-computing one composed constant,
# to avoid a silent quaternion-math arithmetic error with no way to check it
# tonight (same reasoning as the explicit-trig position math above).
_OPTICAL_FRAME_FIX_QUAT_XYZW = (-0.5, 0.5, -0.5, 0.5)


def _quat_mul(q1, q2):
    """Hamilton product, (x,y,z,w) order -- q1 applied after q2 (q1*q2*v*q2^-1*q1^-1)."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )

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

# Manual drive/turn tester (2026-08-29) -- lets Alex "pretend to be the
# Jetson" and send one isolated primitive at a time (drive N meters, turn
# N degrees), completely outside the waypoint patrol -- for tuning the
# ESP32 PID against controlled, repeatable motions, and for later checking
# raw odometry accuracy (commanded distance/angle vs. what /odom reports).
# "Turn" deliberately commands pure angular.z with linear.x=0 (in-place
# spin, not a pivot) -- decouples heading change from position change,
# same reasoning discussed with Alex for why in-place spin is the right
# primitive here. "Drive forward" commands pure linear.x with angular.z=0
# -- holding straight is entirely the ESP32's own gyro-based heading loop
# doing its job; this script doesn't add any steering correction of its
# own for this test mode, unlike _navigate_tick's continuous pursuit.
MANUAL_MOVE_TIMEOUT_S = 30.0   # safety cutoff -- auto-stop if a target is
                                # somehow never reached (e.g. odom stuck)

# Manual turn ramp-down (2026-08-27) -- the manual turn primitive used to
# command a constant MAX_ANGULAR_SPEED right up until the target was crossed,
# then cut to 0 (bang-bang). At 1.5 rad/s (~86deg/s) the combined lag of
# /odom's 10Hz publish, this node's own 10Hz control_tick, and the motors
# coasting (no active brake -- PWM=0 just stops driving them, it doesn't stop
# them) turned into pure overshoot: confirmed live, a commanded 90deg turn was
# landing around 144deg. _navigate_tick already avoids this for patrol driving
# by commanding angular.z proportional to heading error (KP_HEADING) instead
# of a constant -- same fix applied here. Ramping down as the remaining angle
# shrinks means the robot is moving slowly by the time it actually crosses the
# target, so that same lag becomes a small error instead of tens of degrees.
KP_TURN = 2.0                    # rad/s commanded per rad of remaining angle
MIN_TURN_ANGULAR_SPEED = 0.3     # rad/s -- floor so the P term doesn't decay
                                  # below what the motors can actually turn at
                                  # (deadband/static friction) and stall short
TURN_DONE_TOLERANCE = math.radians(2.0)  # good enough -- don't chase the
                                          # last fraction of a degree forever

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
# Must match motor_f1.c's g_kp/g_ki/g_max_i_contribution/g_kheading/
# g_max_heading_trim_rpm/g_trim_left/g_trim_right defaults -- these are only
# the fallback used before any dashboard save has ever happened (or if
# pid_gains.json is missing/corrupt).
DEFAULT_PID_GAINS = {'kp': 3.0, 'ki': 0.2, 'max_i': 40.0, 'khead': 15.0,
                      'max_head_trim': 20.0, 'trim_left': 0.0, 'trim_right': 0.0,
                      'klock': 2.0, 'max_lock': 0.6}


def load_pid_gains():
    try:
        with open(PID_GAINS_PATH) as f:
            data = json.load(f)
        return {
            'kp': float(data['kp']), 'ki': float(data['ki']),
            'max_i': float(data['max_i']), 'khead': float(data['khead']),
            'max_head_trim': float(data['max_head_trim']),
            'trim_left': float(data['trim_left']), 'trim_right': float(data['trim_right']),
            # .get(), not data[...] -- an existing pid_gains.json saved before
            # heading-lock existed won't have these keys; fall back to the
            # defaults for just these two instead of resetting every gain.
            'klock': float(data.get('klock', DEFAULT_PID_GAINS['klock'])),
            'max_lock': float(data.get('max_lock', DEFAULT_PID_GAINS['max_lock'])),
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


def _median_xyz_distance(region_pts, mask):
    """Median X/Y/Z (camera_optical convention) + Euclidean distance over
    the masked points of a reprojectImageTo3D region. Shared by the
    full-box and core-region computations in _estimate_pointcloud_position
    so a future change to the distance formula only has to happen once --
    2026-08-29 code review flagged the previous copy-pasted version of
    this as a real drift risk (exactly the class of bug that made the
    core-region fix's own debug point cloud go stale, see the
    used_region_pts/used_valid_mask comment below)."""
    x = float(np.median(region_pts[:, :, 0][mask]))
    y = float(np.median(region_pts[:, :, 1][mask]))
    z = float(np.median(region_pts[:, :, 2][mask]))
    return x, y, z, math.sqrt(x ** 2 + y ** 2 + z ** 2)


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

        self.motor_killed = False  # what we last ASKED the ESP32 for -- see
                                    # set_motor_killed and motor_f1.c's
                                    # MOTOR_OFF/MOTOR_ON sentinels. Drives the
                                    # 2s periodic re-assert in
                                    # _publish_pid_gains, NOT what the
                                    # dashboard displays (see motor_confirmed_
                                    # killed / on_diag for that - ground
                                    # truth from the ESP32 itself).
        self.motor_confirmed_killed = False  # what the ESP32 itself last
                                              # reported via /esp32_diag's
                                              # MOTOR=on/off field

        self.manual_move = None  # None, or a dict describing an in-progress
                                  # manual drive/turn test -- see MANUAL_MOVE_
                                  # TIMEOUT_S comment and _manual_move_tick.
                                  # Mutually exclusive with patrol (self.started) --
                                  # enforced at the /manual_move and /start routes,
                                  # not here, so there's one obvious place each
                                  # guard lives rather than duplicated checks.

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
        self.latest_pointcloud_reading = None  # {'x_m','y_m','z_m','distance_m','world_x','world_y',
                                                # 'world_z','n_points','compute_ms','t'} -- see
                                                # ENABLE_POINTCLOUD_ESTIMATE comment above. Independent
                                                # of latest_stereo_reading, meant to be compared
                                                # against it, not to replace it (yet).
        self._last_pointcloud_estimate_t = 0.0  # rate-limit gate, see POINTCLOUD_ESTIMATE_PERIOD_S
        self._pointcloud_marker_id_counter = 0  # cycles 0..POINTCLOUD_MARKER_HISTORY-1, see that constant

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pid_pub = self.create_publisher(String, '/pid_gains', 10)
        # odom->base_link TF, broadcast here directly (2026-08-28) instead of
        # needing the separate jetson/nav2/odom_to_tf.py node running --
        # this script already tracks self.x/y/theta from the same /odom
        # messages, so it's the same data, just also published as real TF.
        # Added specifically so RViz2 has SOMETHING that registers the
        # "odom" frame -- with zero TF ever published, RViz's Fixed Frame
        # can't be set to "odom" at all ("Frame [odom] does not exist"),
        # even though Marker messages already carry frame_id="odom" --
        # message frame_id alone doesn't register a frame with tf2, only
        # actual /tf publications do.
        self.tf_broadcaster = TransformBroadcaster(self)
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
        # Q captured (was discarded as `_`) -- needed by cv2.reprojectImageTo3D
        # for the point-cloud position estimate, see ENABLE_POINTCLOUD_ESTIMATE.
        R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
            K1, D1, K2, D2, size, R, T,
            flags=cv2.CALIB_ZERO_DISPARITY, alpha=0,
        )
        self.Q = Q
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

        if ENABLE_POINTCLOUD_ESTIMATE:
            # Parameters are the well-known OpenCV stereo_match.py sample
            # defaults, NOT tuned for this specific rig/scene yet -- a
            # reasonable untested starting point, not a validated choice.
            # numDisparities RAISED 192->384 (2026-08-29, real bug found via
            # ruler benchmark): a duck at 0.30m needs disparity ~= fx*baseline
            # /0.30 = 875*0.085/0.30 ~= 248px, which the old 192 cap couldn't
            # search at all -- confirmed by back-solving the wrong 0.43m
            # reading that came out instead: 875*0.085/0.43 ~= 173px, which
            # DOES fall inside [0,192), meaning SGBM was silently finding a
            # spurious in-range match instead of the true out-of-range one.
            # 384 covers down to ~19cm with margin; real compute cost scales
            # with this value (already ~1000-1100ms at 192), expect it to
            # rise meaningfully -- watch the logged compute_ms after this
            # change, don't assume the old ~1s estimate still holds.
            self.stereo_matcher = cv2.StereoSGBM_create(
                minDisparity=0,
                numDisparities=384,
                blockSize=7,
                P1=8 * 3 * 7 ** 2,
                P2=32 * 3 * 7 ** 2,
                disp12MaxDiff=1,
                uniquenessRatio=10,
                speckleWindowSize=100,
                speckleRange=2,
            )
            self.duck_marker_pub = self.create_publisher(Marker, '/duck_position_marker', 10)
            self.duck_cloud_pub = self.create_publisher(PointCloud2, '/duck_pointcloud_region', 10)

            # Static base_link -> camera_optical TF (2026-08-28) -- lets
            # RViz2 place the raw point-cloud debug view (published in
            # camera_optical frame below) correctly relative to odom/
            # base_link, instead of only ever seeing the single reduced-to-
            # one-point Marker. Composed at runtime via _quat_mul rather
            # than hand-computing one constant quaternion -- see that
            # function's comment for why.
            q_yaw = (0.0, 0.0, math.sin(CAMERA_BEARING_OFFSET_RAD / 2), math.cos(CAMERA_BEARING_OFFSET_RAD / 2))
            q_cam = _quat_mul(q_yaw, _OPTICAL_FRAME_FIX_QUAT_XYZW)
            static_tf = TransformStamped()
            static_tf.header.stamp = self.get_clock().now().to_msg()
            static_tf.header.frame_id = 'base_link'
            static_tf.child_frame_id = 'camera_optical'
            static_tf.transform.translation.x = BASE_TO_LEFT_CAM_XYZ[0]
            static_tf.transform.translation.y = BASE_TO_LEFT_CAM_XYZ[1]
            static_tf.transform.translation.z = BASE_TO_LEFT_CAM_XYZ[2]
            static_tf.transform.rotation.x = q_cam[0]
            static_tf.transform.rotation.y = q_cam[1]
            static_tf.transform.rotation.z = q_cam[2]
            static_tf.transform.rotation.w = q_cam[3]
            self.static_tf_broadcaster = StaticTransformBroadcaster(self)
            self.static_tf_broadcaster.sendTransform(static_tf)
            # Also re-sent periodically over the REGULAR (non-static)
            # broadcaster (2026-08-29) -- found live: a one-time-only
            # publish relies on /tf_static's transient-local QoS correctly
            # delivering to a subscriber that joins later (any RViz2 session
            # started after this node), and that didn't reliably happen --
            # confirmed via RViz2's own terminal log spamming "Message
            # Filter dropping message: frame 'camera_optical' ... queue is
            # full", the exact same symptom the missing odom->base_link TF
            # caused earlier, just for this frame instead. Re-publishing
            # every 2s (matches PID_GAINS_REPUBLISH_PERIOD_S's existing
            # "keep re-asserting, don't trust delivery once" pattern in this
            # file) means any RViz2 session picks it up within a couple
            # seconds of connecting, regardless of when it was launched.
            self._camera_static_tf = static_tf
            self.create_timer(2.0, self._republish_camera_tf)

            self.get_logger().info(
                "Point-cloud position estimate ENABLED (experimental, unverified against "
                "real hardware as of 2026-08-27 -- Jetson was offline when this was built)."
            )

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

        # Same pattern as jetson/nav2/odom_to_tf.py -- inlined here so RViz2
        # gets a real odom->base_link TF without needing that separate node.
        #
        # Real bug found 2026-08-29: msg.header.stamp is ALWAYS sec=0/
        # nanosec=0 (confirmed via `ros2 topic echo /odom --field
        # header.stamp` -- every single message, not intermittent) because
        # the ESP32/micro-ROS firmware never sets a real timestamp on the
        # Odometry message it publishes. Blindly copying that zero stamp
        # into this TF broadcast pinned odom->base_link at "time 0" forever
        # in tf2's buffer -- `x`/`y`/`theta` kept updating fine (this
        # callback still runs every message), but any TF lookup AT THE
        # CURRENT time (tf2_echo showed "Lookup would require extrapolation
        # into the future") failed. This is why the sphere Markers (placed
        # directly in the 'odom' frame via already-computed world_x/world_y,
        # no tf2 lookup needed) kept rendering fine while /duck_pointcloud_
        # region (a real PointCloud2, which RViz2 must tf2-transform into
        # the Fixed Frame at the message's own stamp) silently never
        # rendered -- not a rendering/size/QoS issue, a stale-TF-buffer one.
        # Fixed: stamp with the Jetson's own wall clock instead, same
        # pattern _republish_camera_tf already uses below.
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = msg.header.frame_id       # "odom"
        t.child_frame_id = msg.child_frame_id         # "base_link"
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

    def _republish_camera_tf(self):
        """Re-sends the base_link->camera_optical transform every 2s over
        the regular broadcaster -- see the comment where self._camera_
        static_tf is built in _init_vision for why a one-time /tf_static
        publish alone wasn't reliably reaching RViz2 sessions started later."""
        self._camera_static_tf.header.stamp = self.get_clock().now().to_msg()
        self.tf_broadcaster.sendTransform(self._camera_static_tf)

    def on_diag(self, msg: String):
        self.esp32_diag = msg.data
        # Ground truth for the motor kill switch (2026-08-28 fix): the
        # dashboard used to show self.motor_killed, which is just "what we
        # last asked for" - set the instant the button is clicked, with no
        # confirmation the ESP32 ever actually got the message. For a safety
        # switch that's not good enough (a single dropped fire-and-forget
        # publish would show "OFF" while the robot is still fully powered).
        # Parse the ESP32's own MOTOR=on/off out of its diagnostic line
        # instead - this only flips once the firmware itself reports it.
        for token in msg.data.split():
            if token.startswith('MOTOR='):
                self.motor_confirmed_killed = (token[len('MOTOR='):] == 'off')
                break

    # -- PID tuning -----------------------------------------------------------

    def _publish_pid_gains(self):
        msg = String()
        g = self.pid_gains
        msg.data = (f"KP={g['kp']:.4f},KI={g['ki']:.4f},MAXI={g['max_i']:.4f},KHEAD={g['khead']:.4f},"
                    f"TRIML={g['trim_left']:.4f},TRIMR={g['trim_right']:.4f},MAXHEAD={g['max_head_trim']:.4f},"
                    f"KLOCK={g['klock']:.4f},MAXLOCK={g['max_lock']:.4f}")
        self.pid_pub.publish(msg)
        if self.motor_killed:
            # Re-assert on the same 2s cadence as the gains above, for the
            # same reason: the ESP32 resets g_motor_killed to False on
            # reboot, and a silent motor re-enable would defeat the whole
            # point of this switch.
            kill_msg = String()
            kill_msg.data = "MOTOR_OFF"
            self.pid_pub.publish(kill_msg)

    def set_motor_killed(self, killed: bool):
        """Live motor kill switch - pulls the TB6612FNG's STBY pin instead
        of just commanding zero speed, so it's a real power cut the PID/
        heading loops can't fight (see motor_f1.c's MOTOR_OFF/MOTOR_ON
        sentinels on this same /pid_gains channel).

        Fires the command 5x over ~100ms rather than once (2026-08-28 fix) -
        this is a fire-and-forget publish over a serial link, and a single
        dropped message on a safety switch is a real problem, not just an
        inconvenience. The dashboard doesn't take this call's word for it
        either - see motor_confirmed_killed / on_diag, which only flips once
        the ESP32 itself reports the change back."""
        self.motor_killed = killed
        msg = String()
        msg.data = "MOTOR_OFF" if killed else "MOTOR_ON"
        for _ in range(5):
            self.pid_pub.publish(msg)
            time.sleep(0.02)

    def set_pid_gains(self, kp: float, ki: float, max_i: float, khead: float,
                       trim_left: float, trim_right: float, max_head_trim: float,
                       klock: float, max_lock: float):
        self.pid_gains = {'kp': kp, 'ki': ki, 'max_i': max_i, 'khead': khead,
                           'trim_left': trim_left, 'trim_right': trim_right,
                           'max_head_trim': max_head_trim,
                           'klock': klock, 'max_lock': max_lock}
        save_pid_gains(self.pid_gains)
        self._publish_pid_gains()

    def reset_odom(self):
        """Tells the ESP32 to zero its own (x,y,theta) - see motor_f1.c's
        pid_gains_callback RESET_ODOM sentinel. Reuses the /pid_gains channel
        rather than adding a new topic, same reasoning as the gains format
        itself."""
        msg = String()
        msg.data = "RESET_ODOM"
        self.pid_pub.publish(msg)

    # -- main loop ------------------------------------------------------------

    def control_tick(self):
        if not self.have_odom:
            return  # wait for first /odom message before doing anything

        # Manual move takes priority over patrol -- checked first,
        # unconditionally, so it works whether or not Start was ever
        # clicked (the whole point: test motions without running a mission).
        # Starting one while patrol is active is refused at the route level
        # (see /manual_move), so in practice these two never overlap.
        if self.manual_move is not None:
            self._manual_move_tick()
            return

        # Camera grab + detection run on their own background thread (see
        # _vision_loop) -- this callback is navigation-only now, so a slow
        # detection frame can never delay a cmd_vel update.
        if self.done or not self.started:
            self.cmd_pub.publish(Twist())  # stay stopped -- either finished, or waiting for Start
            return

        self._navigate_tick()

    # -- manual drive/turn tester ---------------------------------------------

    def _manual_move_tick(self):
        m = self.manual_move
        if time.time() - m['start_t'] > MANUAL_MOVE_TIMEOUT_S:
            self.get_logger().warn(
                f"Manual {m['type']} move timed out after {MANUAL_MOVE_TIMEOUT_S:.0f}s -- "
                "stopping, target not reached (odometry stuck? PID not converging?)"
            )
            self.cmd_pub.publish(Twist())
            self.manual_move = None
            return

        cmd = Twist()
        if m['type'] == 'distance':
            # Straight-line distance from where this move started -- valid as
            # long as the robot is actually driving straight (angular.z=0
            # commanded below, held by the ESP32's own gyro heading loop) -
            # if it wanders, this would under-count real distance traveled,
            # which is itself a useful signal something's off with heading
            # hold, not just a measurement quirk to ignore.
            traveled = math.hypot(self.x - m['start_x'], self.y - m['start_y'])
            target = abs(m['value'])
            if traveled >= target:
                self.cmd_pub.publish(Twist())
                self.get_logger().info(f"Manual move done: drove {traveled:.3f}m (target {target:.3f}m)")
                self.manual_move = None
                return
            cmd.linear.x = MAX_LINEAR_SPEED if m['value'] >= 0 else -MAX_LINEAR_SPEED
            cmd.angular.z = 0.0
        else:  # 'turn'
            # Accumulate small per-tick deltas rather than one wrapped
            # comparison against the start heading -- correctly handles any
            # target angle (including >180deg) without the +/-180deg wrap
            # boundary causing a bogus "done" or "never done" result.
            delta = wrap_angle(self.theta - m['last_theta'])
            m['last_theta'] = self.theta
            m['total_rotated'] += delta
            # Signed remaining angle (not abs()) -- shrinks from m['value']
            # toward 0 as the turn progresses, same shape as _navigate_tick's
            # heading_error, so the same clamp(KP * error) pattern applies.
            remaining = m['value'] - m['total_rotated']
            if abs(remaining) <= TURN_DONE_TOLERANCE:
                self.cmd_pub.publish(Twist())
                self.get_logger().info(
                    f"Manual move done: turned {math.degrees(m['total_rotated']):.1f}deg "
                    f"(target {math.degrees(m['value']):.1f}deg)"
                )
                self.manual_move = None
                return
            # Pure in-place spin: angular.z only, linear.x=0 - see the
            # MANUAL_MOVE_TIMEOUT_S comment above for why this primitive
            # (not a pivot-on-one-wheel curve) is the deliberate choice.
            # Positive value = positive angular.z = left turn, same
            # convention as everywhere else in this file/the ESP32 firmware.
            # Proportional ramp-down (see KP_TURN comment above), floored at
            # MIN_TURN_ANGULAR_SPEED so it doesn't stall out approaching zero.
            speed = KP_TURN * remaining
            if speed >= 0:
                speed = max(MIN_TURN_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, speed))
            else:
                speed = min(-MIN_TURN_ANGULAR_SPEED, max(-MAX_ANGULAR_SPEED, speed))
            cmd.linear.x = 0.0
            cmd.angular.z = speed
        self.cmd_pub.publish(cmd)

    def _manual_move_status(self):
        """JSON-friendly progress snapshot for the dashboard - None when no
        manual move is active."""
        m = self.manual_move
        if m is None:
            return None
        if m['type'] == 'distance':
            traveled = math.hypot(self.x - m['start_x'], self.y - m['start_y'])
            return {'type': 'distance', 'target': m['value'], 'progress': traveled}
        return {'type': 'turn', 'target_deg': math.degrees(m['value']),
                'progress_deg': math.degrees(m['total_rotated'])}

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

        # Size-sanity gate -- see ENABLE_DUCK_SIZE_CHECK comment above.
        # Averages left+right box height rather than trusting either alone,
        # same reasoning as averaging disparity from both cameras elsewhere.
        avg_height_px = ((ly2 - ly1) + (ry2 - ry1)) / 2.0
        implied_height_m = avg_height_px * distance / self.fx_rect
        size_ratio = implied_height_m / DUCK_HEIGHT_M
        if ENABLE_DUCK_SIZE_CHECK and not (DUCK_SIZE_MIN_RATIO <= size_ratio <= DUCK_SIZE_MAX_RATIO):
            self.get_logger().warn(
                f"Duck stereo distance={distance:.2f}m + box height={avg_height_px:.0f}px implies "
                f"real height={implied_height_m:.2f}m ({size_ratio:.1f}x expected {DUCK_HEIGHT_M}m) "
                "-- outside sanity range, skipping sighting"
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
            'implied_height_m': implied_height_m,
            't': time.time(),
        }
        self.stereo_distance_history.append(distance)

        world_x = self.x + distance * math.cos(self.theta + CAMERA_BEARING_OFFSET_RAD + bearing)
        world_y = self.y + distance * math.sin(self.theta + CAMERA_BEARING_OFFSET_RAD + bearing)

        self.duck_sightings.append((world_x, world_y))
        self.get_logger().info(
            f"Duck seen (stereo): disparity={disparity_px:.1f}px dist={distance:.2f}m "
            f"bearing={math.degrees(bearing):.1f}deg implied_height={implied_height_m:.2f}m "
            f"({size_ratio:.1f}x) -> estimated world ({world_x:.2f}, {world_y:.2f})"
        )

        if ENABLE_POINTCLOUD_ESTIMATE:
            now = time.time()
            if now - self._last_pointcloud_estimate_t >= POINTCLOUD_ESTIMATE_PERIOD_S:
                self._last_pointcloud_estimate_t = now
                try:
                    self._estimate_pointcloud_position(lx1, ly1, lx2, ly2, distance)
                except Exception as e:
                    # Experimental path, not allowed to take the proven
                    # centroid-based pipeline above down with it if
                    # something in here is wrong -- log and move on.
                    self.get_logger().warn(f"Point-cloud position estimate failed: {e}")

    def _estimate_pointcloud_position(self, lx1, ly1, lx2, ly2, centroid_distance_m):
        """Dense-stereo alternative to the single-disparity centroid distance
        above -- see ENABLE_POINTCLOUD_ESTIMATE comment for why. Takes the
        rectified box corners (already computed by the caller) plus the
        existing method's distance for side-by-side comparison logging."""
        import cv2

        frame_l = self.latest_frame_left
        frame_r = self.latest_frame_right
        if frame_l is None or frame_r is None:
            return

        t0 = time.time()
        gray_l = cv2.cvtColor(frame_l, cv2.COLOR_BGR2GRAY)
        gray_r = cv2.cvtColor(frame_r, cv2.COLOR_BGR2GRAY)
        # Photometric L/R normalization (2026-09-04): live-measured on the
        # Jetson (6s, 38 samples) -- each camera's OWN auto-exposure is very
        # stable frame-to-frame (std~0.3 gray levels), but the two CSI
        # modules independently converge to different brightness for the
        # SAME scene (right ~3-4 levels / ~3% brighter than left, stable,
        # not flicker -- a fixed per-module offset). SGBM's cost function
        # assumes matching L/R intensities, so a constant brightness gap
        # biases which disparity looks like the "best" match. Rescaling R's
        # mean onto L's mean every frame (cheaper/more robust than an Argus
        # exposure/gain hardware lock, which needs a hand-picked fixed
        # exposure risking under/over-exposure or motion blur once the
        # robot moves, and won't self-adjust if the offset drifts).
        r_mean = gray_r.mean()
        if r_mean > 1.0:
            gray_r = np.clip(
                gray_r.astype(np.float32) * (gray_l.mean() / r_mean), 0, 255
            ).astype(np.uint8)
        # Full-frame, not cropped to the box -- see the "correctness bug"
        # note in ENABLE_POINTCLOUD_ESTIMATE's comment (row-cropping before
        # matching would silently invalidate Q's principal point). Cropped
        # to the box AFTER reprojecting instead, below.
        disparity = self.stereo_matcher.compute(gray_l, gray_r).astype(np.float32) / 16.0
        points3d = cv2.reprojectImageTo3D(disparity, self.Q)
        # Found live 2026-08-28: Z was coming out consistently NEGATIVE for
        # real, in-front-of-camera geometry (confirmed via diagnostic log --
        # z range at disparity-passing pixels was e.g. [-2.22, -0.44]m,
        # matching the CENTROID method's real ~0.78m distance in magnitude,
        # just sign-flipped). Root cause: cv2.reprojectImageTo3D's formula
        # has X/Y/Z all sharing the same (-Tx/disparity) scale factor from
        # Q, so this calibration's baseline (T) sign convention vs. what
        # OpenCV's Q formula assumes produced a uniform sign flip across
        # all three axes together, not a per-axis bug -- negating the whole
        # array once is the correct, minimal fix (same class of issue as
        # the documented LEFT_SENSOR_ID/RIGHT_SENSOR_ID swap bug elsewhere
        # in this file, different mechanism, same root cause: an unverified
        # sign convention assumption).
        points3d = -points3d

        h, w = disparity.shape
        x1 = max(0, min(w - 1, int(round(lx1))))
        x2 = max(0, min(w, int(round(lx2))))
        y1 = max(0, min(h - 1, int(round(ly1))))
        y2 = max(0, min(h, int(round(ly2))))
        if x2 <= x1 or y2 <= y1:
            return

        region_disp = disparity[y1:y2, x1:x2]
        region_pts = points3d[y1:y2, x1:x2]
        # X/Y/Z in OpenCV's optical convention: X=right, Y=down, Z=forward
        # (into the scene) -- same convention cv2.reprojectImageTo3D always
        # uses, matches REP103 "optical frame" (see stereo_depth_argus.
        # launch.py's own note on this exact convention).
        z = region_pts[:, :, 2]
        valid = (
            (region_disp > STEREO_MIN_DISPARITY_PX)
            & np.isfinite(z)
            & (z > 0.05)
            & (z < STEREO_MAX_DISTANCE_M)
        )
        n_valid = int(np.count_nonzero(valid))
        compute_ms = (time.time() - t0) * 1000.0
        if n_valid < POINTCLOUD_MIN_VALID_POINTS:
            # Diagnostic breakdown (2026-08-28, live debugging: first real
            # test consistently got 0 valid points) -- which individual
            # condition is actually failing, not just the combined count.
            # SGBM marks a pixel as unmatched with disparity -1 (raw -16
            # before the /16.0 scaling) when it can't find a confident
            # correlation -- a real, common failure mode on a smooth,
            # low-texture object like a rubber duck, which block-matching
            # stereo has nothing to latch onto (unlike the centroid method,
            # which only needs YOLO's box, not per-pixel texture).
            n_disp_ok = int(np.count_nonzero(region_disp > STEREO_MIN_DISPARITY_PX))
            n_finite = int(np.count_nonzero(np.isfinite(z)))
            disp_ok_mask = region_disp > STEREO_MIN_DISPARITY_PX
            z_at_disp_ok = z[disp_ok_mask] if n_disp_ok > 0 else np.array([0.0])
            self.get_logger().warn(
                f"Point-cloud estimate: only {n_valid} valid points in box "
                f"(need {POINTCLOUD_MIN_VALID_POINTS}) -- skipping, compute={compute_ms:.0f}ms. "
                f"Diagnostic: disparity range [{region_disp.min():.1f}, {region_disp.max():.1f}]px "
                f"(mean {region_disp.mean():.1f}px), {n_disp_ok}/{region_disp.size} pixels pass "
                f">threshold, {n_finite}/{region_disp.size} pixels finite depth, "
                f"z range at disp-ok pixels [{z_at_disp_ok.min():.3f}, {z_at_disp_ok.max():.3f}]m"
            )
            return

        # Median, not mean -- same reasoning as STEREO_SMOOTHING_WINDOW
        # above: reject outliers (background bleeding into the box edges,
        # bad individual disparity pixels) rather than let them drag an
        # average around. This full-box median is kept only as the
        # FALLBACK value now (see below) -- production readings prefer the
        # core-region median once the edge-bleeding bias was confirmed.
        full_x_cam, full_y_cam, full_z_cam, full_distance_m = _median_xyz_distance(region_pts, valid)

        # Promoted to production 2026-08-29 (was diagnostic-only): the
        # edge-bleeding hypothesis was directly confirmed live (full-box
        # 2.10m vs centroid's simultaneous 0.87m on a frame where the SAME
        # region's core gave 0.90m) -- a loosely-fit detection box lets
        # background pixels near the EDGES (farther than the duck) drag the
        # full-box median toward "farther". Recomputing the median from just
        # the CENTER of the box (POINTCLOUD_EROSION_MARGIN_FRAC inset on
        # each side) excludes that background bleed.
        h_box, w_box = valid.shape
        my = int(h_box * POINTCLOUD_EROSION_MARGIN_FRAC)
        mx = int(w_box * POINTCLOUD_EROSION_MARGIN_FRAC)
        core_valid = valid[my:h_box - my, mx:w_box - mx]
        core_disparity_ok = int(np.count_nonzero(core_valid))
        core_distance_txt = "n/a"
        # Coverage trade-off flagged when this was still diagnostic-only:
        # a distant/small duck can erode down to too few points. Require
        # the SAME POINTCLOUD_MIN_VALID_POINTS bar the full box uses (not
        # the looser bar the old diagnostic-only log line used) before
        # trusting the core region -- fall back to the full-box reading
        # rather than dropping the tick entirely when it doesn't clear that
        # bar, so a tight/small box doesn't just go silent.
        if core_disparity_ok >= POINTCLOUD_MIN_VALID_POINTS:
            core_region_pts = region_pts[my:h_box - my, mx:w_box - mx]
            x_cam, y_cam, z_cam, distance_m = _median_xyz_distance(core_region_pts, core_valid)
            n_valid = core_disparity_ok
            core_distance_txt = f"{distance_m:.2f}m ({core_disparity_ok}pts) [USED]"
            # Keep the /duck_pointcloud_region debug cloud published below
            # in sync with whichever region actually produced distance_m --
            # 2026-08-29 code review caught this still pointing at the
            # full-box points unconditionally, which would silently show
            # background edge-bleeding pixels in the exact debug view built
            # to rule that out.
            used_region_pts, used_valid_mask = core_region_pts, core_valid
        else:
            x_cam, y_cam, z_cam, distance_m = full_x_cam, full_y_cam, full_z_cam, full_distance_m
            core_distance_txt = f"n/a ({core_disparity_ok}pts, below {POINTCLOUD_MIN_VALID_POINTS} min) [fell back to full-box]"
            used_region_pts, used_valid_mask = region_pts, valid

        # See POINTCLOUD_CENTROID_MAX_DISAGREEMENT_FRAC's comment above --
        # rescale onto the centroid reading (proven reliable this session)
        # if this method's raw depth disagrees too much, likely from
        # locking onto background instead of the duck.
        raw_distance_m = distance_m
        agreement_frac = abs(distance_m - centroid_distance_m) / max(centroid_distance_m, 0.05)
        gated = agreement_frac > POINTCLOUD_CENTROID_MAX_DISAGREEMENT_FRAC
        if gated and distance_m > 1e-6:
            scale = centroid_distance_m / distance_m
            x_cam, y_cam, z_cam = x_cam * scale, y_cam * scale, z_cam * scale
            distance_m = centroid_distance_m

        # camera_optical (X=right,Y=down,Z=forward) -> a ROS-convention local
        # frame (x=forward,y=left,z=up) -- the same fixed axis relationship
        # stereo_depth_argus.launch.py's BASE_TO_LEFT_CAM_QUAT encodes, just
        # applied here as explicit trig instead of a quaternion/TF (so this
        # number is correct on its own without depending on a TF chain this
        # session has no way to visually verify -- see PR description).
        x_local = z_cam
        y_local = -x_cam
        z_local = -y_cam

        # Rig-mount yaw (same CAMERA_BEARING_OFFSET_RAD used by the existing
        # centroid method's bearing math) -- rotates the camera-forward local
        # frame into base_link's frame. NOT the same as stereo_depth_argus.
        # launch.py's BASE_TO_LEFT_CAM_QUAT verbatim -- that constant assumes
        # a straight-forward (unyawed) mount, which stopped being true once
        # this mission's rig got rotated 45deg for perimeter-hugging
        # (2026-08-26). Composing them wrong was a real, easy mistake to
        # make here -- flagging explicitly since it can't be tested tonight.
        c, s = math.cos(CAMERA_BEARING_OFFSET_RAD), math.sin(CAMERA_BEARING_OFFSET_RAD)
        dx_base = x_local * c - y_local * s
        dy_base = x_local * s + y_local * c
        dz_base = z_local

        # BASE_TO_LEFT_CAM_XYZ (camera mount offset from base_link) reused
        # from stereo_depth_argus.launch.py -- that offset is about mount
        # HEIGHT/position relative to the wheel axle, which the 45deg yaw
        # rotation (a rotation about the mount's own vertical axis) plausibly
        # doesn't change, but this carry-over is NOT independently
        # re-measured for this rig, unlike the yaw composition above which
        # IS derived correctly. Worth a real tape-measure check, not just
        # trusted, if this number ever needs to be precise.
        base_x = dx_base + BASE_TO_LEFT_CAM_XYZ[0]
        base_y = dy_base + BASE_TO_LEFT_CAM_XYZ[1]
        base_z = dz_base + BASE_TO_LEFT_CAM_XYZ[2]

        # base_link -> world/odom -- same flat-ground, theta-only rotation
        # the existing centroid method's world_x/world_y already use.
        wc, ws = math.cos(self.theta), math.sin(self.theta)
        world_x = self.x + base_x * wc - base_y * ws
        world_y = self.y + base_x * ws + base_y * wc
        world_z = base_z

        self.latest_pointcloud_reading = {
            'x_m': x_cam, 'y_m': y_cam, 'z_m': z_cam,
            'distance_m': distance_m,
            'world_x': world_x, 'world_y': world_y, 'world_z': world_z,
            'n_points': n_valid,
            'compute_ms': compute_ms,
            'gated': gated,
            't': time.time(),
        }
        gated_txt = (
            f" [GATED, raw was {raw_distance_m:.2f}m -> rescaled onto centroid]"
            if gated else ""
        )
        self.get_logger().info(
            f"Point-cloud position: dist={distance_m:.2f}m (centroid method said "
            f"{centroid_distance_m:.2f}m, diff={distance_m - centroid_distance_m:+.2f}m, "
            f"full-box was {full_distance_m:.2f}m, core={core_distance_txt}){gated_txt} "
            f"n_points={n_valid} compute={compute_ms:.0f}ms -> world ({world_x:.2f}, {world_y:.2f}, {world_z:.2f})"
        )

        # Cycles through POINTCLOUD_MARKER_HISTORY distinct ids instead of
        # always id=0 -- see that constant's comment. The latest sighting
        # is still the newest/brightest-looking one to the eye (freshly
        # (re)published each time), older ones in the ring just haven't
        # been overwritten yet and fade out via lifetime if sightings stop.
        sphere_id = self._pointcloud_marker_id_counter % POINTCLOUD_MARKER_HISTORY
        self._pointcloud_marker_id_counter += 1

        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'duck_pointcloud'
        marker.id = sphere_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = world_x
        marker.pose.position.y = world_y
        marker.pose.position.z = world_z
        marker.pose.orientation.w = 1.0
        # Smaller than DUCK_HEIGHT_M (was full-size when there was only ever
        # one) -- POINTCLOUD_MARKER_HISTORY spheres at full duck size would
        # just overlap into a blob, defeating the point of seeing the scatter.
        marker.scale.x = marker.scale.y = marker.scale.z = DUCK_HEIGHT_M * 0.3
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.85, 0.0, 0.75
        marker.lifetime.sec = 15  # long enough to see ~15s of recent scatter, clears out once sightings stop
        self.duck_marker_pub.publish(marker)

        # Text label (2026-08-28) -- same topic, reserved id right after the
        # cycling sphere range so it never collides with one of those (RViz2's
        # Marker display tracks markers per (ns, id), so this renders
        # alongside the spheres above, not instead of them). Reading the
        # number straight off the terminal log was the original complaint
        # this solves -- now it floats right next to the latest marker.
        text_marker = Marker()
        text_marker.header.frame_id = 'odom'
        text_marker.header.stamp = marker.header.stamp
        text_marker.ns = 'duck_pointcloud'
        text_marker.id = POINTCLOUD_MARKER_HISTORY
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = world_x
        text_marker.pose.position.y = world_y
        text_marker.pose.position.z = world_z + DUCK_HEIGHT_M  # float above the sphere, not inside it
        text_marker.pose.orientation.w = 1.0
        text_marker.scale.z = 0.08  # text height, meters -- RViz2 ignores scale.x/y for TEXT_VIEW_FACING
        text_marker.color.r, text_marker.color.g, text_marker.color.b, text_marker.color.a = 1.0, 1.0, 1.0, 1.0
        text_marker.text = f"{distance_m:.2f}m ({n_valid}pts)"
        text_marker.lifetime.sec = 2
        self.duck_marker_pub.publish(text_marker)

        # Raw point-cloud debug view (2026-08-28) -- the single median point
        # above throws away everything about WHERE on the duck stereo
        # matching actually succeeded vs. failed. Published in camera_optical
        # frame (untransformed region_pts, as SGBM/reprojectImageTo3D
        # produced them) so RViz2 places it via the static base_link->
        # camera_optical TF instead of needing a second copy of the world-
        # transform math here -- one geometry source of truth, not two.
        # Uses used_region_pts/used_valid_mask (core region when [USED],
        # full box on fallback) -- NOT unconditionally the full box -- so
        # this debug cloud always matches the points that actually produced
        # distance_m above it.
        valid_pts = used_region_pts[used_valid_mask]
        cloud_msg = point_cloud2.create_cloud_xyz32(
            header=marker.header,  # stamp reused, but frame_id gets overwritten below
            points=valid_pts.reshape(-1, 3).tolist(),
        )
        cloud_msg.header.frame_id = 'camera_optical'
        self.duck_cloud_pub.publish(cloud_msg)

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

        @app.after_request
        def _no_cache(response):
            # 2026-08-27: found the root cause of the dashboard's stale
            # "no current stereo lock" bug -- pollCount/tickCount kept
            # incrementing on a healthy 400ms cadence while the fetched
            # /state payload itself stayed stuck on an old reading, even
            # though the backend was independently confirmed fresh
            # (<0.1s old) at that exact moment. That's the signature of a
            # cached response being served instead of a live one, not a
            # slow backend or a throttled tab. `fetch(..., {cache:
            # 'no-store'})` on the client stops the BROWSER from doing
            # this; this header stops anything on the network path
            # (a WiFi router/proxy) from doing the same thing, since a
            # client-side fetch option alone can't control what a
            # caching proxy in between does.
            response.headers['Cache-Control'] = 'no-store, no-cache, must-revalidate'
            return response

        @app.route('/')
        def index():
            return HTML_PAGE

        @app.route('/vision')
        def vision_debug():
            return VISION_HTML_PAGE

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
                'latest_pointcloud': node.latest_pointcloud_reading,
                'stereo_distance_smoothed_m': (
                    statistics.median(node.stereo_distance_history)
                    if node.stereo_distance_history else None
                ),
                'stereo_smoothing_window': STEREO_SMOOTHING_WINDOW,
                'report': node.report,
                'has_camera': not node.nav_only,
                'target_waypoint': WAYPOINTS[node.waypoint_idx] if node.waypoint_idx < len(WAYPOINTS) else None,
                'pid_gains': node.pid_gains,
                'manual_move': node._manual_move_status(),
                'motor_killed': node.motor_confirmed_killed,  # ground truth
                                                                # from the ESP32
                                                                # itself, not
                                                                # just what
                                                                # we asked for
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
                max_head_trim = float(data['max_head_trim'])
                klock = float(data['klock'])
                max_lock = float(data['max_lock'])
            except (TypeError, ValueError, KeyError):
                return jsonify({'ok': False, 'error': 'expected JSON {kp, ki, max_i, khead, trim_left, trim_right, max_head_trim, klock, max_lock} as numbers'}), 400
            node.set_pid_gains(kp, ki, max_i, khead, trim_left, trim_right, max_head_trim, klock, max_lock)
            node.get_logger().info(f"PID gains updated via dashboard: Kp={kp} Ki={ki} MaxI={max_i} Khead={khead} "
                                    f"TrimL={trim_left} TrimR={trim_right} MaxHeadTrim={max_head_trim} "
                                    f"Klock={klock} MaxLock={max_lock}")
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
            if node.manual_move is not None:
                return jsonify({'started': False, 'error': 'a manual move is in progress -- cancel it first'}), 400
            node.started = True
            node.get_logger().info("Patrol STARTED (via dashboard)")
            return jsonify({'started': True})

        @app.route('/manual_move', methods=['POST'])
        def manual_move():
            if node.started:
                return jsonify({'ok': False, 'error': 'stop the patrol first'}), 400
            if node.manual_move is not None:
                return jsonify({'ok': False, 'error': 'a manual move is already in progress'}), 400
            try:
                data = request.get_json(force=True)
                move_type = data['type']
                value = float(data['value'])
            except (TypeError, ValueError, KeyError):
                return jsonify({'ok': False, 'error': 'expected JSON {type: "distance"|"turn", value: number}'}), 400
            if move_type == 'distance':
                node.manual_move = {
                    'type': 'distance', 'value': value,
                    'start_x': node.x, 'start_y': node.y,
                    'start_t': time.time(),
                }
            elif move_type == 'turn':
                node.manual_move = {
                    'type': 'turn', 'value': math.radians(value),
                    'last_theta': node.theta, 'total_rotated': 0.0,
                    'start_t': time.time(),
                }
            else:
                return jsonify({'ok': False, 'error': 'type must be "distance" or "turn"'}), 400
            node.get_logger().info(f"Manual move started (via dashboard): {move_type}={value}")
            return jsonify({'ok': True})

        @app.route('/manual_move/cancel', methods=['POST'])
        def manual_move_cancel():
            node.manual_move = None
            node.cmd_pub.publish(Twist())
            node.get_logger().info("Manual move cancelled (via dashboard)")
            return jsonify({'ok': True})

        @app.route('/stop', methods=['POST'])
        def stop():
            node.started = False
            node.cmd_pub.publish(Twist())
            node.get_logger().info("Patrol STOPPED (via dashboard)")
            return jsonify({'started': False})

        @app.route('/motor_kill', methods=['POST'])
        def motor_kill():
            # Zero any pending drive command first so re-enabling doesn't
            # lurch forward with a stale nonzero /cmd_vel the instant STBY
            # goes back HIGH.
            node.cmd_pub.publish(Twist())
            node.set_motor_killed(True)
            node.get_logger().info("Motor kill switch: OFF (via dashboard)")
            return jsonify({'motor_killed': True})

        @app.route('/motor_resume', methods=['POST'])
        def motor_resume():
            node.set_motor_killed(False)
            node.get_logger().info("Motor kill switch: ON (via dashboard)")
            return jsonify({'motor_killed': False})

        @app.route('/reset', methods=['POST'])
        def reset():
            # Resets mission state (waypoint progress, duck sightings,
            # start/done flags) AND tells the ESP32 to zero its own
            # (x,y,theta) (2026-08-27, see Node.reset_odom) -- previously the
            # odometry half needed an actual ESP32 reset (unplug/replug or
            # the board's button) since there was no software path to it.
            # self.x/y/theta here still update from the next /odom message
            # as usual, so there's a brief lag (one message) before the
            # dashboard shows 0,0,0 -- not instant, but no manual step needed.
            node.started = False
            node.done = False
            node.waypoint_idx = 0
            node.manual_move = None
            node.duck_sightings = []
            node.latest_detection_left = None
            node.latest_detection_right = None
            node.latest_stereo_reading = None
            node.stereo_distance_history.clear()
            node.report = None
            node.cmd_pub.publish(Twist())
            node.reset_odom()
            node.get_logger().info("Dashboard: RESET (mission state cleared + ESP32 odometry reset requested)")
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
            # Lowered from 0.1s (10fps) 2026-08-27 -- hypothesis for a live-observed
            # multi-second dashboard lag: two of these generators (one per camera)
            # each JPEG-encoding a frame every 100ms on Flask's threaded dev server
            # is real CPU work competing with YOLO + ROS spin on the same Jetson
            # cores, worse than it looks since the overlaid box only actually
            # changes at TARGET_DETECT_HZ=5Hz anyway -- streaming faster than that
            # buys smoother raw video at the cost of CPU, not more useful detection
            # info. NOT yet confirmed as the actual cause (see the "Vision loop: XHz"
            # log, which stayed near target throughout the lag -- so the bottleneck
            # was in streaming/serving, not detection) -- revert to 0.1 if this
            # doesn't help and the real cause turns out to be something else
            # (e.g. leaked generator threads from unclosed browser tabs -- check
            # "video client connected" vs "disconnected" counts in the terminal).
            time.sleep(0.2)
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
  /* Three logical groups side by side (overview, tuning, vision) instead of
     one narrow column doing all the control panels and one wide column
     mostly empty below the camera feeds -- uses the available horizontal
     width instead of stacking everything tall on a small screen's worth of
     column. Degrades to 2 then 1 column as the window narrows. */
  .layout { display: grid; grid-template-columns: repeat(3, minmax(320px, 1fr)); gap: 16px; align-items: start; }
  @media (max-width: 1400px) { .layout { grid-template-columns: repeat(2, minmax(320px, 1fr)); } }
  @media (max-width: 700px) { .layout { grid-template-columns: 1fr; } }
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
  #motorKillBtn.on { background: #5a1c1c; color: #ffb3b3; }
  #motorKillBtn.off { background: #7a1414; color: #ffe3e3; box-shadow: 0 0 0 2px #ff5b5b; }
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
    <button id="motorKillBtn" class="on" onclick="toggleMotorKill()">Motors: ON</button>
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
    </div>
    <div class="column">
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
        </div>
        <div class="pid-row">
          <label>Heading Kp (gyro) <input id="pidKhead" type="number" step="1"></label>
          <label>Max heading trim (RPM) <input id="pidMaxHeadTrim" type="number" step="5"></label>
          <button id="pidPushBtn" onclick="pushPidGains()">Push to robot</button>
        </div>
        <div class="pid-row">
          <label>Trim left (PWM) <input id="pidTrimL" type="number" step="1"></label>
          <label>Trim right (PWM) <input id="pidTrimR" type="number" step="1"></label>
        </div>
        <div class="pid-row">
          <label>Heading lock Kp <input id="pidKlock" type="number" step="0.5"></label>
          <label>Max lock rate (rad/s) <input id="pidMaxLock" type="number" step="0.1"></label>
        </div>
        <div class="legend" id="pidStatus">
          re-pushed automatically every 2s, so an ESP32 reboot picks the saved values back up on its own.
          Trim = a flat PWM offset added on top of the PID output, for balancing a mechanically weaker motor -
          only applies while actually driving, never during a stop.
          Max heading trim = the ceiling on how hard heading correction can push the wheels apart -- set it
          very high for effectively no ceiling.
          Heading lock = holds whatever heading the robot is pointed at whenever it's not actively told to turn
          (straight driving, or sitting still after a turn) -- Kp is how hard it corrects a heading error,
          max lock rate caps how fast it's allowed to spin the robot while doing that correction.
        </div>
      </div>
      <div class="panel">
        <div class="panel-title">Manual drive tester (bypasses mission Start)</div>
        <div class="pid-row">
          <label>Distance (m) <input id="manualDist" type="number" step="0.1" value="0.5"></label>
          <button id="manualDriveBtn" onclick="manualMove('distance')">Drive Forward</button>
        </div>
        <div class="pid-row">
          <label>Turn (deg, +left / -right) <input id="manualDeg" type="number" step="5" value="90"></label>
          <button id="manualTurnBtn" onclick="manualMove('turn')">Turn</button>
        </div>
        <div class="pid-row">
          <button id="manualCancelBtn" onclick="manualCancel()">Cancel</button>
        </div>
        <div class="legend" id="manualStatus">idle</div>
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
        <div id="pointcloudBox"></div>
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

let motorKilled = false;  // updated from /state each poll -- see poll() below

function toggleMotorKill() {
  sendCmd(motorKilled ? '/motor_resume' : '/motor_kill');
}

let pidFieldsInitialized = false;  // only fill the inputs once from the server --
                                    // otherwise the 400ms poll would overwrite
                                    // whatever vịt is mid-typing

function pushPidGains() {
  const kp = parseFloat(document.getElementById('pidKp').value);
  const ki = parseFloat(document.getElementById('pidKi').value);
  const max_i = parseFloat(document.getElementById('pidMaxI').value);
  const khead = parseFloat(document.getElementById('pidKhead').value);
  const max_head_trim = parseFloat(document.getElementById('pidMaxHeadTrim').value);
  const trim_left = parseFloat(document.getElementById('pidTrimL').value);
  const trim_right = parseFloat(document.getElementById('pidTrimR').value);
  const klock = parseFloat(document.getElementById('pidKlock').value);
  const max_lock = parseFloat(document.getElementById('pidMaxLock').value);
  if (!isFinite(kp) || !isFinite(ki) || !isFinite(max_i) || !isFinite(khead) ||
      !isFinite(max_head_trim) || !isFinite(trim_left) || !isFinite(trim_right) ||
      !isFinite(klock) || !isFinite(max_lock)) {
    document.getElementById('pidStatus').innerText = 'enter valid numbers in all fields first';
    return;
  }
  fetch('/pid_gains', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ kp, ki, max_i, khead, trim_left, trim_right, max_head_trim, klock, max_lock }),
  }).then(r => r.json()).then(res => {
    document.getElementById('pidStatus').innerText = res.ok
      ? `saved + pushed at ${new Date().toLocaleTimeString()} -- watch the diag panel above for KP/KI/MAXI/KHEAD/MAXHEAD/TRIML/TRIMR/KLOCK/MAXLOCK/LOCK to confirm the ESP32 picked it up`
      : (res.error || 'push failed');
  }).catch(() => {
    document.getElementById('pidStatus').innerText = 'push failed (dashboard unreachable?)';
  });
}

function manualMove(type) {
  const inputId = type === 'distance' ? 'manualDist' : 'manualDeg';
  const value = parseFloat(document.getElementById(inputId).value);
  if (!isFinite(value)) {
    document.getElementById('manualStatus').innerText = 'enter a valid number first';
    return;
  }
  fetch('/manual_move', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ type, value }),
  }).then(r => r.json()).then(res => {
    if (!res.ok) document.getElementById('manualStatus').innerText = res.error || 'failed to start';
  }).catch(() => {
    document.getElementById('manualStatus').innerText = 'request failed (dashboard unreachable?)';
  });
}

function manualCancel() {
  fetch('/manual_move/cancel', { method: 'POST' }).catch(() => {});
}

function poll() {
  fetch('/state', { cache: 'no-store' }).then(r => r.json()).then(state => {
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
      document.getElementById('pidMaxHeadTrim').value = state.pid_gains.max_head_trim;
      document.getElementById('pidTrimL').value = state.pid_gains.trim_left;
      document.getElementById('pidTrimR').value = state.pid_gains.trim_right;
      document.getElementById('pidKlock').value = state.pid_gains.klock;
      document.getElementById('pidMaxLock').value = state.pid_gains.max_lock;
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

    // Point-cloud (dense-stereo) distance, mirrored here from the /vision
    // debug page (2026-09-04) so it's visible without switching pages --
    // this is the box that shows POINTCLOUD_CENTROID_MAX_DISAGREEMENT_FRAC
    // gating (search_and_rescue.py) when point-cloud disagrees with the
    // centroid method above by too much.
    const pointcloudBox = document.getElementById('pointcloudBox');
    const pcFresh = state.latest_pointcloud && (Date.now() / 1000 - state.latest_pointcloud.t) < 3.0;
    if (pcFresh) {
      const p = state.latest_pointcloud;
      const s = state.latest_stereo;
      const diffTxt = (stereoFresh && s)
        ? ` (centroid method: ${s.distance_m.toFixed(2)}m, diff ${(p.distance_m - s.distance_m).toFixed(2)}m)`
        : '';
      const gatedTxt = p.gated
        ? ' <span style="color:#e0a030;">[gated -&gt; using centroid]</span>' : '';
      pointcloudBox.innerHTML = `<div class="stat-highlight">point-cloud distance: ` +
        `<b>${p.distance_m.toFixed(2)}m</b>${diffTxt}${gatedTxt} ` +
        `<span style="color:#8a94a6;">(${p.n_points} points, ${p.compute_ms.toFixed(0)}ms compute)</span></div>`;
    } else {
      pointcloudBox.innerHTML = `<div class="stat-highlight stale">no current point-cloud estimate</div>`;
    }

    document.getElementById('startBtn').disabled = state.started || state.done || !!state.manual_move;
    document.getElementById('stopBtn').disabled = !state.started;

    motorKilled = !!state.motor_killed;
    const motorBtn = document.getElementById('motorKillBtn');
    motorBtn.innerText = motorKilled ? 'Motors: OFF (tap to resume)' : 'Motors: ON';
    motorBtn.className = motorKilled ? 'off' : 'on';

    const manualActive = !!state.manual_move;
    document.getElementById('manualDriveBtn').disabled = state.started || manualActive;
    document.getElementById('manualTurnBtn').disabled = state.started || manualActive;
    document.getElementById('manualCancelBtn').disabled = !manualActive;
    if (manualActive) {
      const mm = state.manual_move;
      document.getElementById('manualStatus').innerText = mm.type === 'distance'
        ? `driving forward: ${mm.progress.toFixed(2)}m / ${mm.target.toFixed(2)}m`
        : `turning: ${mm.progress_deg.toFixed(1)}deg / ${mm.target_deg.toFixed(1)}deg`;
    } else {
      document.getElementById('manualStatus').innerText = 'idle';
    }

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


# Minimal, dedicated vision/stereo debug page -- deliberately separate from
# HTML_PAGE above (2026-08-27). That page has accreted a lot of PID-tuning
# UI from Alex's work that's irrelevant to the stereo-distance display bug,
# making it hard to tell "is this a real data problem" from "is this buried
# in 400 lines of shared JS". This page reuses the exact same /state JSON
# and /video_feed_left /video_feed_right routes (no new backend needed) but
# renders almost nothing else -- just the two camera feeds, the distance
# readout, and a live diagnostic strip that answers the open staleness
# question at a glance: pollCount/clockTick tick on their own 100ms timer
# independent of the 400ms /state fetch, so if the BROWSER TAB itself is
# throttled (e.g. backgrounded on a phone), this whole strip visibly
# freezes; if only the fetch stops getting fresh data, the strip keeps
# ticking smoothly while "state age" grows unbounded -- two different bugs,
# now visually distinguishable without opening devtools.
VISION_HTML_PAGE = """<!doctype html>
<html>
<head>
<meta charset="utf-8">
<title>Vision debug -- stereo duck distance</title>
<style>
  :root { color-scheme: dark; }
  body {
    background: #12151a; color: #e6e6e6;
    font-family: -apple-system, Segoe UI, Roboto, sans-serif;
    margin: 0; padding: 16px;
  }
  h1 { font-size: 1.1rem; font-weight: 600; margin: 0 0 12px; color: #9fd3ff; }
  .video-row { display: flex; gap: 10px; flex-wrap: wrap; margin-bottom: 12px; }
  .video-cell { flex: 1 1 300px; }
  .video-cell img { width: 100%; border-radius: 6px; background: #000; display: block; }
  .video-cell .label { font-size: 0.7rem; color: #8a94a6; margin-bottom: 4px; }
  .stat-highlight {
    font-family: ui-monospace, Menlo, Consolas, monospace; font-size: 1.15rem;
    padding: 10px 12px; margin-bottom: 12px; border-radius: 8px;
    background: #10201c; border: 1px solid #1c4d2b; color: #9fe6ae;
  }
  .stat-highlight.stale { background: #201c10; border-color: #4a3f18; color: #8a94a6; }
  .diag {
    font-family: ui-monospace, Menlo, Consolas, monospace; font-size: 0.78rem;
    color: #8a94a6; background: #1b1f27; border: 1px solid #2a2f3a; border-radius: 8px;
    padding: 8px 10px; margin-bottom: 12px; line-height: 1.6;
  }
  .diag b { color: #c8ccd4; }
  pre {
    background: #0e1116; border: 1px solid #2a2f3a; border-radius: 8px;
    padding: 10px; font-size: 0.75rem; overflow-x: auto; max-height: 300px;
  }
</style>
</head>
<body>
  <h1>Vision debug -- stereo duck distance (separate from the main patrol dashboard)</h1>
  <div class="video-row">
    <div class="video-cell">
      <div class="label">left camera</div>
      <img src="/video_feed_left">
    </div>
    <div class="video-cell">
      <div class="label">right camera</div>
      <img src="/video_feed_right">
    </div>
  </div>
  <div id="stereoBox" class="stat-highlight">connecting...</div>
  <div id="pointcloudBox" class="stat-highlight">connecting...</div>
  <div id="diagBox" class="diag">-</div>
  <pre id="rawJson">-</pre>

<script>
let lastState = null;
let lastFetchAt = 0;
let pollCount = 0;
let tickCount = 0;

function poll() {
  fetch('/state', { cache: 'no-store' }).then(r => r.json()).then(state => {
    lastState = state;
    lastFetchAt = Date.now();
    pollCount++;
    document.getElementById('rawJson').innerText = JSON.stringify(
      { latest_stereo: state.latest_stereo,
        latest_pointcloud: state.latest_pointcloud,
        stereo_distance_smoothed_m: state.stereo_distance_smoothed_m,
        has_camera: state.has_camera },
      null, 2
    );
  }).catch(err => {
    document.getElementById('diagBox').innerHTML =
      '<b>fetch error:</b> ' + err + ' (backend unreachable, not a staleness issue)';
  });
}

function tick() {
  tickCount++;
  const now = Date.now();
  const sinceFetchMs = lastFetchAt ? (now - lastFetchAt) : null;
  const stereoFresh = lastState && lastState.latest_stereo &&
    (now / 1000 - lastState.latest_stereo.t) < 2.0;

  const stereoBox = document.getElementById('stereoBox');
  if (stereoFresh) {
    const s = lastState.latest_stereo;
    const smoothed = lastState.stereo_distance_smoothed_m;
    const smoothedTxt = smoothed !== null && smoothed !== undefined
      ? ` / <b>${smoothed.toFixed(2)}m</b> smoothed` : '';
    stereoBox.className = 'stat-highlight';
    stereoBox.innerHTML = `distance to duck now: <b>${s.distance_m.toFixed(2)}m</b> raw${smoothedTxt} ` +
      `@ ${s.bearing_deg.toFixed(1)}&deg; (disparity ${s.disparity_px.toFixed(0)}px)`;
  } else {
    stereoBox.className = 'stat-highlight stale';
    stereoBox.innerHTML = 'no current stereo lock (duck not seen by both cameras right now)';
  }

  // Point-cloud method (2026-08-27 night, experimental, see
  // ENABLE_POINTCLOUD_ESTIMATE in search_and_rescue.py) -- runs much slower
  // (~1Hz target vs the centroid method's ~5Hz) so its freshness window is
  // looser. Shown side by side with the box above specifically so the two
  // methods' distance numbers can be eyeballed against each other live --
  // that comparison IS the point of this box, not just a duplicate readout.
  const pcFresh = lastState && lastState.latest_pointcloud &&
    (now / 1000 - lastState.latest_pointcloud.t) < 3.0;
  const pcBox = document.getElementById('pointcloudBox');
  if (pcFresh) {
    const p = lastState.latest_pointcloud;
    const s = lastState.latest_stereo;
    const diffTxt = (stereoFresh && s)
      ? ` (centroid method: ${s.distance_m.toFixed(2)}m, diff ${(p.distance_m - s.distance_m).toFixed(2)}m)`
      : '';
    // p.gated (2026-09-04): true when this reading disagreed with the
    // centroid method by more than POINTCLOUD_CENTROID_MAX_DISAGREEMENT_FRAC
    // and got rescaled onto centroid's distance instead of its own raw
    // (likely background-locked) depth -- see that constant's comment.
    const gatedTxt = p.gated
      ? ' <span style="color:#e0a030;">[gated -> using centroid]</span>' : '';
    pcBox.className = 'stat-highlight';
    pcBox.innerHTML = `point-cloud distance: <b>${p.distance_m.toFixed(2)}m</b>${diffTxt}${gatedTxt} ` +
      `<span style="color:#8a94a6;">(${p.n_points} points, ${p.compute_ms.toFixed(0)}ms compute)</span>`;
  } else {
    pcBox.className = 'stat-highlight stale';
    pcBox.innerHTML = 'no current point-cloud estimate';
  }

  const ageTxt = lastState && lastState.latest_stereo
    ? (now / 1000 - lastState.latest_stereo.t).toFixed(2) + 's'
    : 'n/a (no reading yet)';
  document.getElementById('diagBox').innerHTML =
    `<b>tickCount</b>: ${tickCount} (100ms timer -- freezing this means the BROWSER TAB is throttled, not a data bug)<br>` +
    `<b>pollCount</b>: ${pollCount} (400ms /state fetches completed)<br>` +
    `<b>time since last successful /state fetch</b>: ${sinceFetchMs !== null ? sinceFetchMs + 'ms' : 'n/a'}<br>` +
    `<b>latest_stereo.t age</b>: ${ageTxt} (backend-reported freshness of the last duck sighting)`;
}

setInterval(poll, 400);
setInterval(tick, 100);
poll();
tick();
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
