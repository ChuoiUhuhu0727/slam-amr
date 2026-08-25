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
  rectified camera frames, use the pixel shift (disparity) of the duck's box
  between the two frames + the known 8.3cm camera separation to triangulate
  real-world distance. Needs only plain OpenCV + the existing calibration
  file -- no Isaac ROS / ROS image pipeline dependency. Replaces the earlier
  monocular distance-from-known-duck-height approach (kept as
  search_and_rescue_monocular_backup.py) which needed the duck's real height
  hardcoded -- this doesn't need to know the duck's size at all.

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
import math
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# ---------------------------------------------------------------------------
# CONFIG -- confirm/measure these before trusting the output
# ---------------------------------------------------------------------------

# Room origin = robot's start position, facing +x down one wall (confirmed
# with vịt 2026-08-24). 0.3m margin from the actual walls so the robot
# doesn't clip anything while hugging the perimeter.
WAYPOINTS = [
    (0.3, 0.3),
    (1.7, 0.3),
    (1.7, 1.7),
    (0.3, 1.7),
    (0.3, 0.3),  # back to start -- also how we measure real loop drift
]

# Full room extent (NOT the inset patrol path) -- used only for grid reporting
# and drawing the dashboard map. Easy to change later, just constants.
ROOM_SIZE_M = 2.0
GRID_CELL_M = 0.5  # -> 4x4 grid

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

GOAL_TOLERANCE_M = 0.10       # "close enough" to a waypoint
TURN_IN_PLACE_THRESHOLD = 0.35  # rad (~20deg) -- above this, stop and turn first
MAX_LINEAR_SPEED = 0.15       # m/s -- deliberately slow, this is a small room
MAX_ANGULAR_SPEED = 0.8       # rad/s
KP_HEADING = 1.5

CONTROL_PERIOD_S = 0.1   # 10 Hz control loop, navigation only -- camera grab +
                         # detection run on their own thread (see _vision_loop),
                         # paced naturally by how long YOLO takes, not this timer.

WEIGHTS_PATH = Path(__file__).resolve().parents[1] / "training/runs/detect/train-4/weights/best.pt"
CALIB_PATH = Path(__file__).resolve().parents[2] / "stereo_calibration.npz"
CONF_THRESHOLD = 0.3
# Detection runs on the CPU on this Jetson (torch/CUDA driver mismatch --
# separate, pre-existing issue, not fixed here). Shrinking the image the
# model looks at (default would be the full 1280x720 frame) cuts CPU cost a
# lot for not much accuracy loss -- the duck is a large, obvious shape, it
# doesn't need full resolution to be found. Now running twice per detect
# tick (once per camera) instead of once, so this matters more than before.
DETECT_IMGSZ = 480

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

        self.waypoint_idx = 0
        self.done = False
        self.started = False  # gated by the dashboard's Start button -- camera/
                               # detection run regardless, only driving waits

        self.duck_sightings = []  # list of (world_x, world_y)
        self.report = None        # filled in once the loop finishes

        self.latest_frame_left = None       # for the dashboard video feed (rectified)
        self.latest_frame_right = None
        self.latest_detection_left = None   # {'x1','y1','x2','y2','conf','t'} for the overlay box
        self.latest_detection_right = None
        self.latest_stereo_reading = None   # {'distance_m','bearing_deg','disparity_px','t'} -- for
                                             # a live "distance right now" readout, useful for a
                                             # ruler accuracy check without reading terminal logs

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.control_timer = self.create_timer(CONTROL_PERIOD_S, self.control_tick)

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
            self.latest_frame_left = cv2.remap(frame_l, self.map_left_x, self.map_left_y, cv2.INTER_LINEAR)
            self.latest_frame_right = cv2.remap(frame_r, self.map_right_x, self.map_right_y, cv2.INTER_LINEAR)
        elif self.vision_loop_count % 50 == 0:  # throttled -- don't spam if it keeps failing
            self.get_logger().warn(
                f"Camera frame grab failing (left ok={ok_l}, right ok={ok_r})"
            )

    def _best_box(self, frame):
        """Run detection on one rectified frame, return the highest-confidence
        box as (x1,y1,x2,y2,conf), or None if nothing above threshold."""
        results = self.model(frame, conf=CONF_THRESHOLD, imgsz=DETECT_IMGSZ, verbose=False)
        boxes = results[0].boxes
      
        if boxes is None or len(boxes) == 0:
            return None
        box = boxes[boxes.conf.argmax()]
        x1, y1, x2, y2 = box.xyxy[0].tolist()
        conf = float(box.conf[0])
        return (x1, y1, x2, y2, conf)

    def _detect_tick(self):
        frame_l = self.latest_frame_left
        frame_r = self.latest_frame_right
        if frame_l is None or frame_r is None:
            return

        box_l = self._best_box(frame_l)
        box_r = self._best_box(frame_r)

        self.latest_detection_left = None
        self.latest_detection_right = None
        if box_l:
            x1, y1, x2, y2, conf = box_l
            self.latest_detection_left = {'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2, 'conf': conf, 't': time.time()}
        if box_r:
            x1, y1, x2, y2, conf = box_r
            self.latest_detection_right = {'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2, 'conf': conf, 't': time.time()}

        if not box_l or not box_r:
            # Duck only visible in one camera (or neither) -- per plan, don't
            # guess. Skip this sighting entirely and wait for a tick where
            # both cameras agree the duck is there.
            return

        x1_l, _, x2_l, _, _ = box_l
        x1_r, _, x2_r, _, _ = box_r
        center_x_l = (x1_l + x2_l) / 2.0
        center_x_r = (x1_r + x2_r) / 2.0

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
        cell_x = max(0, min(int(ROOM_SIZE_M / GRID_CELL_M) - 1, int(avg_x // GRID_CELL_M)))
        cell_y = max(0, min(int(ROOM_SIZE_M / GRID_CELL_M) - 1, int(avg_y // GRID_CELL_M)))

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
        from flask import Flask, Response, jsonify

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
                'waypoint_idx': node.waypoint_idx,
                'waypoints': WAYPOINTS,
                'room_size': ROOM_SIZE_M,
                'grid_cell': GRID_CELL_M,
                'done': node.done,
                'started': node.started,
                'duck_sightings': node.duck_sightings,
                'duck_estimate': node.duck_estimate(),
                'latest_stereo': node.latest_stereo_reading,
                'report': node.report,
                'has_camera': not node.nav_only,
                'target_waypoint': WAYPOINTS[node.waypoint_idx] if node.waypoint_idx < len(WAYPOINTS) else None,
            })

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
        <canvas id="map" width="440" height="440"></canvas>
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

function draw(state) {
  const room = state.room_size;
  const scale = canvas.width / room;
  const toScreen = (x, y) => [x * scale, canvas.height - y * scale];

  ctx.clearRect(0, 0, canvas.width, canvas.height);

  // grid
  ctx.strokeStyle = '#232833';
  ctx.lineWidth = 1;
  for (let g = 0; g <= room + 1e-6; g += state.grid_cell) {
    let [gx, ] = toScreen(g, 0);
    ctx.beginPath(); ctx.moveTo(gx, 0); ctx.lineTo(gx, canvas.height); ctx.stroke();
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

    const stereoFresh = state.latest_stereo && (Date.now() / 1000 - state.latest_stereo.t) < 2.0;
    document.getElementById('duckStatus').innerHTML =
      `camera: <b>${state.has_camera ? 'on' : 'off (--nav-only)'}</b><br>` +
      `duck sightings recorded: <b>${state.duck_sightings.length}</b>`;

    const stereoBox = document.getElementById('stereoBox');
    if (stereoFresh) {
      stereoBox.innerHTML = `<div class="stat-highlight">distance to duck now: ` +
        `<b>${state.latest_stereo.distance_m.toFixed(2)}m</b> @ ${state.latest_stereo.bearing_deg.toFixed(1)}&deg; ` +
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


def main():
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
