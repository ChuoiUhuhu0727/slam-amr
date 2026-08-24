"""Option 1 search-and-rescue demo: robot patrols the room perimeter (hardcoded
waypoints, go-to-goal control using /odom -- gyro-fused heading, wheel-encoder
position), running duck detection the whole time. Reports which grid cell the
duck was in once the loop completes.

Also runs a live web dashboard (Flask, background thread) showing the room
map, robot position/heading, duck sightings, and the camera feed with the
current detection box drawn on it -- for watching/debugging a live run instead
of only reading terminal logs. Reachable from any device on the same network
at http://<jetson-hostname>.local:8080 (this Jetson's mDNS hostname is
chuoi.local, already set up from earlier SSH work).

Deliberately does NOT use Nav2, VSLAM, or the stereo depth pipeline:
- Nav2/VSLAM: skipped because cuVSLAM still has an unresolved ~3-5x pose scale
  bug (see project memory), and Nav2's costmap/planner stack has never been
  tested end-to-end on real hardware. A small known/bounded room doesn't need
  either -- hardcoded waypoints + /odom (already tested, gyro-fused heading)
  cover it.
- Stereo depth (/stereo/points2): needs the full Isaac ROS Docker container +
  rectified stereo launch running just to get points. Instead, this uses
  monocular distance-from-known-object-height, which needs only one camera
  frame + the existing calibration intrinsics -- no Isaac ROS dependency.

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

DUCK_HEIGHT_M = 0.13  # measured 2026-08-24

# How far the camera is physically rotated relative to the chassis' forward
# direction (base_link +x), radians. While hugging the perimeter the robot's
# forward direction points ALONG the wall, not toward room center -- a
# straight-ahead-mounted camera would mostly look down the wall and rarely
# catch the duck. Angling the camera inward fixes this without needing a
# second camera. Positive = rotated toward the robot's left (REP103 CCW+).
# Confirmed: camera mounted 45deg toward the robot's RIGHT -> negative.
CAMERA_BEARING_OFFSET_RAD = -math.radians(45)

GOAL_TOLERANCE_M = 0.10       # "close enough" to a waypoint
TURN_IN_PLACE_THRESHOLD = 0.35  # rad (~20deg) -- above this, stop and turn first
MAX_LINEAR_SPEED = 0.15       # m/s -- deliberately slow, this is a small room
MAX_ANGULAR_SPEED = 0.8       # rad/s
KP_HEADING = 1.5

CONTROL_PERIOD_S = 0.1   # 10 Hz control loop
DETECT_EVERY_N_TICKS = 3  # run YOLO ~every 3rd control tick (~3.3 Hz) -- inference
                           # is the slow part, don't let it stall steering updates.
                           # The camera frame itself is still grabbed every tick
                           # (cheap) so the dashboard video feed stays smooth.

WEIGHTS_PATH = Path(__file__).resolve().parents[1] / "training/runs/detect/train-4/weights/best.pt"
CALIB_PATH = Path(__file__).resolve().parents[2] / "stereo_calibration.npz"
CONF_THRESHOLD = 0.3

CSI_PIPELINE = (
    "nvarguscamerasrc sensor-id=0 ! "
    "video/x-raw(memory:NVMM), width=1280, height=720, framerate=30/1 ! "
    "nvvidconv ! "
    "video/x-raw, format=BGRx ! "
    "videoconvert ! "
    "video/x-raw, format=BGR ! appsink drop=1"
)

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
        self.tick_count = 0

        self.duck_sightings = []  # list of (world_x, world_y)
        self.report = None        # filled in once the loop finishes

        self.latest_frame = None       # for the dashboard video feed
        self.latest_detection = None   # {'x1','y1','x2','y2','conf','t'} for the overlay box

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
        K1 = calib["K1"]
        self.fx = float(K1[0, 0])
        self.cx = float(K1[0, 2])

        self.model = YOLO(str(WEIGHTS_PATH))
        self.cap = cv2.VideoCapture(CSI_PIPELINE, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            raise RuntimeError("Could not open CSI camera -- check sensor-id / GStreamer pipeline")
        self.get_logger().info(f"Vision ready: fx={self.fx:.1f}, cx={self.cx:.1f}")

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

        self.tick_count += 1

        if not self.nav_only:
            self._grab_frame()
            if self.tick_count % DETECT_EVERY_N_TICKS == 0:
                self._detect_tick()

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
        ok, frame = self.cap.read()
        if ok:
            self.latest_frame = frame
        elif self.tick_count % 50 == 0:  # throttled -- don't spam if it keeps failing
            self.get_logger().warn("Camera frame grab failing (cap.read() returned False)")

    def _detect_tick(self):
        frame = self.latest_frame
        if frame is None:
            return

        results = self.model(frame, conf=CONF_THRESHOLD, verbose=False)
        boxes = results[0].boxes
        if boxes is None or len(boxes) == 0:
            self.latest_detection = None
            return

        # take the largest/most confident box if more than one hit
        box = boxes[boxes.conf.argmax()]
        x1, y1, x2, y2 = box.xyxy[0].tolist()
        conf = float(box.conf[0])
        bbox_height_px = y2 - y1
        bbox_center_x = (x1 + x2) / 2.0

        self.latest_detection = {'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2, 'conf': conf, 't': time.time()}

        if bbox_height_px <= 1:
            return

        distance = (DUCK_HEIGHT_M * self.fx) / bbox_height_px

        # Pixel offset from image center -> bearing angle. Camera mount has
        # ~0 x/y offset from base_link (see stereo_depth_argus.launch.py),
        # just faces a fixed direction relative to the chassis, so camera
        # position ~= robot's own -- no extra translation needed, only the
        # CAMERA_BEARING_OFFSET_RAD rotation below.
        # SIGN NOT YET EMPIRICALLY VERIFIED: image-right (+dx) should mean
        # "duck is to the robot's right" = negative yaw offset (REP103: CCW
        # positive). Test with the duck placed to one known side first: if
        # the reported y comes out on the wrong side, flip this sign.
        dx_px = bbox_center_x - self.cx
        bearing = -math.atan2(dx_px, self.fx)

        world_x = self.x + distance * math.cos(self.theta + CAMERA_BEARING_OFFSET_RAD + bearing)
        world_y = self.y + distance * math.sin(self.theta + CAMERA_BEARING_OFFSET_RAD + bearing)

        self.duck_sightings.append((world_x, world_y))
        self.get_logger().info(
            f"Duck seen: dist={distance:.2f}m bearing={math.degrees(bearing):.1f}deg "
            f"-> estimated world ({world_x:.2f}, {world_y:.2f})"
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
                'report': node.report,
                'has_camera': not node.nav_only,
                'target_waypoint': WAYPOINTS[node.waypoint_idx] if node.waypoint_idx < len(WAYPOINTS) else None,
            })

        @app.route('/video_feed')
        def video_feed():
            if node.nav_only:
                return Response("Camera not active in --nav-only mode", mimetype='text/plain')
            return Response(_mjpeg_generator(node), mimetype='multipart/x-mixed-replace; boundary=frame')

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

        thread = threading.Thread(
            target=lambda: app.run(host=WEB_HOST, port=WEB_PORT, debug=False, use_reloader=False, threaded=True),
            daemon=True,
        )
        thread.start()
        self.get_logger().info(f"Web dashboard on http://<jetson-hostname>.local:{WEB_PORT}  (e.g. http://chuoi.local:{WEB_PORT})")


def _mjpeg_generator(node):
    import cv2
    node.get_logger().info("Dashboard: video client connected")
    try:
        while True:
            frame = node.latest_frame
            if frame is None:
                # No real frame yet -- send a placeholder instead of nothing, so the
                # browser actually renders something instead of spinning forever
                # waiting for the first byte. If you see this image, the camera
                # pipeline itself isn't producing frames (check the "Camera frame
                # grab failing" warning in the terminal); if you never see even
                # this, the problem is the HTTP stream, not the camera.
                display = np.zeros((360, 640, 3), dtype=np.uint8)
                cv2.putText(display, "waiting for camera frame...", (30, 180),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)
            else:
                display = frame.copy()
                det = node.latest_detection
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
        node.get_logger().info("Dashboard: video client disconnected")


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
  .layout { display: flex; gap: 20px; flex-wrap: wrap; align-items: flex-start; }
  .panel {
    background: #1b1f27; border: 1px solid #2a2f3a; border-radius: 10px;
    padding: 14px;
  }
  canvas { background: #0e1116; border-radius: 6px; display: block; }
  img#video { width: 480px; max-width: 100%; border-radius: 6px; background: #000; }
  .status-line { font-family: ui-monospace, Menlo, Consolas, monospace; font-size: 0.85rem; line-height: 1.6; }
  .status-line b { color: #9fd3ff; }
  .badge {
    display: inline-block; padding: 2px 8px; border-radius: 999px; font-size: 0.75rem;
    background: #2a2f3a; margin-left: 6px;
  }
  .badge.done { background: #1c4d2b; color: #9fe6ae; }
  .badge.waiting { background: #4a3f18; color: #ffe08a; }
  .report {
    margin-top: 10px; padding: 10px; border-radius: 8px; background: #2a2410;
    border: 1px solid #4a3f18; color: #ffe08a; font-family: ui-monospace, monospace; font-size: 0.85rem;
  }
  .controls { margin-bottom: 14px; }
  button {
    font-size: 0.95rem; font-weight: 600; padding: 10px 20px; border-radius: 8px;
    border: none; cursor: pointer; margin-right: 10px;
  }
  #startBtn { background: #1c4d2b; color: #9fe6ae; }
  #startBtn:disabled { background: #23282f; color: #555; cursor: not-allowed; }
  #stopBtn { background: #5a1c1c; color: #ffb3b3; }
  #stopBtn:disabled { background: #23282f; color: #555; cursor: not-allowed; }
</style>
</head>
<body>
  <h1>Search &amp; Rescue -- live dashboard</h1>
  <div class="controls">
    <button id="startBtn" onclick="sendCmd('/start')">Start Patrol</button>
    <button id="stopBtn" onclick="sendCmd('/stop')">Stop</button>
  </div>
  <div class="layout">
    <div class="panel">
      <canvas id="map" width="440" height="440"></canvas>
      <div style="font-size:0.72rem; color:#8a94a6; margin-top:8px; line-height:1.5;">
        solid box = room walls &middot; dashed box = patrol path &middot;
        <span style="color:#ff5b5b;">&#9679;</span> = current best duck estimate &middot;
        <span style="color:#ffd76b;">&#9679;</span> = individual sightings &middot;
        <span style="color:#9fd3ff;">&#9654;</span> = robot &middot;
        line = current target
      </div>
    </div>
    <div class="panel">
      <img id="video" src="/video_feed">
    </div>
    <div class="panel" style="min-width: 240px;">
      <div class="status-line" id="status">connecting...</div>
      <div id="reportBox"></div>
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

    document.getElementById('status').innerHTML =
      `pos: <b>(${state.x.toFixed(2)}, ${state.y.toFixed(2)})</b> ` +
      `heading: <b>${state.theta_deg.toFixed(1)}&deg;</b> ${odomBadge}<br>` +
      `waypoint: <b>${state.waypoint_idx}/${state.waypoints.length}</b> ${statusBadge}<br>` +
      `duck sightings: <b>${state.duck_sightings.length}</b><br>` +
      `camera: <b>${state.has_camera ? 'on' : 'off (--nav-only)'}</b>`;

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
    document.getElementById('status').innerText = 'connection lost, retrying...';
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
