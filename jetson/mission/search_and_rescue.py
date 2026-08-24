"""Option 1 search-and-rescue demo: robot patrols the room perimeter (hardcoded
waypoints, go-to-goal control using /odom -- gyro-fused heading, wheel-encoder
position), running duck detection the whole time. Reports which grid cell the
duck was in once the loop completes.

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

Prerequisite: micro-ROS agent already connected (ESP32 publishing /odom,
subscribed to /cmd_vel) -- this node doesn't manage that.
"""
import argparse
import math
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

# Full room extent (NOT the inset patrol path) -- used only for grid reporting.
ROOM_SIZE_M = 2.0
GRID_CELL_M = 0.5  # -> 4x4 grid

# TODO(vịt): measure the duck toy's real height with a ruler, meters.
DUCK_HEIGHT_M = 0.10  # PLACEHOLDER -- replace with a real measurement

# How far the camera is physically rotated relative to the chassis' forward
# direction (base_link +x), radians. While hugging the perimeter the robot's
# forward direction points ALONG the wall, not toward room center -- a
# straight-ahead-mounted camera would mostly look down the wall and rarely
# catch the duck. Angling the camera inward fixes this without needing a
# second camera. Positive = rotated toward the robot's left (REP103 CCW+).
# PLACEHOLDER -- set this to match however you actually mount it (e.g. -0.785
# for 45deg toward the room interior if the room center is to the right of
# the direction of travel; sign depends on which way you mount it).
CAMERA_BEARING_OFFSET_RAD = 0.0

GOAL_TOLERANCE_M = 0.10       # "close enough" to a waypoint
TURN_IN_PLACE_THRESHOLD = 0.35  # rad (~20deg) -- above this, stop and turn first
MAX_LINEAR_SPEED = 0.15       # m/s -- deliberately slow, this is a small room
MAX_ANGULAR_SPEED = 0.8       # rad/s
KP_HEADING = 1.5

CONTROL_PERIOD_S = 0.1   # 10 Hz control loop
DETECT_EVERY_N_TICKS = 3  # run YOLO ~every 3rd control tick (~3.3 Hz) -- inference
                           # is the slow part, don't let it stall steering updates

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
        self.tick_count = 0

        self.duck_sightings = []  # list of (world_x, world_y)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.control_timer = self.create_timer(CONTROL_PERIOD_S, self.control_tick)

        if not nav_only:
            self._init_vision()
        else:
            self.get_logger().info("--nav-only: skipping camera/model, navigation loop only")

    def _init_vision(self):
        import cv2
        from ultralytics import YOLO

        self.cv2 = cv2
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

        if self.done:
            self.cmd_pub.publish(Twist())  # stay stopped
            return

        if not self.nav_only and self.tick_count % DETECT_EVERY_N_TICKS == 0:
            self._detect_tick()

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

    def _detect_tick(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn("Frame grab failed")
            return

        results = self.model(frame, conf=CONF_THRESHOLD, verbose=False)
        boxes = results[0].boxes
        if boxes is None or len(boxes) == 0:
            return

        # take the largest/most confident box if more than one hit
        box = boxes[boxes.conf.argmax()]
        x1, y1, x2, y2 = box.xyxy[0].tolist()
        bbox_height_px = y2 - y1
        bbox_center_x = (x1 + x2) / 2.0

        if bbox_height_px <= 1:
            return

        distance = (DUCK_HEIGHT_M * self.fx) / bbox_height_px

        # Pixel offset from image center -> bearing angle. Camera mount has
        # ~0 x/y offset from base_link (see stereo_depth_argus.launch.py),
        # just faces the same forward direction, so camera position/heading
        # ~= robot's own -- no extra transform needed beyond this angle.
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

    def _report(self):
        if not self.duck_sightings:
            self.get_logger().info("=== REPORT: loop complete, no duck sighted ===")
            return

        avg_x = sum(p[0] for p in self.duck_sightings) / len(self.duck_sightings)
        avg_y = sum(p[1] for p in self.duck_sightings) / len(self.duck_sightings)
        cell_x = max(0, min(int(ROOM_SIZE_M / GRID_CELL_M) - 1, int(avg_x // GRID_CELL_M)))
        cell_y = max(0, min(int(ROOM_SIZE_M / GRID_CELL_M) - 1, int(avg_y // GRID_CELL_M)))

        self.get_logger().info(
            f"=== REPORT: {len(self.duck_sightings)} sighting(s), "
            f"averaged position ({avg_x:.2f}, {avg_y:.2f}) -> grid cell ({cell_x}, {cell_y}) ==="
        )


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
