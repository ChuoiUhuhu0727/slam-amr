#!/usr/bin/env python3
"""Quick straight-line drive test: publishes /cmd_vel and prints /esp32_diag +
/odom heading together in one terminal. Run for a few seconds, then Ctrl+C or
let it auto-stop.

Usage: python3 drive_test.py [speed] [duration_sec]
Defaults: speed=0.2 m/s, duration=5s
"""
import sys
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class DriveTest(Node):
    def __init__(self, speed, duration):
        super().__init__('drive_test')
        self.speed = speed
        self.duration = duration
        self.latest_diag = "(no /esp32_diag yet)"
        self.latest_heading = None

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(String, '/esp32_diag', self.diag_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

        self.start_time = self.get_clock().now()
        self.cmd_timer = self.create_timer(0.1, self.publish_cmd)   # 10Hz
        self.print_timer = self.create_timer(0.5, self.print_status)  # 2Hz

    def elapsed(self):
        return (self.get_clock().now() - self.start_time).nanoseconds / 1e9

    def diag_cb(self, msg):
        self.latest_diag = msg.data

    def odom_cb(self, msg):
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.latest_heading = math.degrees(math.atan2(siny_cosp, cosy_cosp))

    def publish_cmd(self):
        t = self.elapsed()
        msg = Twist()
        if t < self.duration:
            msg.linear.x = self.speed
        else:
            msg.linear.x = 0.0
        self.cmd_pub.publish(msg)
        if t >= self.duration + 1.0:
            self.get_logger().info("Done. Stopping.")
            rclpy.shutdown()

    def print_status(self):
        t = self.elapsed()
        heading_str = f"{self.latest_heading:.1f} deg" if self.latest_heading is not None else "N/A"
        print(f"[t={t:5.1f}s] heading={heading_str:>10} | {self.latest_diag}")


def main():
    speed = float(sys.argv[1]) if len(sys.argv) > 1 else 0.2
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0

    rclpy.init()
    node = DriveTest(speed, duration)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        try:
            node.cmd_pub.publish(Twist())  # zero out on exit, safety
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
