"""Bridges /odom (nav_msgs/Odometry, published by the ESP32 over micro-ROS) into the
odom -> base_link TF that Nav2's costmaps and controller actually read robot pose from.

The ESP32 firmware (esp32/motor_f1/main/motor_f1.c) only publishes the Odometry
message, it doesn't broadcast TF -- micro-ROS TF broadcasting is heavier to add
firmware-side, and this project's split keeps ESP32 to hardware-facing pub/sub only.
This node is the thin Jetson-side piece that fills that gap, native ROS2, no docker.

Without this, Nav2 has no way to answer "where is base_link in the odom frame" and
will fail with "Could not get robot pose" as soon as a goal is sent.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class OdomToTf(Node):
    def __init__(self):
        super().__init__('odom_to_tf')
        self.broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(Odometry, '/odom', self.on_odom, 10)

    def on_odom(self, msg: Odometry):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = msg.header.frame_id       # "odom"
        t.child_frame_id = msg.child_frame_id         # "base_link"
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.broadcaster.sendTransform(t)


def main():
    rclpy.init()
    rclpy.spin(OdomToTf())


if __name__ == '__main__':
    main()
