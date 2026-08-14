#!/usr/bin/env python3
"""Subscribe to /imu_raw and log every message to CSV.

Ctrl+C to stop — each row is flushed to disk immediately on arrival,
so nothing is lost when the process is interrupted.

Raw int16 values only (no unit conversion) — MPU6050 wakes into its
power-on-reset default full-scale range: accel +/-2g (16384 LSB/g),
gyro +/-250 deg/s (131 LSB/(deg/s)). Divide by those to get physical
units during processing.

Usage:
    python3 log_imu_raw.py [output.csv]
    (default output: imu_raw_<timestamp>.csv in the current dir)
"""
import csv
import re
import sys
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

IMU_RE = re.compile(
    r"ax=(?P<ax>-?\d+) ay=(?P<ay>-?\d+) az=(?P<az>-?\d+) "
    r"gx=(?P<gx>-?\d+) gy=(?P<gy>-?\d+) gz=(?P<gz>-?\d+)"
)


class ImuLogger(Node):
    def __init__(self, csv_path):
        super().__init__('imu_raw_logger')
        self.csv_file = open(csv_path, 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(
            ['t_sec', 'ax_raw', 'ay_raw', 'az_raw',
             'gx_raw', 'gy_raw', 'gz_raw', 'raw'])
        self.csv_file.flush()
        self.t0 = self.get_clock().now()
        self.row_count = 0
        self.csv_path = csv_path
        self.create_subscription(String, '/imu_raw', self.on_msg, 10)
        self.get_logger().info(f'Logging /imu_raw -> {csv_path} (Ctrl+C to stop)')

    def on_msg(self, msg):
        t = (self.get_clock().now() - self.t0).nanoseconds / 1e9
        m = IMU_RE.search(msg.data)
        if m:
            row = [f'{t:.3f}', m['ax'], m['ay'], m['az'],
                   m['gx'], m['gy'], m['gz'], msg.data]
        else:
            row = [f'{t:.3f}', '', '', '', '', '', '', msg.data]
        self.writer.writerow(row)
        self.csv_file.flush()
        self.row_count += 1

    def close(self):
        self.csv_file.close()


def main():
    csv_path = sys.argv[1] if len(sys.argv) > 1 else \
        datetime.now().strftime('imu_raw_%Y%m%d_%H%M%S.csv')
    rclpy.init()
    node = ImuLogger(csv_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        print(f'\nSaved {node.row_count} rows -> {node.csv_path}')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
