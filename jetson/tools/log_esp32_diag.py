#!/usr/bin/env python3
"""Subscribe to /esp32_diag and log every message to CSV.

Ctrl+C to stop — each row is flushed to disk immediately on arrival,
so nothing is lost when the process is interrupted.

Usage:
    python3 log_esp32_diag.py [output.csv]
    (default output: esp32_diag_<timestamp>.csv in the current dir)
"""
import csv
import re
import sys
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

DIAG_RE = re.compile(
    r"reset=(?P<reset>\S+) RPM L=(?P<rpm_l>-?\d+\.?\d*) R=(?P<rpm_r>-?\d+\.?\d*) "
    r"PWM L=(?P<pwm_l>-?\d+\.?\d*) R=(?P<pwm_r>-?\d+\.?\d*)"
)


class DiagLogger(Node):
    def __init__(self, csv_path):
        super().__init__('esp32_diag_logger')
        self.csv_file = open(csv_path, 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(
            ['t_sec', 'reset_reason', 'rpm_l', 'rpm_r', 'pwm_l', 'pwm_r', 'raw'])
        self.csv_file.flush()
        self.t0 = self.get_clock().now()
        self.row_count = 0
        self.csv_path = csv_path
        self.create_subscription(String, '/esp32_diag', self.on_msg, 10)
        self.get_logger().info(f'Logging /esp32_diag -> {csv_path} (Ctrl+C to stop)')

    def on_msg(self, msg):
        t = (self.get_clock().now() - self.t0).nanoseconds / 1e9
        m = DIAG_RE.search(msg.data)
        if m:
            row = [f'{t:.3f}', m['reset'], m['rpm_l'], m['rpm_r'],
                   m['pwm_l'], m['pwm_r'], msg.data]
        else:
            row = [f'{t:.3f}', '', '', '', '', '', msg.data]
        self.writer.writerow(row)
        self.csv_file.flush()
        self.row_count += 1

    def close(self):
        self.csv_file.close()


def main():
    csv_path = sys.argv[1] if len(sys.argv) > 1 else \
        datetime.now().strftime('esp32_diag_%Y%m%d_%H%M%S.csv')
    rclpy.init()
    node = DiagLogger(csv_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        print(f'\nSaved {node.row_count} rows -> {node.csv_path}')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
