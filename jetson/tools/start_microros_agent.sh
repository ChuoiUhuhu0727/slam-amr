#!/bin/bash
# One command to (re)start the micro-ROS agent reliably, instead of the
# manual 2-step dance (esptool reset -> start agent) figured out live
# 2026-08-28. The agent gets stuck at "logger setup" forever if the ESP32's
# own connection state is stale relative to a freshly (re)started agent --
# happens after every Jetson reboot, and sometimes after just restarting
# the agent alone. `esptool chip_id` forces a real hardware reset via the
# RTS pin (same mechanism idf.py monitor uses, but that needs a real TTY
# and can't run over a plain SSH command) -- cheap, read-only, always safe
# to run even when the agent was already working fine.
set -e

source /opt/ros/humble/setup.bash
source ~/microros_ws/install/setup.bash
source ~/esp/esp-idf/export.sh > /dev/null 2>&1

echo "Resetting ESP32 via esptool (RTS pin toggle)..."
python3 -m esptool --port /dev/ttyUSB0 chip_id > /dev/null 2>&1
sleep 1

echo "Starting micro-ROS agent..."
exec ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
