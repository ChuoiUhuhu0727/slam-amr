"""Nav2 closed loop -- Week 3 MVP: click a goal in RViz2, robot drives there on /odom.

Deliberately hand-rolled instead of including nav2_bringup's navigation_launch.py:
that stock launch pairs with localization_launch.py (map_server + amcl), which this
project doesn't use yet (no map -- see nav2_params.yaml header for why). Only the
four lifecycle nodes nav2_params.yaml actually configures are started here.

Runs natively on the Jetson's ROS2 Humble install (ros-humble-navigation2 +
ros-humble-nav2-bringup), NOT inside the Isaac ROS docker container -- Nav2 only
needs /odom (already published natively by the ESP32/micro-ROS side), so this is
fully decoupled from the Isaac ROS environment.
"""
import os
import sys

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

THIS_DIR = os.path.dirname(__file__)
PARAMS_FILE = os.path.join(THIS_DIR, 'nav2_params.yaml')
ODOM_TO_TF_SCRIPT = os.path.join(THIS_DIR, 'odom_to_tf.py')

LIFECYCLE_NODES = ['controller_server', 'planner_server', 'behavior_server', 'bt_navigator']


def generate_launch_description():
    nodes = [
        # Bridges the ESP32's /odom Odometry message into the odom->base_link TF
        # that costmaps/controller actually read pose from -- see odom_to_tf.py.
        # Not a colcon package (same convention as jetson/slam/), so run directly
        # by file path rather than via `ros2 run`.
        ExecuteProcess(
            cmd=[sys.executable, ODOM_TO_TF_SCRIPT],
            output='screen',
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[PARAMS_FILE],
            # DEBUG temporarily: goal test sends "Passing new path to controller" every
            # cycle but /cmd_vel never publishes a single message (confirmed live via
            # `ros2 topic echo /cmd_vel`, silent for 60s+) -- INFO level doesn't show why
            # computeVelocityCommands() isn't producing/publishing output. Drop back to
            # default (remove this arguments line) once the cause is found.
            arguments=['--ros-args', '--log-level', 'controller_server:=debug'],
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[PARAMS_FILE],
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[PARAMS_FILE],
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[PARAMS_FILE],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'autostart': True,
                'node_names': LIFECYCLE_NODES,
            }],
        ),
    ]
    return LaunchDescription(nodes)
