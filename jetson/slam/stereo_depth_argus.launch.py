"""Week 6 Phase A2 -- stereo depth for Nav2 obstacle avoidance.

Publishes a PointCloud2 (/stereo/points2) from the same rectified stereo pair
visual_slam_argus_rectified.launch.py already produces. Deliberately does NOT
re-launch ArgusMonoNode/RectifyNode here -- the physical cameras are already
claimed by whichever visual_slam launch file is running, and a second process
can't open the same camera_id. Run this ALONGSIDE visual_slam_argus_rectified,
not instead of it. Subscribes to the plain ROS2 topics that launch file already
publishes (visual_slam/image_0, camera_info_0, image_1, camera_info_1) -- no
NITROS zero-copy across this process boundary, but keeps this file independently
restartable while iterating, matching how nav2_launch.py is already decoupled
from the Isaac ROS container.

Pipeline: DisparityNode (left/right rectified + camera_info -> disparity image)
-> PointCloudNode (disparity + camera_info -> PointCloud2 in
left_camera_optical_frame). Uses the native ROS2 `stereo_image_proc` package
(CPU, from ros-humble-stereo-image-proc) -- confirmed 2026-08-11 that
isaac_ros_stereo_image_proc is NOT built in this workspace (`ros2 pkg list`
came back empty), and getting an Isaac ROS package built from source has
historically been a multi-error slog on this project (see micro-ROS/Isaac ROS
bring-up history in project memory) -- not worth it just to get GPU disparity
when CPU is enough to prove the pipeline works. Revisit swapping to the GPU
version later ONLY if disparity computation turns out to be a real latency
bottleneck (measure first, don't assume).

Install if missing: `sudo apt install ros-humble-stereo-image-proc`.

Also publishes the static TF base_link -> left_camera_optical_frame, needed for
Nav2 to transform the point cloud (published in camera frame) into odom. THE
OFFSET BELOW IS A PLACEHOLDER (0,0,0) -- NOT YET MEASURED on the real chassis.
Obstacle positions in the costmap will be wrong (off by however far the camera
actually sits from base_link) until this is replaced with a real tape-measure
reading, same discipline as the wheelbase/wheel-diameter constants in
motor_f1.c. Measure once, hardcode here (matches this project's existing
convention -- no live calibration step for static mechanical offsets).

CONFIRMED WORKING on real hardware 2026-08-11: /stereo/points2 publishes at
~1-1.5Hz (much slower than disparity's ~10Hz -- point cloud generation is the
expensive step on CPU; revisit GPU version per the note above only if this
rate turns out to be a real bottleneck for Nav2, not preemptively). Getting
here took a real debugging chain worth remembering: all 4 raw inputs checked
individually first (`ros2 topic hz`, all fine) before suspecting the node
itself; visual_slam's container silently dying mid-session (closed by
accident) looked identical to a real bug until re-launching it fixed nothing
by itself; approximate_sync was a reasonable, correctly-reasoned guess that
turned out NOT to be the actual cause; the real fix only came from `ros2 node
info <node>` showing the ACTUAL live topic names instead of trusting this
file's own remap dict -- a silently-wrong remap doesn't error in ROS2, it just
does nothing.
"""
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

# Measured 2026-08-11 (tape measure, after the forward-direction flip in
# motor_f1.c -- camera now roughly faces the same way as base_link's forward
# axis, which is why this is a small/near-zero offset instead of needing a
# 180-degree rotation): x ~0 (camera sits almost directly over the wheel
# axle), y=0 (camera roughly centered left-right on the chassis), z=0.14m
# (14cm up from the ground to the lens).
BASE_TO_LEFT_CAM_XYZ = (0.0, 0.0, 0.14)

# Fixed 2026-08-13: this was (0,0,0,1) i.e. no rotation, which is wrong even
# with zero tilt/roll -- base_link (REP103: x-forward, y-left, z-up) and
# left_camera_optical_frame (REP103 optical convention: x-right, y-down,
# z-forward-into-scene) are different axis conventions, not just possibly
# offset by mount tilt. Symptom that exposed this: /stereo/points2 had real,
# non-NaN points (confirmed via a direct rclpy read, ~11932/921600) but
# nothing rendered in RViz2 even after ruling out Color Transformer/point
# size/view framing -- because points 1-3m in front of the camera (large Z
# in the optical frame) were being placed 1-3m in the air (mapped straight
# to base_link's Z) instead of in front (base_link's X), well outside the
# default ground-level view. This is the standard fixed rotation for a
# forward-facing, untilted optical-frame camera -- if the mount is later
# found to have real tilt/roll, that would compose with this, not replace it.
BASE_TO_LEFT_CAM_QUAT = (-0.5, 0.5, -0.5, 0.5)


def generate_launch_description():
    static_tf_base_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_left_camera_tf',
        arguments=[
            '--x', str(BASE_TO_LEFT_CAM_XYZ[0]),
            '--y', str(BASE_TO_LEFT_CAM_XYZ[1]),
            '--z', str(BASE_TO_LEFT_CAM_XYZ[2]),
            '--qx', str(BASE_TO_LEFT_CAM_QUAT[0]),
            '--qy', str(BASE_TO_LEFT_CAM_QUAT[1]),
            '--qz', str(BASE_TO_LEFT_CAM_QUAT[2]),
            '--qw', str(BASE_TO_LEFT_CAM_QUAT[3]),
            '--frame-id', 'base_link',
            # NOT 'left_camera_optical_frame' -- confirmed live 2026-08-13 via a tf2
            # MessageFilter warning on /stereo/points2 ("discarding message because the
            # queue is full" for frame "camera_optical") that ArgusMonoNode stamps its
            # published images/camera_info with frame_id "camera_optical" by default,
            # not the name used elsewhere in this repo's launch files/TF tree. Renamed
            # this TF's child frame to match reality instead of chasing ArgusMonoNode's
            # own default (visual_slam_node doesn't care about this string -- it uses
            # separate base_frame/camera_optical_frames params for its own pose output,
            # unrelated to what's stamped on the raw image/camera_info messages).
            '--child-frame-id', 'camera_optical',
        ],
    )

    disparity_node = ComposableNode(
        package='stereo_image_proc',
        plugin='stereo_image_proc::DisparityNode',
        name='disparity_node',
        remappings=[
            ('left/image_rect', 'visual_slam/image_0'),
            ('left/camera_info', 'visual_slam/camera_info_0'),
            ('right/image_rect', 'visual_slam/image_1'),
            ('right/camera_info', 'visual_slam/camera_info_1'),
            # NOT remapped, deliberately: confirmed live 2026-08-11 via
            # `ros2 node info /disparity_node` that this node's real internal
            # output topic is plain `disparity`, not `left/disparity` -- the
            # remap that used to be here (`left/disparity` -> `stereo/
            # disparity`) was a silent no-op (ROS2 doesn't error when you
            # remap a topic name the node doesn't actually have) and cost
            # real debugging time chasing sync/QoS/encoding theories before
            # `ros2 node info` gave the real answer. point_cloud_node below
            # has the identical situation and also defaults to plain
            # `/disparity`, so the two nodes end up correctly wired to each
            # other anyway -- left unremapped on purpose, not "fixed", to
            # avoid reintroducing the same silent-no-op risk.
        ],
        parameters=[{
            # Tried first as the fix for the no-output investigation above;
            # turned out NOT to be the actual cause (the dead remap was).
            # Left in place anyway -- still the technically correct setting
            # given the real inter-camera jitter between the two ArgusMonoNode
            # streams (the same jitter visual_slam_node needed
            # sync_matching_threshold_ms=50.0 to tolerate).
            'approximate_sync': True,
            'queue_size': 10,
        }],
    )

    point_cloud_node = ComposableNode(
        package='stereo_image_proc',
        plugin='stereo_image_proc::PointCloudNode',
        name='point_cloud_node',
        remappings=[
            ('left/image_rect_color', 'visual_slam/image_0'),
            ('left/camera_info', 'visual_slam/camera_info_0'),
            ('right/camera_info', 'visual_slam/camera_info_1'),
            # NOT remapped -- see disparity_node's comment above, identical
            # situation, both default to plain `disparity` which matches.
            ('points2', 'stereo/points2'),
        ],
        parameters=[{
            'approximate_sync': True,
            'queue_size': 10,
        }],
    )

    container = ComposableNodeContainer(
        name='stereo_depth_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[disparity_node, point_cloud_node],
        output='screen',
    )

    return LaunchDescription([static_tf_base_to_camera, container])
