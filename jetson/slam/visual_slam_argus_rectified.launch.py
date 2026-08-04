"""Rectified-mode variant of visual_slam_argus.launch.py -- built specifically to test whether
the raw-mode (rectified_images:=False) camera_info interpretation is the source of the
vo_pose scale bug (see README "Lessons Learned" 2026-08-04, Part 3, and PR #27/#28 for the
failed camera_info P-matrix Tx patch on the raw-mode path).

Pipeline: 2x ArgusMonoNode (raw distorted images, camera_info carrying the REAL rectification
R/P from cv2.stereoRectify via npz_to_camera_info_yaml_rectified.py) -> 2x RectifyNode
(isaac_ros_image_proc, GPU rectification via VPI, applies that R/P to actually warp the image)
-> VisualSlamNode with rectified_images:=True.

Deliberately kept identical to visual_slam_argus.launch.py in every other respect (same TF from
the same raw R/T, same sync/jitter thresholds) so this is a single-variable comparison: only
raw-vs-rectified changes. Do NOT "helpfully" adjust the TF to a rectified-frame baseline here --
that would confound this specific test (see the PR #27/#28 finding that combining TF baseline
with a second baseline source made scale worse, not better).

Requires: npz_to_camera_info_yaml_rectified.py already run (writes left_rect.yaml/right_rect.yaml)
and isaac_ros_image_proc built in the workspace alongside isaac_ros_argus_camera/isaac_ros_visual_slam.
"""
import numpy as np

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

CALIB_PATH = "/workspaces/isaac_ros-dev/stereo_calibration.npz"
LEFT_CAMERA_INFO_URL = "file:///workspaces/isaac_ros-dev/camera_info/left_rect.yaml"
RIGHT_CAMERA_INFO_URL = "file:///workspaces/isaac_ros-dev/camera_info/right_rect.yaml"

# Must match the sensor mode (mode 4) capture resolution -- NOT RectifyNode's own default
# (1280x800). A mismatch here would reproduce the exact "dimensions do not correspond to
# camera resolution" rejection already hit once during the raw-mode bring-up.
RECT_WIDTH = 1280
RECT_HEIGHT = 720


def rotmat_to_quat(R):
    """R is near-identity for this rig (aligned stereo mount) -- safe to use the simple
    trace-based formula without the branching needed for arbitrary/degenerate rotations."""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    s = np.sqrt(tr + 1.0) * 2
    qw = 0.25 * s
    qx = (R[2, 1] - R[1, 2]) / s
    qy = (R[0, 2] - R[2, 0]) / s
    qz = (R[1, 0] - R[0, 1]) / s
    return qx, qy, qz, qw


def generate_launch_description():
    calib = np.load(CALIB_PATH)
    R, T = calib["R"], calib["T"].flatten()
    qx, qy, qz, qw = rotmat_to_quat(R)

    static_tf_left_to_right = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='left_to_right_camera_tf',
        arguments=[
            '--x', str(T[0]), '--y', str(T[1]), '--z', str(T[2]),
            '--qx', str(qx), '--qy', str(qy), '--qz', str(qz), '--qw', str(qw),
            '--frame-id', 'left_camera_optical_frame',
            '--child-frame-id', 'right_camera_optical_frame',
        ],
    )

    argus_left_node = ComposableNode(
        package='isaac_ros_argus_camera',
        plugin='nvidia::isaac_ros::argus::ArgusMonoNode',
        name='argus_mono_left',
        remappings=[
            ('left/image_raw', 'stereo/left/image_raw'),
            ('left/camera_info', 'stereo/left/camera_info'),
        ],
        parameters=[{
            'camera_id': 0,
            'module_id': -1,
            'mode': 4,
            'camera_info_url': LEFT_CAMERA_INFO_URL,
        }],
    )

    argus_right_node = ComposableNode(
        package='isaac_ros_argus_camera',
        plugin='nvidia::isaac_ros::argus::ArgusMonoNode',
        name='argus_mono_right',
        remappings=[
            ('left/image_raw', 'stereo/right/image_raw'),
            ('left/camera_info', 'stereo/right/camera_info'),
        ],
        parameters=[{
            'camera_id': 1,
            'module_id': -1,
            'mode': 4,
            'camera_info_url': RIGHT_CAMERA_INFO_URL,
        }],
    )

    rectify_left_node = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::RectifyNode',
        name='rectify_left',
        remappings=[
            ('image_raw', 'stereo/left/image_raw'),
            ('camera_info', 'stereo/left/camera_info'),
            ('image_rect', 'visual_slam/image_0'),
            ('camera_info_rect', 'visual_slam/camera_info_0'),
        ],
        parameters=[{
            'output_width': RECT_WIDTH,
            'output_height': RECT_HEIGHT,
        }],
    )

    rectify_right_node = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::RectifyNode',
        name='rectify_right',
        remappings=[
            ('image_raw', 'stereo/right/image_raw'),
            ('camera_info', 'stereo/right/camera_info'),
            ('image_rect', 'visual_slam/image_1'),
            ('camera_info_rect', 'visual_slam/camera_info_1'),
        ],
        parameters=[{
            'output_width': RECT_WIDTH,
            'output_height': RECT_HEIGHT,
        }],
    )

    visual_slam_node = ComposableNode(
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        name='visual_slam_node',
        parameters=[{
            'num_cameras': 2,
            'rectified_images': True,
            'sync_matching_threshold_ms': 50.0,
            'image_jitter_threshold_ms': 300.0,
            'base_frame': 'left_camera_optical_frame',
            'camera_optical_frames': [
                'left_camera_optical_frame',
                'right_camera_optical_frame',
            ],
            'enable_slam_visualization': True,
            'enable_landmarks_view': True,
            'enable_observations_view': True,
        }],
    )

    container = ComposableNodeContainer(
        name='argus_visual_slam_rectified_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            argus_left_node, argus_right_node,
            rectify_left_node, rectify_right_node,
            visual_slam_node,
        ],
        output='screen',
    )

    return LaunchDescription([static_tf_left_to_right, container])
