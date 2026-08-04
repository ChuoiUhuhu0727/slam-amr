"""Combined launch: 2x ArgusMonoNode (left/right IMX219 via isaac_ros_argus_camera) feeding
directly into isaac_ros_visual_slam's VisualSlamNode, using our own stereo calibration.

rectified_images is False: raw distorted images + accurate camera_info (K, D from our
calibration) go straight into cuVSLAM, which rectifies on GPU internally -- no separate
rectification node needed.

base_frame is set to left_camera_optical_frame (not a real robot base_link) since we don't
have a robot URDF/extrinsics yet -- this is a first correctness check ("does a trajectory
appear at all"), not a navigation-ready frame setup. Revisit once the robot has a proper
base_link -> camera extrinsic calibration.

The left/right camera optical frame TF is computed directly from stereo_calibrate.py's R, T
(the same calibration feeding camera_info), not guessed -- so stereo geometry is internally
consistent between camera_info and TF.

Must run INSIDE the Isaac ROS dev container (needs isaac_ros_argus_camera and
isaac_ros_visual_slam built in the workspace). Requires stereo_calibration.npz already
converted to camera_info YAML via jetson/calibration/npz_to_camera_info_yaml.py.
"""
import numpy as np

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

CALIB_PATH = "/home/chuoichiuchiu/slam-amr/stereo_calibration.npz"
LEFT_CAMERA_INFO_URL = "file:///workspaces/isaac_ros-dev/camera_info/left.yaml"
RIGHT_CAMERA_INFO_URL = "file:///workspaces/isaac_ros-dev/camera_info/right.yaml"


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
            ('left/image_raw', 'visual_slam/image_0'),
            ('left/camera_info', 'visual_slam/camera_info_0'),
        ],
        parameters=[{
            'camera_id': 0,
            'module_id': -1,
            'camera_info_url': LEFT_CAMERA_INFO_URL,
        }],
    )

    argus_right_node = ComposableNode(
        package='isaac_ros_argus_camera',
        plugin='nvidia::isaac_ros::argus::ArgusMonoNode',
        name='argus_mono_right',
        remappings=[
            ('left/image_raw', 'visual_slam/image_1'),
            ('left/camera_info', 'visual_slam/camera_info_1'),
        ],
        parameters=[{
            'camera_id': 1,
            'module_id': -1,
            'camera_info_url': RIGHT_CAMERA_INFO_URL,
        }],
    )

    visual_slam_node = ComposableNode(
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        name='visual_slam_node',
        parameters=[{
            'num_cameras': 2,
            'rectified_images': False,
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
        name='argus_visual_slam_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[argus_left_node, argus_right_node, visual_slam_node],
        output='screen',
    )

    return LaunchDescription([static_tf_left_to_right, container])
