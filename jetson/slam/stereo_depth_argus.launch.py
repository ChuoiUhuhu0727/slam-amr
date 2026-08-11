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

Nothing in this file has been run yet -- first-pass implementation from reading
image_pipeline's documented topic/remap conventions, not from a live test on
this workspace. Expect at least one plugin-name or param-name mismatch on first
launch; check `ros2 component types stereo_image_proc` for the exact registered
plugin names on this ROS distro if DisparityNode/PointCloudNode fail to load.
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

# Placeholder: camera looks straight forward, no tilt/roll in the mount.
# qx,qy,qz,qw for "no rotation" is (0,0,0,1). If the camera is physically
# tilted (e.g. angled down to see the floor sooner), this needs the real
# rotation too, not just translation -- flag this to vịt if obstacles end up
# spatially wrong even after the translation is fixed.
BASE_TO_LEFT_CAM_QUAT = (0.0, 0.0, 0.0, 1.0)


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
            '--child-frame-id', 'left_camera_optical_frame',
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
            # explicit, so this literally matches point_cloud_node's input below
            # instead of relying on both nodes defaulting to the same name
            ('left/disparity', 'stereo/disparity'),
        ],
        parameters=[{
            # Added 2026-08-11: /stereo/points2 never published, no error --
            # confirmed all 4 inputs (image_0/1, camera_info_0/1) were flowing
            # fine individually (20-30Hz each via `ros2 topic hz`), so the gap
            # is downstream, inside this node's own left/right time sync.
            # Default exact-time sync needs matching timestamps, but these two
            # ArgusMonoNode streams have real inter-camera jitter -- the SAME
            # jitter visual_slam_node itself already needed
            # sync_matching_threshold_ms=50.0 to tolerate (see
            # visual_slam_argus_rectified.launch.py). approximate_sync lets
            # this node's own (separate) synchronizer tolerate it too.
            # NOT YET CONFIRMED this fixes it -- next thing to check if
            # /stereo/disparity still doesn't publish after this.
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
            ('left/disparity', 'stereo/disparity'),
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
