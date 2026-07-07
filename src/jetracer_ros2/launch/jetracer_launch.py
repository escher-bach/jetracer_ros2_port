import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # The camera sits on a manually adjustable hinge, so its tilt is a
    # per-setup value, not a constant. Lock the hinge (mark/tape it), measure
    # the angle (see scripts/scan_overlay.py), and pass it here.
    # Positive = tilted down, radians.
    camera_pitch = LaunchConfiguration('camera_pitch')
    declare_camera_pitch = DeclareLaunchArgument(
        'camera_pitch',
        default_value='0.0',
        description='Camera downward tilt in radians (positive = down)'
    )

    pkg_dir = get_package_share_directory('jetracer_ros2')
    ekf_config_path = os.path.join(pkg_dir, 'config', 'ekf.yaml')

    jetracer_node = Node(
        package='jetracer_ros2',
        executable='jetracer_node',
        name='jetracer',
        output='screen',
        parameters=[{
            'port_name': '/dev/ttyACM0',
            'publish_odom_transform': False, # EKF will publish it
        }],
        remappings=[
            ('/odom', '/odom_raw'),
        ]
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path],
        remappings=[
            ('odometry/filtered', 'odom')
        ]
    )

    base_imu_link_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_to_imu',
        arguments=['0', '0', '0.02', '0', '0', '0', 'base_footprint', 'base_imu_link']
    )

    base_link_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_to_base_link',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link']
    )

    laser_frame_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_to_laser',
        arguments=['0', '0.0', '0.1', '3.14', '0.0', '0.0', 'base_footprint', 'laser_frame']
    )

    # CSI camera mount. x/z are from the CAD-derived URDF in JetRacer-ROS2
    # (jetracer_description), not measured on this car — verify with a ruler:
    # x = lens forward of the rear-axle/chassis midpoint, z = lens height above
    # ground. Argument order is x y z yaw pitch roll.
    camera_link_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_to_camera',
        arguments=['0.115', '0.0', '0.11', '0.0', camera_pitch, '0.0',
                   'base_footprint', 'camera_link']
    )

    # Optical frame (z forward, x right, y down) — REP 103 camera convention.
    # gscam stamps images with this frame; RTAB-Map and image_geometry expect it.
    camera_optical_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_to_camera_optical',
        arguments=['0', '0', '0', '-1.5707963', '0.0', '-1.5707963', 'camera_link', 'camera_link_optical']
    )

    return LaunchDescription([
        declare_camera_pitch,
        jetracer_node,
        ekf_node,
        base_imu_link_tf,
        base_link_tf,
        laser_frame_tf,
        camera_link_tf,
        camera_optical_tf
    ])
