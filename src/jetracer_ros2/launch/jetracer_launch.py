import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
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

    return LaunchDescription([
        jetracer_node,
        ekf_node,
        base_imu_link_tf,
        base_link_tf,
        laser_frame_tf
    ])
