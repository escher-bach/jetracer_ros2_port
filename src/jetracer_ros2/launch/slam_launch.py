import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Package directories
    jetracer_dir = get_package_share_directory('jetracer_ros2')
    sllidar_dir = get_package_share_directory('sllidar_ros2')

    # Paths to launch files and configs
    jetracer_launch_path = os.path.join(jetracer_dir, 'launch', 'jetracer_launch.py')
    sllidar_launch_path = os.path.join(sllidar_dir, 'launch', 'sllidar_a1_launch.py')
    filter_config_path = os.path.join(jetracer_dir, 'config', 'chassis_filter.yaml')
    slam_config_path = os.path.join(jetracer_dir, 'config', 'slam_toolbox_params.yaml')

    return LaunchDescription([
        # 1. Base Robot Launch (Motors, IMU, EKF, TFs)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(jetracer_launch_path)
        ),

        # 2. LiDAR Launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sllidar_launch_path),
            launch_arguments={
                'serial_port': '/dev/ttyACM1',
                'frame_id': 'laser_frame'
            }.items()
        ),

        # 3. Laser Filter Node (to crop out the robot chassis)
        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='scan_filter_chain',
            output='screen',
            parameters=[filter_config_path],
            remappings=[
                ('scan', '/scan'),
                ('scan_filtered', '/scan_filtered')
            ]
        ),

        # 4. SLAM Toolbox Online Async Node (Mapping)
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_config_path]
        )
    ])
