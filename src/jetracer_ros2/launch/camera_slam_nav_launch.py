import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_jetracer_ros2 = get_package_share_directory('jetracer_ros2')

    # Launch file paths
    camera_launch_path = os.path.join(pkg_jetracer_ros2, 'launch', 'csi_camera_launch.py')
    slam_launch_path = os.path.join(pkg_jetracer_ros2, 'launch', 'slam_launch.py')
    nav_launch_path = os.path.join(pkg_jetracer_ros2, 'launch', 'nav_launch.py')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    # Include Camera launch
    camera_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_launch_path)
    )

    # Include SLAM launch
    slam_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_path),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Include Navigation launch
    nav_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav_launch_path),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        camera_cmd,
        slam_cmd,
        nav_cmd
    ])
