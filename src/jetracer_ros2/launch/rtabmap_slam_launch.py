import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Package directories
    jetracer_dir = get_package_share_directory('jetracer_ros2')
    sllidar_dir = get_package_share_directory('sllidar_ros2')

    # Paths to launch files and configs
    jetracer_launch_path = os.path.join(jetracer_dir, 'launch', 'jetracer_launch.py')
    camera_launch_path = os.path.join(jetracer_dir, 'launch', 'csi_camera_launch.py')
    sllidar_launch_path = os.path.join(sllidar_dir, 'launch', 'sllidar_a1_launch.py')
    filter_config_path = os.path.join(jetracer_dir, 'config', 'chassis_filter.yaml')
    rtabmap_params_path = os.path.join(jetracer_dir, 'config', 'rtabmap_params.yaml')

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

        # 4. CSI Camera Launch (loop-closure imagery for RTAB-Map)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(camera_launch_path)
        ),

        # 4b. Scan deskewing: A1 rev takes ~133 ms, up to ~19 deg smear/scan at speed (bench 2026-07-13)
        Node(
            package='rtabmap_util',
            executable='lidar_deskewing',
            name='lidar_deskewing',
            output='screen',
            parameters=[{
                'fixed_frame_id': 'odom',
                'wait_for_transform': 0.2,   # EKF TF is 30 Hz; default 0.01 drops scans
                'slerp': True,               # start/end interpolation, cheap on the Nano CPU
            }],
            remappings=[('input_scan', '/scan_filtered')]
            # publishes PointCloud2 on /scan_filtered/deskewed
        ),

        # 5. RTAB-Map SLAM. Replaces slam_toolbox as the map->odom publisher —
        # never run this together with slam_launch.py.
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[rtabmap_params_path],
            remappings=[
                ('rgb/image', '/csi_cam_0/image_raw'),
                ('rgb/camera_info', '/csi_cam_0/camera_info'),
                ('scan_cloud', '/scan_filtered/deskewed'),  # deskewed cloud replaces raw scan
                ('odom', '/odom'),
                ('grid_map', '/map')  # Nav2 expects /map
            ],
            # Delete old DB on start while experimenting; remove -d to keep
            # mapping incrementally across runs (RTABMAP.md §5).
            arguments=['-d']
        )
    ])
