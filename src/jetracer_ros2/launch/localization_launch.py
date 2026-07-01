import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    jetracer_dir = get_package_share_directory('jetracer_ros2')
    nav2_bringup  = get_package_share_directory('nav2_bringup')
    sllidar_dir   = get_package_share_directory('sllidar_ros2')

    # ── Launch arguments ────────────────────────────────────────────────────
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(jetracer_dir, 'maps', 'jetracer_map.yaml'),
        description='Full path to the saved map .yaml file')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true')

    # ── Config file paths ───────────────────────────────────────────────────
    jetracer_launch = os.path.join(jetracer_dir, 'launch', 'jetracer_launch.py')
    sllidar_launch  = os.path.join(sllidar_dir,  'launch', 'sllidar_a1_launch.py')
    filter_cfg      = os.path.join(jetracer_dir, 'config', 'chassis_filter.yaml')
    amcl_cfg        = os.path.join(jetracer_dir, 'config', 'amcl_params.yaml')
    nav2_cfg        = os.path.join(jetracer_dir, 'config', 'nav2_params.yaml')

    # ── 1. Base robot (motors, IMU, EKF, TFs) ──────────────────────────────
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(jetracer_launch))

    # ── 2. LiDAR ───────────────────────────────────────────────────────────
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sllidar_launch),
        launch_arguments={
            'serial_port': '/dev/ttyACM1',
            'frame_id':    'laser_frame'
        }.items())

    # ── 3. Laser filter (chassis crop) ─────────────────────────────────────
    laser_filter = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='scan_filter_chain',
        output='screen',
        parameters=[filter_cfg],
        remappings=[
            ('scan',          '/scan'),
            ('scan_filtered', '/scan_filtered')
        ])

    # ── 4. Map server — serves the saved occupancy grid ────────────────────
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            amcl_cfg,
            {'yaml_filename': LaunchConfiguration('map')}
        ])

    # ── 5. AMCL — particle-filter localizer ────────────────────────────────
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_cfg])

    # ── 6. Lifecycle manager for map_server + amcl ─────────────────────────
    # These two nodes need lifecycle management to come up correctly
    lifecycle_manager_loc = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'autostart': True},
            {'node_names': ['map_server', 'amcl']}
        ])

    # ── 7. Nav2 navigation stack (planner, controller, BT, behaviours) ─────
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file':  nav2_cfg
        }.items())

    return LaunchDescription([
        map_arg,
        use_sim_time_arg,
        robot,
        lidar,
        laser_filter,
        map_server,
        amcl,
        lifecycle_manager_loc,
        navigation,
    ])
