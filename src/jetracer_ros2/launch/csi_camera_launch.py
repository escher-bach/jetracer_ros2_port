import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_name = 'jetracer_ros2'

    # 1. Resolve Default Path for the YAML file
    try:
        pkg_share_dir = get_package_share_directory(pkg_name)
        default_cam_info_url = 'file://' + os.path.join(
            pkg_share_dir, 'config', 'cam_640x480.yaml'
        )
    except Exception:
        # Fallback if the package isn't built/sourced yet
        default_cam_info_url = ''

    # 2. Define Launch Configurations
    cam_name = LaunchConfiguration('cam_name')
    frame_id = LaunchConfiguration('frame_id')
    camera_info_url = LaunchConfiguration('camera_info_url')
    socket_path = LaunchConfiguration('socket_path')
    jpeg_quality = LaunchConfiguration('jpeg_quality')

    # 3. Declare Launch Arguments
    declare_cam_name = DeclareLaunchArgument(
        'cam_name', 
        default_value='csi_cam_0',
        description='Namespace and node name for the camera'
    )
    
    declare_frame_id = DeclareLaunchArgument(
        'frame_id', 
        default_value='camera_frame',
        description='Camera TF frame ID'
    )
    
    declare_camera_info_url = DeclareLaunchArgument(
        'camera_info_url', 
        default_value=default_cam_info_url,
        description='URL to the camera calibration YAML'
    )
    
    declare_socket_path = DeclareLaunchArgument(
        'socket_path', 
        default_value='/tmp/gst_shm_socket',
        description='Path to the shared memory socket from the primary pipeline'
    )

    declare_jpeg_quality = DeclareLaunchArgument(
        'jpeg_quality', 
        default_value='80',
        description='JPEG compression quality for the compressed image transport (1-100)'
    )

    # 4. Construct the GStreamer Pipeline String with ParameterValue
    # This explicitly forces ROS 2 to evaluate the list as a single concatenated string
    gscam_config = ParameterValue([
        'shmsrc socket-path=', socket_path,
        ' is-live=true do-timestamp=true ! ',
        'video/x-raw, format=(string)RGBA, width=(int)680, height=(int)420, framerate=(fraction)30/1 ! ',
        'videoconvert'
    ], value_type=str)

    # 5. Define the Node
    gscam_node = Node(
        package='gscam',
        executable='gscam_node',
        name=cam_name,
        namespace=cam_name,
        output='screen',
        remappings=[
            ('camera/image_raw', 'image_raw'),
            ('camera/image_raw/compressed', 'image_raw/compressed'),
            ('camera/image_raw/compressedDepth', 'image_raw/compressedDepth'),
            ('camera/image_raw/theora', 'image_raw/theora'),
            ('camera/camera_info', 'camera_info')
        ],
        parameters=[{
            'gscam_config': gscam_config,
            'camera_name': cam_name,
            'sync_sink': False,
            'preroll': False,
            'image_encoding': 'rgb8',
            'frame_id': frame_id,
            'camera_info_url': camera_info_url,
            'camera/image_raw.compressed.jpeg_quality': jpeg_quality
        }]
    )

    # 6. Producer pipeline command running via nsenter
    # We delete the old socket first to avoid race conditions where the consumer connects to an old dead socket
    producer_cmd = ExecuteProcess(
        cmd=[
            'nsenter', '-t', '1', '-m', '-u', '-n', '-i', 'sh', '-c',
            ['rm -f ', socket_path, ' && gst-launch-1.0 nvarguscamerasrc sensor-id=0 ! '
             "'video/x-raw(memory:NVMM), width=(int)1280, height=(int)720, format=(string)NV12, framerate=(fraction)30/1' ! "
             'nvvidconv flip-method=0 ! '
             "'video/x-raw(memory:NVMM), width=(int)680, height=(int)420, format=(string)RGBA' ! "
             'nvvidconv ! '
             "'video/x-raw, format=(string)RGBA' ! "
             'queue leaky=downstream max-size-buffers=5 ! '
             'shmsink socket-path=', socket_path, ' shm-size=40000000 wait-for-connection=false']
        ],
        output='screen'
    )

    # 7. Wait for the socket file to be recreated by the producer
    wait_for_socket_cmd = ExecuteProcess(
        cmd=['bash', '-c', ['sleep 1 && while [ ! -S ', socket_path, ' ]; do sleep 0.1; done']],
        output='screen'
    )

    # 8. Start gscam_node only after the socket is ready
    start_gscam_after_socket = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=wait_for_socket_cmd,
            on_exit=[gscam_node]
        )
    )

    return LaunchDescription([
        declare_cam_name,
        declare_frame_id,
        declare_camera_info_url,
        declare_socket_path,
        declare_jpeg_quality,
        producer_cmd,
        wait_for_socket_cmd,
        start_gscam_after_socket
    ])