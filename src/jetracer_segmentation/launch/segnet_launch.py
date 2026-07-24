import os
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_name = 'jetracer_segmentation'

    ensure_script = os.path.join(
        get_package_prefix(pkg_name), 'lib', pkg_name, 'ensure_model.sh'
    )

    # 1. Define Launch Configurations
    model = LaunchConfiguration('model')
    model_dir = LaunchConfiguration('model_dir')
    input_topic = LaunchConfiguration('input_topic')
    overlay_alpha = LaunchConfiguration('overlay_alpha')

    # 2. Declare Launch Arguments
    declare_model = DeclareLaunchArgument(
        'model',
        default_value='fcn-resnet18-voc-320x320',
        description='Pretrained model name (see ensure_model.sh for the pinned set)'
    )

    declare_model_dir = DeclareLaunchArgument(
        'model_dir',
        default_value='/data/models',
        description='Persistent model/engine cache (docker-compose volume)'
    )

    declare_input_topic = DeclareLaunchArgument(
        'input_topic',
        default_value='/csi_cam_0/image_raw',
        description='Input image topic'
    )

    declare_overlay_alpha = DeclareLaunchArgument(
        'overlay_alpha',
        default_value='120.0',
        description='Overlay blend alpha (0-255)'
    )

    # 3. Download (sha256-pinned) + build the TensorRT engine unless cached.
    # First run takes minutes on the Nano; afterwards this exits immediately.
    ensure_model = ExecuteProcess(
        cmd=[ensure_script, model, model_dir],
        output='screen'
    )

    # 4. The segmentation node; topic interface mirrors ros_deep_learning's
    # segnet (image_in -> overlay / color_mask / class_mask under /segnet)
    segnet_node = Node(
        package=pkg_name,
        executable='segnet_node',
        name='segnet',
        namespace='segnet',
        output='screen',
        remappings=[('image_in', input_topic)],
        parameters=[{
            'engine_path': ParameterValue(
                [model_dir, '/', model, '/fcn_resnet18.engine'], value_type=str),
            'labels_path': ParameterValue(
                [model_dir, '/', model, '/classes.txt'], value_type=str),
            'colors_path': ParameterValue(
                [model_dir, '/', model, '/colors.txt'], value_type=str),
            'overlay_alpha': ParameterValue(overlay_alpha, value_type=float),
        }]
    )

    # 5. Start the node only after the engine exists (same event pattern as
    # csi_camera_launch.py)
    start_node_after_model = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=ensure_model,
            on_exit=[segnet_node]
        )
    )

    return LaunchDescription([
        declare_model,
        declare_model_dir,
        declare_input_topic,
        declare_overlay_alpha,
        ensure_model,
        start_node_after_model,
    ])
