import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('Nova2_camera_calibration_ros2'),
        'config',
        # 'param.yaml'
    )
    return LaunchDescription([
        Node(
            package='Nova2_camera_calibration_ros2',
            namespace='calibration',
            executable='calibration',
            remappings=[('/input_img', '/camera/color/image_raw'),
                        ('/input_caminfo', '/camera/color/camera_info')],
            # parameters=[config]
        ),
    ])