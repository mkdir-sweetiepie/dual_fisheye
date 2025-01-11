import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('dual_fisheye'),
        'config',
        'insta360air.yaml'
    )

    return LaunchDescription([
        Node(
            package='usb_camera',
            executable='usb_camera_node',
            name='usb_camera_node',
            parameters=[config_file]
        ),
        Node(
            package='dual_fisheye',
            executable='fisheye_image_control_node',
            name='fisheye_image_control_node',
            parameters=[config_file]
        )
    ])
