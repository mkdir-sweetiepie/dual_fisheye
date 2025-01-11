import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('dual_fisheye'),
        'config',
        'dual_fisheye_calibration.yaml'
    )
    
    return LaunchDescription([
        Node(
            package='dual_fisheye',
            executable='image_dual_undistort_node',
            name='image_dual_undistort_node',
            parameters=[config],
            output='screen'
        )
    ])