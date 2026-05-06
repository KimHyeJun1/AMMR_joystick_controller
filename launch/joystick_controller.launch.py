import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

config_file = PathJoinSubstitution([
    FindPackageShare("joystick_controller"),
    "config",
    "joystick_controller.yaml"
])

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joystick_controller',
            executable='joystick_controller_node',
            name='joystick_controller_node',
            parameters=[config_file],
            output='screen',
            # arguments=['--ros-args', '--log-level', 'DEBUG'] # 로그 필요할 때 주석해제 후 로그 볼 수 있음.
            )
    ])