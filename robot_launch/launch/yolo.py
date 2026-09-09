from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = LaunchConfiguration('config')
    default = str(Path(get_package_share_directory('robot_launch')) / 'config' / 'robot.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default),
        DeclareLaunchArgument('serial', default_value='false'),
        Node(package='robot_vision', executable='camera_publisher', parameters=[config], output='screen'),
        Node(package='robot_vision', executable='yolo_detection', parameters=[config], output='screen'),
    ])
