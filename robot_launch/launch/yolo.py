import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

# Python 虚拟环境路径，可按实际部署环境修改
VENV = os.path.expanduser("~/robot_venv")


def generate_launch_description():
    return LaunchDescription(
        [
            # 全局环境节点
            Node(
                package="robot_vision",
                executable="camera_publisher",
            ),
            # 虚拟环境节点
            ExecuteProcess(
                cmd=[
                    "bash",
                    "-c",
                    f'"source {VENV}/bin/activate && '
                    f'export PYTHONPATH="{VENV}/lib/python3.10/site-packages:$PYTHONPATH" && '
                    'DISPLAY=:1.0 ros2 run robot_vision yolo_detection"',
                ],
                shell=True,
            ),
        ]
    )
