from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # 全局环境节点
        Node(
            package='robot_vision',
            executable='camera_publisher',
        ),
        
        # 虚拟环境节点
        ExecuteProcess(
            cmd=[
                'bash', '-c',
                '"source /home/orin/robot_venv/bin/activate && '
                'export PYTHONPATH="/home/orin/robot_venv/lib/python3.10/site-packages:$PYTHONPATH" && '
                'DISPLAY=:1.0 ros2 run robot_vision yolo_detection"'
            ],
            shell=True,
        ),
    ])

