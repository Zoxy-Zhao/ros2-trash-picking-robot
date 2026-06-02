from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='robot_arm',
            executable='arm_control',
        ),


        Node(
            package='robot_serial',
            executable='serial_send',
        ),
        
    ])

