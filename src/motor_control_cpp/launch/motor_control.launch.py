from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='motor_control_cpp',
            executable='motor_control_node',
            name='motor_control_node',
            output='screen'
        ),
    ])