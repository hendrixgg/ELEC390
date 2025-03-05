from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joystick_controller',
            executable='joystick_controller_node',
            name='joy_control'
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_hw'
        ),
    ])

