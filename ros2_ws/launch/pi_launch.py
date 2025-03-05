from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_ros',
            executable='camera_node',
            name='pix_camera',
            parameters=[{"format": "YUYV", "width": 320, "height": 240}]
        ),
        Node(
            package='pix_driver',
            executable='pix_node',
            name='pix_control',
            parameters=[{"turn_offset": 1.0, "diff_ratio": 0.65}]
        ),
    ])

