from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='image_tools',
            executable='cam2image',
            name='testCam'
        ),
        Node(
            package='roadline_tracker',
            executable='roadline_tracker',
            name='rdln_tracker'
        ),
    ])
