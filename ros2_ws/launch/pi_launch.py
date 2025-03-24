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
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='realsense',
            parameters=[
                {"enable_depth": True},
                {"enable_color": True},
                {"enable_infra": True},
                {"publish_tf": False},  # Disable transform publishing
                {"enable_gyro": False},  # Disable IMU (gyro)
                {"enable_accel": True},  # Disable IMU (accelerometer)
                {"enable_sync": True},  # Sync depth and color frames
                {"depth_module.depth_profile": "640x480x30"},
                {"rgb_camera.color_profile": "1280x720x30"},
                {"depth_module.infra_profile": "640x480x30"},
                ],
        ),
    ])
