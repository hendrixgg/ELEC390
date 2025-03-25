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
        # Node(
        #     package='driver',
        #     executable='driver',
        #     name='controller',
        #     parameters=[{"drive_power": 40.0,
        #                  "left_turn_angle": -20.0,
        #                  "pid_p": 0.1,
        #                  "pid_i": 0.2,
        #                  "pid_d": 0.05,
        #                  "intersection_time_ms": 300,
        #                  }]
        #     ),
        Node(
            package='vision',
            executable='roadline_tracker',
            name='roadline_tracker',
            parameters=[{"image_topic": "/pix_camera/image_raw",
                         "output_topic": "line_deviation"}]
            ),
        # Node(
        #     package='realsense2_camera',
        #     executable='realsense2_camera_node',
        #     name='realsense',
        #     parameters=[
        #         {"enable_depth": True},
        #         {"enable_color": True},
        #         {"enable_infra": True},
        #         {"publish_tf": False},  # Disable transform publishing
        #         {"enable_gyro": False},  # Disable IMU (gyro)
        #         {"enable_accel": True},  # Disable IMU (accelerometer)
        #         {"enable_sync": True},  # Sync depth and color frames
        #         {"depth_module.depth_profile": "640x480x30"},
        #         {"rgb_camera.color_profile": "1280x720x30"},
        #         {"depth_module.infra_profile": "640x480x30"},
        #         ],
        # ),
    ])
