import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 카메라 영상 전송만 단독 실행 (조종 없이 FPV 확인용).
    # 사전 조건: ros2 launch peripherals usb_cam.launch.py
    config = os.path.join(
        get_package_share_directory('puppy_vr_control'),
        'config',
        'vr_control_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='puppy_vr_control',
            executable='camera_udp_sender',
            name='camera_udp_sender',
            output='screen',
            parameters=[config],
        ),
    ])
