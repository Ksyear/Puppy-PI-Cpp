import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 파라미터 파일: config/vr_control_params.yaml
    config = os.path.join(
        get_package_share_directory('puppy_vr_control'),
        'config',
        'vr_control_params.yaml'
    )

    use_camera = LaunchConfiguration('use_camera', default='true')
    debug = LaunchConfiguration('debug', default='false')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_camera', default_value='true',
            description='카메라 UDP 전송 노드(camera_udp_sender) 실행 여부'),
        DeclareLaunchArgument(
            'debug', default_value='false',
            description='수신/발행 값을 콘솔에 출력'),

        # VR(Quest) UDP 조이스틱 수신 → /puppy_control/velocity/autogait
        Node(
            package='puppy_vr_control',
            executable='vr_udp_teleop',
            name='vr_udp_teleop',
            output='screen',
            parameters=[config, {'debug': debug}],
        ),

        # 카메라 JPEG → UDP 청크 전송 (usb_cam.launch.py 가 먼저 실행돼 있어야 함)
        Node(
            package='puppy_vr_control',
            executable='camera_udp_sender',
            name='camera_udp_sender',
            output='screen',
            parameters=[config],
            condition=IfCondition(use_camera),
        ),

        # 배터리/Wi-Fi/링크 상태 → UDP 1Hz (Quest HUD 표시용)
        Node(
            package='puppy_vr_control',
            executable='robot_status_sender',
            name='robot_status_sender',
            output='screen',
            parameters=[config],
        ),
    ])
