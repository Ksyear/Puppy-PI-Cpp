import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('puppy_vr_control_py'),
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

        Node(
            package='puppy_vr_control_py',
            executable='vr_udp_teleop',
            name='vr_udp_teleop',
            output='screen',
            parameters=[config, {'debug': debug}],
        ),

        Node(
            package='puppy_vr_control_py',
            executable='camera_udp_sender',
            name='camera_udp_sender',
            output='screen',
            parameters=[config],
            condition=IfCondition(use_camera),
        ),

        # 배터리/Wi-Fi/링크 상태 → UDP 1Hz (Quest HUD 표시용)
        Node(
            package='puppy_vr_control_py',
            executable='robot_status_sender',
            name='robot_status_sender',
            output='screen',
            parameters=[config],
        ),
    ])
