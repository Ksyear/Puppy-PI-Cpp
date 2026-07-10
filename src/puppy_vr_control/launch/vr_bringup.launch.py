import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 부팅 자동실행용 통합 launch: 카메라 + VR 조종 + 영상 전송 + 상태 전송.
    # (로봇을 움직이는 puppy_control / ros_robot_controller 는 Hiwonder 기본
    #  서비스가 이미 부팅 시 띄운다 — 이 launch 는 VR 관련만 추가)
    # 모든 노드 respawn=True: 카메라 미연결 등으로 죽어도 자동 재시작.
    config = os.path.join(
        get_package_share_directory('puppy_vr_control'),
        'config',
        'vr_control_params.yaml'
    )
    usb_cam_params = os.path.join(
        get_package_share_directory('peripherals'),
        'config',
        'usb_cam_param.yaml'
    )

    return LaunchDescription([
        # 카메라 (원본 peripherals/usb_cam.launch.py 와 동일 노드/설정,
        # need_compile 환경변수 의존 없이 직접 실행)
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            output='screen',
            parameters=[usb_cam_params],
            respawn=True,
            respawn_delay=3.0,
        ),

        Node(
            package='puppy_vr_control',
            executable='vr_udp_teleop',
            name='vr_udp_teleop',
            output='screen',
            parameters=[config],
            respawn=True,
            respawn_delay=2.0,
        ),

        Node(
            package='puppy_vr_control',
            executable='camera_udp_sender',
            name='camera_udp_sender',
            output='screen',
            parameters=[config],
            respawn=True,
            respawn_delay=2.0,
        ),

        Node(
            package='puppy_vr_control',
            executable='robot_status_sender',
            name='robot_status_sender',
            output='screen',
            parameters=[config],
            respawn=True,
            respawn_delay=2.0,
        ),
    ])
