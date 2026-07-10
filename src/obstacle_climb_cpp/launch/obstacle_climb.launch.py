from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 사전 조건: puppy_control 실행 중 + usb_cam 실행 중
    #   ros2 launch peripherals usb_cam.launch.py
    return LaunchDescription([
        Node(
            package='obstacle_climb_cpp',
            executable='obstacle_climb',
            name='obstacle_climb',
            output='screen',
            parameters=[{
                # 로봇의 lab_config.yaml 에서 캘리브레이션된 값으로 교체할 것
                'lab_min': [0, 150, 130],
                'lab_max': [255, 255, 255],
                'climb_action': 'up_stairs_2cm.d6ac',
                'autostart': True,
            }],
        ),
    ])
