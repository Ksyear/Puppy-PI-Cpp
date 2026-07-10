from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 원본 joystick_control.launch.py 대응.
    # C++판은 /dev/input/js0 을 직접 읽으므로 joy_node 는 필요 없다.
    return LaunchDescription([
        Node(
            package='peripherals_cpp',
            executable='remote_control_joystick',
            name='remote_control_joystick',
            output='screen',
        ),
    ])
