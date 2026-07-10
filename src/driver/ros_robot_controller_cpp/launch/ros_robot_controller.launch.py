from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 원본 ros_robot_controller.launch.py 와 동일 구성 (패키지만 C++판으로 교체)
    imu_frame = LaunchConfiguration('imu_frame', default='imu_link')
    imu_frame_arg = DeclareLaunchArgument('imu_frame', default_value=imu_frame)

    ros_robot_controller_node = Node(
        package='ros_robot_controller_cpp',
        executable='ros_robot_controller',
        output='screen',
        parameters=[{'imu_frame': imu_frame}]
    )

    return LaunchDescription([
        imu_frame_arg,
        ros_robot_controller_node
    ])
