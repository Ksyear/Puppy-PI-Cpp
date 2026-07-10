from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # LiDAR 드라이버(ros2 launch peripherals lidar.launch.py)가 먼저 떠 있어야 한다
    map_name = LaunchConfiguration('map_name', default='cpp_map')

    return LaunchDescription([
        DeclareLaunchArgument('map_name', default_value='cpp_map',
                              description='저장할 지도 이름 (.pgm/.yaml)'),
        Node(
            package='lidar_mapping_cpp',
            executable='lidar_mapping',
            name='lidar_mapping',
            output='screen',
            parameters=[{
                'map_name': map_name,
            }],
        ),
    ])
