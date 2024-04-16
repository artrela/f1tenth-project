from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[{'yaml_filename': '/home/sridevi/Documents/f1tenth-project/src/cpp_vo/config/tepper_locker_all.yaml'}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', '/home/sridevi/Documents/f1tenth-project/src/cpp_vo/config/vo_rviz_config.rviz']
        ),
    ])
