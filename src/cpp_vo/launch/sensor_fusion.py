from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cpp_vo',
            executable='vo_node',
            name='visual_odometry_node'
        ),
        Node(
            package='py_vo',
            executable='fake_origin_frame.py',
            name='visual_odometry_node'
        ),
        Node(
            package='py_vo',
            executable='frame_link.py',
            name='visual_odometry_node'
        ),
        Node(
            package='realsense2_camera',
            namespace='realsense', 
            executable='realsense2_camera_node',
            name='realsense_camera',
            parameters=[
                {'pointcloud.enable': True},
                {'enable_gyro': True},
                {'enable_accel': True},
                {'align_depth.enable': True},
                {'unite_imu_method': 2},
            ]
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='sensor_fusion_node',
            parameters=[
                {"imu0": "/realsense/imu"},
                {"odom0": "/visual_odometry/pose"},
                # x, y, yaw
                {"imu0_config": [False, False, False,
                                False, False, False,
                                False, False, False,
                                False, False, True,
                                True, True, False]},
                {"odom0_config": [True, True, False,
                                False, False, True,
                                False, False, False,
                                False, False, False,
                                False, False, False]},
                {"two_d_mode": True},
                {"imu0_relative": True},
                {"odom0_relative": True},
                {"imu0_remove_gravitational_acceleration": True},
                {"print_diagnostics": True},
                {'base_link_frame': 'camera_imu_optical_frame'},
                {'odom_frame': 'origin'},
                {'world_frame': 'origin'}
            ]
        )
    ])