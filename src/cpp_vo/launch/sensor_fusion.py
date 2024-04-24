from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='cpp_vo',
        #     executable='vo_node',
        #     name='visual_odometry_node'
        # ),
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
            package='py_vo',
            executable='transform_imu_axes.py',
            name='visual_odometry_node'
        ),
        Node(
            package='realsense2_camera',
            namespace='camera', 
            executable='realsense2_camera_node',
            name='realsense_camera',
            parameters=[
                {'pointcloud.enable': True},
                {'enable_gyro': True},
                {'enable_accel': True},
                {'align_depth.enable': True},
                {'unite_imu_method': 2},
                # enables higher fps https://support.intelrealsense.com/hc/en-us/community/posts/360050831934-Higher-RGB-and-Depth-FPS
                # {'rgb_camera.color_profile': '848x480'},
                # {"depth_fps": 60},
                # {"color_fps": 60},
                # https://github.com/IntelRealSense/realsense-ros/issues/599
            ]
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='sensor_fusion_node',
            parameters=[
                {"imu0": "/imu/data_enu"}, #madgwick
                # {"imu0": "/camera/imu"}, #raw
                {"odom0": "/visual_odometry/pose"},
                # # x, y, yaw
                # {"imu0_config": [False, False, False,
                #                 False, False, False,
                #                 False, False, False,
                #                 False, False, True,
                #                 True, True, False]},
                # {"odom0_config": [True, True, False,
                #                 False, False, True,
                #                 False, False, False,
                #                 False, False, False,
                #                 False, False, False]},
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
                {'world_frame': 'origin'},
                {'dynamic_process_noise_covariance': True},
                {'process_noise_covariance':
                    [1.0, 0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
                    0,    1.0,  0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
                    0,    0,    1.0,  0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
                    0,    0,    0,    0.01, 0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
                    0,    0,    0,    0,    0.01,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
                    0,    0,    0,    0,    0,    0.01,    0,     0,     0,    0,    0,    0,    0,    0,    0,
                    0,    0,    0,    0,    0,    0,    0.01,     0,     0,    0,    0,    0,    0,    0,    0,
                    0,    0,    0,    0,    0,    0,    0,     0.01,     0,    0,    0,    0,    0,    0,    0,
                    0,    0,    0,    0,    0,    0,    0,     0,     0.01,    0,    0,    0,    0,    0,    0,
                    0,    0,    0,    0,    0,    0,    0,     0,     0,    0.01,    0,    0,    0,    0,    0,
                    0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0.01,    0,    0,    0,    0,
                    0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0.01,    0,    0,    0,
                    0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0.01,    0,    0,
                    0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0.01,    0,
                    0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0.01]
                }
            ]
        )
    ]
)