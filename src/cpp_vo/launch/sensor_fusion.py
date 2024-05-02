from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cpp_vo',
            executable='vo_node',
            name='visual_odometry_node',
            parameters=[
                {"cov_x": 0.2},
                {"cov_y": 0.2},
                {"cov_yaw": 0.2}
            ]
        ),
        # Node(
        #     package='imu_filter_madgwick',
        #     executable='imu_filter_madgwick_node',
        #     name='madwick_filter_relay',
        #     parameters=[
        #         {"use_mag": False},
        #         {"remove_gravity_vector": True},
        #     ],
        #     remappings=[
        #         ('imu/data_raw', '/camera/camera/imu')
        #     ]
        # ),
        Node(
            package='py_vo',
            executable='fake_origin_frame.py',
            name='fake_origin_frame_node'
        ),
        Node(
            package='py_vo',
            executable='frame_link.py',
            name='frame_link_node'
        ),
        Node(
            package='py_vo',
            executable='transform_imu_axes.py',
            name='transform_imu_axes_node'
        ),
        Node(
            package='py_vo',
            executable='republish_vicon.py',
            name='republish_vicon_node'
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
                {'rgb_camera.enable_auto_exposure': False},
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
                # {"imu0": "/imu/data_enu"}, #madgwick
                # {"imu0": "/camera/imu"}, #raw
                {"odom0": "/visual_odometry/pose"},
                {"odom1": "/odom"},
                # # x, y, yaw
                # {"imu0_config": [False, False, False,
                #                 False, False, False,
                #                 False, False, False,
                #                 False, False, True,
                #                 True, True, False]},
                {"odom0_config": [True, True, False,
                                False, False, True,
                                False, False, False,
                                False, False, False,
                                False, False, False]},
                {"odom1_config": [True, True, False,
                                False, False, True,
                                False, False, False,
                                False, False, False,
                                False, False, False]},
                {"two_d_mode": True},
                # {"imu0_relative": True},
                {"odom0_relative": True},
                {"odom1_relative": True},
                {"imu0_remove_gravitational_acceleration": False},
                {"print_diagnostics": True},
                {'base_link_frame': 'camera_imu_optical_frame'},
                {'odom_frame': 'odom'},
                {'world_frame': 'origin'},
                {'dynamic_process_noise_covariance': True},
                {'process_noise_covariance':
                    [0.1, 0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
                    0,    0.1,  0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
                    0,    0,    0.1,  0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
                    0,    0,    0,    0.1,  0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
                    0,    0,    0,    0,    0.1,  0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
                    0,    0,    0,    0,    0,    0.1,  0,     0,     0,    0,    0,    0,    0,    0,    0,
                    0,    0,    0,    0,    0,    0,    0.01,     0,     0,    0,    0,    0,    0,    0,    0,
                    0,    0,    0,    0,    0,    0,    0,     0.01,     0,    0,    0,    0,    0,    0,    0,
                    0,    0,    0,    0,    0,    0,    0,     0,     0.01,    0,    0,    0,    0,    0,    0,
                    0,    0,    0,    0,    0,    0,    0,     0,     0,    .01,    0,    0,    0,    0,    0,
                    0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    .01,    0,    0,    0,    0,
                    0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    .01,    0,    0,    0,
                    0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    .01,    0,    0,
                    0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    .01,    0,
                    0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    .01]
                }
                # {'process_noise_covariance':
                #     [0.01, 0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
                #     0,    0.01,  0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
                #     0,    0,    0.01,  0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
                #     0,    0,    0,    0.01,  0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
                #     0,    0,    0,    0,    0.01,  0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
                #     0,    0,    0,    0,    0,    0.01,  0,     0,     0,    0,    0,    0,    0,    0,    0,
                #     0,    0,    0,    0,    0,    0,    0.01,     0,     0,    0,    0,    0,    0,    0,    0,
                #     0,    0,    0,    0,    0,    0,    0,     0.01,     0,    0,    0,    0,    0,    0,    0,
                #     0,    0,    0,    0,    0,    0,    0,     0,     0.01,    0,    0,    0,    0,    0,    0,
                #     0,    0,    0,    0,    0,    0,    0,     0,     0,    1.0,    0,    0,    0,    0,    0,
                #     0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    1.0,    0,    0,    0,    0,
                #     0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    1.0,    0,    0,    0,
                #     0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    1.0,    0,    0,
                #     0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    1.0,    0,
                #     0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    1.0]
                # }
            ]
        )
    ]
)