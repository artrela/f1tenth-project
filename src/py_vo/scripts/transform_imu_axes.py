#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import transforms3d
import math

class IMUTransformNode(Node):
    def __init__(self):
        super().__init__('imu_transform_node')
        self.subscription = self.create_subscription(
            Imu,
            # '/camera/imu',
            '/imu/data',
            self.imu_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.publisher = self.create_publisher(
            Imu,
            '/imu/data_enu',
            10)

    def imu_callback(self, msg):
        # Transform the IMU data to properly orient gravity along the z-axis negatively in ENU

        quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        euler = transforms3d.euler.quat2euler(quat, axes='sxyz')
        new_roll = euler[0] - math.pi/2
        new_pitch = -(euler[1] + math.pi/2)
        new_quat = transforms3d.euler.euler2quat(new_roll, new_pitch, euler[2], axes='sxyz')

        transformed_msg = Imu()
        transformed_msg.header = msg.header
        transformed_msg.orientation.x = new_quat[0]
        transformed_msg.orientation.y = new_quat[1]
        transformed_msg.orientation.z = new_quat[2]
        transformed_msg.orientation.w = new_quat[3]

        # Angular velocity conversion
        transformed_msg.angular_velocity.x =  msg.angular_velocity.z
        transformed_msg.angular_velocity.y =  msg.angular_velocity.x
        transformed_msg.angular_velocity.z = -msg.angular_velocity.y
        # transformed_msg.angular_velocity.x = -msg.angular_velocity.x
        # transformed_msg.angular_velocity.y =  msg.angular_velocity.y
        # transformed_msg.angular_velocity.z = -msg.angular_velocity.z

        # # Linear acceleration conversion
        # transformed_msg.linear_acceleration.x =  msg.linear_acceleration.z
        # transformed_msg.linear_acceleration.y =  msg.linear_acceleration.x
        # transformed_msg.linear_acceleration.z = -msg.linear_acceleration.y
        transformed_msg.linear_acceleration.x =  msg.linear_acceleration.x
        transformed_msg.linear_acceleration.y = -msg.linear_acceleration.y
        transformed_msg.linear_acceleration.z =  msg.linear_acceleration.z

        # covariance
        transformed_msg.linear_acceleration_covariance = [0.0001791136179641669, 0.0, 0.0, 0.0, 0.0001492359730431701, 0.0, 0.0, 0.0, 0.00017867067674247886]
        transformed_msg.angular_velocity_covariance = [2.9881742333011597e-06, 0.0, 0.0, 0.0, 2.773540802644703e-06, 0.0, 0.0, 0.0, 5.279744031021535e-06]

        # Publish the transformed IMU data
        self.publisher.publish(transformed_msg)



def main(args=None):
    rclpy.init(args=args)
    imu_transform_node = IMUTransformNode()
    rclpy.spin(imu_transform_node)
    imu_transform_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()