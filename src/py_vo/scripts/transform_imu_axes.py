#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class IMUTransformNode(Node):
    def __init__(self):
        super().__init__('imu_transform_node')
        self.subscription = self.create_subscription(
            Imu,
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
        transformed_msg = Imu()
        transformed_msg.header = msg.header
        # transformed_msg.header.frame_id = "imu_enu"

        # If initial data is (0, -g, 0) and needs to be (0, 0, -g),
        # then apply a rotation about x-axis and correct the sign

        # Orientation conversion, with hypothetical rotation if needed (example)
        transformed_msg.orientation.x = msg.orientation.x
        transformed_msg.orientation.y = msg.orientation.z
        transformed_msg.orientation.z = msg.orientation.y
        transformed_msg.orientation.w = msg.orientation.w

        # Angular velocity conversion
        transformed_msg.angular_velocity.x = msg.angular_velocity.x
        transformed_msg.angular_velocity.y = msg.angular_velocity.z
        transformed_msg.angular_velocity.z = msg.angular_velocity.y

        # Linear acceleration conversion
        transformed_msg.linear_acceleration.x = msg.linear_acceleration.x
        transformed_msg.linear_acceleration.y = msg.linear_acceleration.z
        transformed_msg.linear_acceleration.z = -msg.linear_acceleration.y  # Inverting to align gravity correctly

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