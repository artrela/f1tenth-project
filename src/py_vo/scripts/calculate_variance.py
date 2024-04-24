#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np

class IMUVarianceCalculator(Node):
    def __init__(self):
        super().__init__('imu_variance_calculator')
        self.subscription = self.create_subscription(
            Imu,
            'imu/data_enu',
            self.imu_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Initialize lists to store IMU data
        self.linear_accel_x = []
        self.linear_accel_y = []
        self.linear_accel_z = []
        self.angular_vel_x = []
        self.angular_vel_y = []
        self.angular_vel_z = []

    def imu_callback(self, msg):
        # Append new data
        self.linear_accel_x.append(msg.linear_acceleration.x)
        self.linear_accel_y.append(msg.linear_acceleration.y)
        self.linear_accel_z.append(msg.linear_acceleration.z)
        self.angular_vel_x.append(msg.angular_velocity.x)
        self.angular_vel_y.append(msg.angular_velocity.y)
        self.angular_vel_z.append(msg.angular_velocity.z)

        # Calculate variance if sufficient data is collected
        if len(self.linear_accel_x) >= 4000:
            self.calculate_variance()

    def calculate_variance(self):
        # Compute variance for each component
        variances = {
            'linear_acceleration_x': np.var(self.linear_accel_x),
            'linear_acceleration_y': np.var(self.linear_accel_y),
            'linear_acceleration_z': np.var(self.linear_accel_z),
            'angular_velocity_x': np.var(self.angular_vel_x),
            'angular_velocity_y': np.var(self.angular_vel_y),
            'angular_velocity_z': np.var(self.angular_vel_z)
        }

        # Print the variance and reset lists
        self.get_logger().info(f'Variances: {variances}')
        self.reset_data()

    def reset_data(self):
        # Reset the data lists
        self.linear_accel_x = []
        self.linear_accel_y = []
        self.linear_accel_z = []
        self.angular_vel_x = []
        self.angular_vel_y = []
        self.angular_vel_z = []

def main(args=None):
    rclpy.init(args=args)
    imu_variance_calculator = IMUVarianceCalculator()
    rclpy.spin(imu_variance_calculator)
    imu_variance_calculator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
