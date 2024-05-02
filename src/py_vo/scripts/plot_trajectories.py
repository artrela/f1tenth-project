#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt

class OdometrySubscriber(Node):
    def __init__(self):
        super().__init__('odometry_subscriber')
        # Subscribe to filtered pose estimate
        self.filter_subscription = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.filter_callback,
            10)

        # Subscribe to wheeled odom
        self.wheeled_odometry_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odometry_callback,
            10)

        # Subscribe to the ground truth pose
        self.gt_subscription = self.create_subscription(
            PoseStamped,
            '/vicon_new',
            self.gt_callback,
            10)

        # Open a file to save the odometry data and pose data
        self.pose_file = open("filtered_data.txt", "w")
        self.odom_file = open("wheeled_odom_data.txt", "w")
        self.gt_file = open("gt_data.txt", "w")

    def filter_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        # Write the position data to a file
        self.pose_file.write(f"{x},{y}\n")

    def gt_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        # Write the position data to a file
        self.gt_file.write(f"{x},{y}\n")

    def close_files(self):
        self.pose_file.close()
        self.odom_file.close()
        self.gt_file.close()

def main(args=None):
    rclpy.init(args=args)
    odometry_subscriber = OdometrySubscriber()
    try:
        rclpy.spin(odometry_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        odometry_subscriber.close_files()
        odometry_subscriber.destroy_node()
        rclpy.shutdown()

        # Plotting after the node has stopped
        x_data, y_data = [], []
        with open("odometry_data.txt", "r") as file:
            for line in file:
                x, y = map(float, line.strip().split(','))
                x_data.append(x)
                y_data.append(y)

        px_data, py_data = [], []
        with open("pose_data.txt", "r") as file:
            for line in file:
                x, y = map(float, line.strip().split(','))
                px_data.append(x)
                py_data.append(y)

        plt.plot(x_data, y_data, 'r-', label='Odometry')  # Red line for odometry
        plt.plot(px_data, py_data, 'b-', label='PoseStamped')  # Blue line for PoseStamped
        plt.title("Robot Positions (X, Y)")
        plt.xlabel("X position")
        plt.ylabel("Y position")
        plt.legend()
        plt.show()

if __name__ == '__main__':
    main()