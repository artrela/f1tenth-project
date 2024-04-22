#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math


class OriginPublisher(Node):
    def __init__(self):
        super().__init__('origin_publisher')
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.timer = self.create_timer(0.1, self.publish_origin)

    def publish_origin(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'origin'
        t.transform.translation.x = -0.6288322893676047
        t.transform.translation.y = 3.736094076704183
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)
        print("Sending transform...")


def main(args=None):
    rclpy.init(args=args)

    origin_publisher = OriginPublisher()

    rclpy.spin(origin_publisher)

    origin_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
