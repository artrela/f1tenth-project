#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import transforms3d
import math

class PoseModifier(Node):
    def __init__(self):
        super().__init__('pose_modifier')
        self.subscription = self.create_subscription(
            PoseStamped,
            '/vicon',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(PoseStamped, '/vicon_new', 10)

    def listener_callback(self, msg):

        msg.header.frame_id = 'origin'  # Modify the frame ID

        quat = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        euler = transforms3d.euler.quat2euler(quat, axes='sxyz')
        new_roll = euler[0] - math.pi/2
        new_pitch = -(euler[1] + math.pi/2)
        new_quat = transforms3d.euler.euler2quat(new_roll, new_pitch, euler[2], axes='sxyz')

        msg.pose.position.z = 2.0
        msg.pose.orientation.x = new_quat[0]
        msg.pose.orientation.y = new_quat[1]
        msg.pose.orientation.z = new_quat[2]
        msg.pose.orientation.w = new_quat[3]

        self.publisher.publish(msg)
        self.get_logger().info(f'Published PoseStamped with frame_id {msg.header.frame_id}')

def main(args=None):
    rclpy.init(args=args)
    pose_modifier = PoseModifier()
    rclpy.spin(pose_modifier)
    pose_modifier.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
