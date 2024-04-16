#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class PoseSaver(Node):
    def __init__(self):
        super().__init__('pose_saver')
        self.subscriber = self.create_subscription(
            PoseStamped,
            'pf/viz/inferred_pose',
            self.pose_callback,
            10)
        self.first_pose_received = False
        self.first_pose = None

    def pose_callback(self, msg):
        if not self.first_pose_received:
            self.first_pose = msg
            self.first_pose_received = True
            self.get_logger().info(f'First pose saved: {self.first_pose.pose}')
            self.subscriber.destroy()

def main(args=None):
    rclpy.init(args=args)
    pose_saver = PoseSaver()
    try:
        rclpy.spin(pose_saver)
    except KeyboardInterrupt:
        pass  # Handle Ctrl-C gracefully
    finally:
        pose_saver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
