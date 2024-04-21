#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R

class FixedFrameBroadcaster(Node):
    def __init__(self):
        super().__init__('fixed_frame_broadcaster')
        self.broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(1.0, self.timer_callback)  # Adjust the timer as needed for the transform update rate

    def timer_callback(self):
        t = TransformStamped()

        # Fill in the header information
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'origin'
        t.child_frame_id = 'camera_imu_optical_frame'

        # Specify the transform details
        t.transform.translation.x = 0.0  # Change these values as needed
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        rotation = R.from_euler('xyz', [0, 0, 0], degrees=True)  # No rotation, as an example
        q = rotation.as_quat()  # Get quaternion from the rotation
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Broadcast the transform
        self.broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = FixedFrameBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass  # Handle Ctrl-C gracefully
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()