#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np
from typing import List

from message_filters import TimeSynchronizer, Subscriber
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from transforms3d.quaternions import mat2quat


class VisualOdometry(Node):
    """
    The class that handles the VO pipeline
    """
    def __init__(self):
        super().__init__('visual_odom')
        """
        """

        self.detector = None
        self.instrinsic_matrix = None

        self.global_transform = np.eye(4)
        self.color_prev = None

        self.load_config("")

        # publisher/subscribers
        tss = TimeSynchronizer([Subscriber(self, Image, "/camera/depth/image_rect_raw"),
                       Subscriber(self, Image, "/camera/color/image_raw")], queue_size=10)
        tss.registerCallback(self.visual_odom_callback)

        # visualizing pose chain
        self.poses_viz_pub_ = self.create_publisher(MarkerArray, 'poses_viz', 10)
        self.traj_viz_pub_ = self.create_publisher(Marker, 'traj_viz', 10)

        self.poses_markers = MarkerArray()

        self.traj_marker = Marker()
        self.traj_marker.type = 4
        self.traj_marker.scale.x = 1.0
        self.traj_marker.color.a = 1.0
        self.traj_marker.color.r = 1.0

        self.visualize_trajectory(self.global_transform)


    def visual_odom_callback(self, color_img: Image, depth_img: Image):
        """
        run VO pipeline
        """

        if self.color_prev is None:
            self.color_prev = color_img
            return
        
        kps1, desc1 = self.get_features(self.color_prev, self.detector)
        kps2, desc2 = self.get_features(color_img, self.detector)

        matches = self.get_matches(desc1, desc2, self.detector)
        
        relative_transform = self.motion_estimate(kps1, kps2, matches, depth_img)
        global_transform = self.trajectory[-1] @ relative_transform
        
        self.trajectory.append(global_transform)
        self.visualize_trajectory(self.trajectory)

        self.color_prev = color_img


    def get_features(self, image, detector):
        """
        get features from image
        """
        pass
    
    def get_matches(self, descriptors1, descriptors2, detector):
        """"
        match features between two frames
        """
        
        pass
    
    def motion_estimate(self, kp_t, kp_tnext, matches, depth_img):
        pass


    def visualize_trajectory(self, transform):
        """_summary_

        Args:
            trajectory (List[np.ndarray]): _description_
        """

        ## TODO visualize point cloud from pose estimate

        rot, trans = transform[:3, :3], transform[3, :3]

        quat = mat2quat(rot)

        pose = Marker()
        pose.type = 0
        pose.pose.position.x = trans[0]
        pose.pose.position.y = trans[1]
        pose.pose.position.z = trans[2]
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        pose.scale.x = 1.0
        pose.scale.y = 1.0
        pose.scale.z = 1.0
        pose.color.a = 1.0
        pose.color.g = 1.0
        self.poses_markers.markers.append(pose)

        point = Point()
        point.x = trans[0]
        point.y = trans[1]
        point.z = trans[2]
        self.traj_marker.points.append(point)

        self.poses_viz_pub_.publish(self.poses_markers)
        self.traj_viz_pub_.publish(self.traj_marker)


    def load_config(self, path):
        pass

def main(args=None):

    rclpy.init(args=args)
    vo_node = VisualOdometry()
    rclpy.spin(vo_node)

    # Destroy the node explicitly
    vo_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
