#!/usr/bin/env python3

import cv2 as cv

import rclpy
from rclpy.node import Node

import numpy as np
from typing import List
from collections import namedtuple

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
        self.instrinsic_matrix = np.zeros((4, 4))

        self.global_transform = np.eye(4)
        self.color_prev = None
        self.depth_prev = None

        self.load_config("")

        # publisher/subscribers
        tss = TimeSynchronizer([Subscriber(self, Image, "/camera/depth/image_rect_raw"),
                       Subscriber(self, Image, "/camera/color/image_raw")], queue_size=10)
        tss.registerCallback(self.visual_odom_callback)

        # visualizing pose chain
        self.poses_viz_pub_ = self.create_publisher(MarkerArray, 'poses_viz', 10)
        self.traj_viz_pub_ = self.create_publisher(Marker, 'traj_viz', 10)

        # TODO add a frame id here
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
            self.depth_prev = depth_img
            return
        
        kps1, desc1 = self.get_features(self.color_prev, self.detector)
        kps2, desc2 = self.get_features(color_img, self.detector)

        matches = self.get_matches(desc1, desc2, self.detector)
        
        relative_transform = self.motion_estimate(kps1, kps2, matches, depth_img)
        global_transform = global_transform @ relative_transform
        
        self.visualize_trajectory(global_transform)

        self.color_prev = color_img
        self.depth_prev = depth_img


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
    
    def motion_estimate(self, kp_tprev, kp_t, matches, depth_t_prev):

        #TODO should depth image be for tprev or t?
        kp_tprev_idx = np.array([ kp_tprev[m.queryIdx].pt for m in matches])
        kp_t_idx = np.array([ kp_t[m.trainIdx].pt for m in matches])

        assert kp_tprev_idx.shape == (len(matches), 2)

        world_pts = np.zeros((len(matches), 3))

        '''
        Projecting World to Camera & vice versa:
        x = fX / Z -> X = xZ / f
        y = fY / Z -> Y = yZ / f

        Note 
        '''

        #TODO depth clipping???
        Z = depth_t_prev[kp_tprev_idx]
        c = self.intrinsic_matrix[0:1, 2]
        f = np.array([self.intrinsic_matrix[0, 0], self.intrinsic_matrix[1, 1]])

        world_pts[:, 0:2] = Z * (kp_tprev_idx - c) / f
        world_pts[2] = Z

        success, rot_est, t_est = cv.solvePnPRansac(world_pts, kp_t_idx, self.instrinsic_matrix)

        transform = np.eye(4)
            
        if success:
            transform[0:3, 0:3] = cv.Rodrigues(rot_est)[0]
            transform[0:3, -1] - t_est

        else:
            self.get_logger().warning("PnP Pose Estimate Failed!")

        return transform


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

