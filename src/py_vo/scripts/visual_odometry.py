#!/usr/bin/env python3

import cv2 as cv

import rclpy
import yaml
import cv2
import os

from rclpy.node import Node

import numpy as np
from typing import List
from collections import namedtuple

from message_filters import TimeSynchronizer, Subscriber, ApproximateTimeSynchronizer
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import Image
from transforms3d.quaternions import mat2quat
from cv_bridge import CvBridge, CvBridgeError
from ament_index_python.packages import get_package_share_directory



class VisualOdometry(Node):
    """
    The class that handles the VO pipeline
    """
    def __init__(self):
        super().__init__('visual_odom')
        """
        """

        # config params
        self.detector = None
        self.intrinsics = None
        self.load_config('vo_config.yaml')

        self.global_transform = np.eye(4)
        self.global_transform[:, -1] = 1

        self.color_prev = None
        self.depth_prev = None
        self.cv_bridge = CvBridge()

        # publisher/subscribers
        tss = ApproximateTimeSynchronizer([Subscriber(self, Image, "/camera/aligned_depth_to_color/image_raw"),
                       Subscriber(self, Image, "/camera/color/image_raw")], 10, 0.01)
        tss.registerCallback(self.visual_odom_callback)

        # visualize matches
        self.matches_pub_ = self.create_publisher(Image, 'matches_viz', 10)

        # visualizing pose chain
        self.poses_viz_pub_ = self.create_publisher(MarkerArray, 'poses_viz', 10)
        self.traj_viz_pub_ = self.create_publisher(Path, 'traj_viz', 10)

        # TODO add a frame id here
        self.poses_markers = MarkerArray()

        self.traj_marker = Path()
        self.traj_marker.header.frame_id = "origin"
        

        self.visualize_trajectory(self.global_transform)


    def visual_odom_callback(self, depth_msg: Image, color_msg: Image):
        """
        run VO pipeline
        """

        color_img = self.convert_image(color_msg)
        depth_img = self.convert_image(depth_msg, depth=True)

        # print(f"{color_img.shape=} | {depth_img.shape=}")

        if self.color_prev is None:
            self.color_prev = color_img
            self.depth_prev = depth_img
            return

        kps_prev, desc_prev = self.get_features(self.color_prev, self.detector)
        kps, desc = self.get_features(color_img, self.detector)

        matches = self.get_matches(desc_prev, desc, self.detector)
        self.draw_matches(self.color_prev, color_img, kps_prev, kps, matches)

        relative_transform = self.motion_estimate(kps_prev, kps, matches, self.depth_prev)
        self.global_transform = self.global_transform @ relative_transform
        
        self.visualize_trajectory(self.global_transform)

        self.color_prev = color_img
        self.depth_prev = depth_img


    def convert_image(self, image_msg, depth=False):
        if depth:
            img = self.cv_bridge.imgmsg_to_cv2(image_msg, "passthrough")
            img = np.array(img.data).reshape((image_msg.height, image_msg.width))
        else:
            img = self.cv_bridge.imgmsg_to_cv2(image_msg, "bgr8")

        return img

    def draw_matches(self, img1, img2, kps1, kps2, matches):
        match_img = cv2.drawMatchesKnn(img1, kps1, img2, kps2, matches[:15], None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
        self.matches_pub_.publish(self.cv_bridge.cv2_to_imgmsg(match_img, "bgr8"))

    def get_features(self, image, detector_type):
        """
        get features from image
        """
        if detector_type == 'sift':
            detector = cv2.SIFT_create()
        elif detector_type == 'orb':
            detector = cv2.ORB_create()

        keypoints, descriptors = detector.detectAndCompute(image, None)

        # each row in descriptor maps to a keypoint
        # descriptor: numpy array
        # keypoints: list of cv2.keypoints objects
        return keypoints, descriptors
    

    def get_matches(self, desc1, desc2, detector, k=2, dist_thesh=0.75):
        """"
        match features between two frames
        """
        # brute-force matching
        if detector == 'sift':
            bf = cv2.BFMatcher(cv2.NORM_L2, crossCheck=False)
        elif detector == 'orb':
            bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)

        # knn match
        matches = bf.knnMatch(desc1, desc2, k)

        # ratio test
        good_matches = []
        for m1,m2 in matches:
            if m1.distance < dist_thesh*m2.distance:
                good_matches.append([m1])
        
        # good matches: list of Dmatch objects
        return good_matches
    

    def motion_estimate(self, kp_tprev, kp_t, matches, depth_t_prev):

        transform = np.eye(4)

        if len(matches) < 4:
            self.get_logger().warning("Less than 4 matches! Cannot attempt PnP...")
            return transform
        

        #TODO should depth image be for tprev or t?
        kp_tprev_idx = np.array([ kp_tprev[m[0].queryIdx].pt for m in matches], dtype=int)
        kp_t_idx = np.array([ kp_t[m[0].trainIdx].pt for m in matches], dtype=float)


        '''
        Projecting World to Camera & vice versa:
        x = fX / Z -> X = xZ / f
        y = fY / Z -> Y = yZ / f

        Note 
        '''

        #TODO depth clipping???
        Z = depth_t_prev[kp_tprev_idx[:, 1], kp_tprev_idx[:, 0]]
        # Z /= 1000
        # low_depth = Z < 6000

        # Z = Z[low_depth]
        # kp_tprev_idx = kp_tprev_idx[low_depth, :]
        # kp_t_idx = kp_t_idx[low_depth, :]

        world_pts = np.zeros((len(Z), 3))

        # if len(Z) < 60:
        #     self.get_logger().warning("Less than 60 matches! Too few matches...")
        #     return transform

        c = np.expand_dims(self.intrinsics[0:1, 2], axis=0)
        f = np.array([[self.intrinsics[0, 0], self.intrinsics[1, 1]]])

        world_pts[:, 0:2] = Z[:, np.newaxis] * (kp_tprev_idx - c) / f
        world_pts[:, 2] = Z
        
        success, rot_est, t_est, _ = cv.solvePnPRansac(world_pts, kp_t_idx, self.intrinsics, None)

        if success:

            rot_est, t_est = cv.solvePnPRefineLM(world_pts, kp_t_idx, self.intrinsics, None,
                                                rvec=rot_est, tvec=t_est)

            transform[0:3, 0:3] = cv.Rodrigues(rot_est)[0]
            transform[0:3, -1] = np.squeeze(t_est)

        else:
            self.get_logger().warning("PnP Pose Estimate Failed!")

        return transform


    def visualize_trajectory(self, transform):
        """_summary_

        Args:
            trajectory (List[np.ndarray]): _description_
        """

        ## TODO visualize point cloud from pose estimate
        rot, trans = transform[:3, :3], transform[0:3, -1]

        quat = mat2quat(rot)

        pose = Marker()
        pose.id = len(self.poses_markers.markers) + 1
        pose.header.frame_id = "origin"
        pose.type = 0
        pose.pose.position.x = trans[0]
        pose.pose.position.y = trans[1]
        pose.pose.position.z = trans[2]
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        pose.scale.x = 0.3
        pose.scale.y = 0.025
        pose.scale.z = 0.025
        pose.color.a = 1.0
        pose.color.b = 1.0
        self.poses_markers.markers.append(pose)

        pose = PoseStamped()
        pose.pose.position.x = trans[0]
        pose.pose.position.y = trans[1]
        pose.pose.position.z = trans[2]
        self.traj_marker.poses.append(pose)

        self.poses_viz_pub_.publish(self.poses_markers)
        self.traj_viz_pub_.publish(self.traj_marker)


    def load_config(self, filename):
        """"
        load parameters from config file
        """
        # cwd = os.getcwd()
        # package_share_directory = get_package_share_directory('py_vo')
        yaml_file_path = os.path.join("/home/sridevi/Documents/f1tenth-project/src/py_vo/config/vo_config.yaml")

        with open(yaml_file_path, 'r') as file:
            data = yaml.safe_load(file)
        
        self.intrinsics = np.array(data['intrinsics'])
        self.detector = data['detector']

def main(args=None):

    rclpy.init(args=args)
    vo_node = VisualOdometry()
    rclpy.spin(vo_node)

    # Destroy the node explicitly
    vo_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

