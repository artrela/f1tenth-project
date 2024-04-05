#!/usr/bin/env python3
import rclpy
import cv2
import numpy as np
import yaml
from typing import List
from rclpy.node import Node
from message_filters import TimeSynchronizer, Subscriber
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class VisualOdometry(Node):
    """
    The class that handles the VO pipeline
    """
    def __init__(self):
        super().__init__('vo_node')
        """
        """

        self.detector = None
        self.instrinsics = None

        self.trajectory: List[np.ndarray] = [np.eye(4)]
        self.color_prev = None
        self.cv_bridge = CvBridge()

        self.load_config("")

        tss = TimeSynchronizer(Subscriber("/camera/depth/image_rect_raw", Image),
                               Subscriber("/camera/color/image_raw", Image))
        tss.registerCallback(self.visual_odom_callback)


    def visual_odom_callback(self, color_msg: Image, depth_msg: Image):
        """
        run VO pipeline
        """
        color_img = self.convert_image(color_msg)
        depth_img = self.convert_image(depth_msg)

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
    
    def convert_image(self, image_msg):
        return self.cv_bridge.imgmsg_to_cv2(image_msg, "bgr8")

    def get_features(self, image, detector_type):
        """
        get features from image
        """
        if detector_type == 'sift':
            detector = cv2.SIFT_create()
        elif detector_type == 'orb':
            detector = cv2.ORB_create()

        keypoints, descriptors = detector.detectAndCompute(image, None)

        return keypoints, descriptors
    
    def get_matches(self, desc1, desc2, detectogit merge --abort
r, k=2, dist_thesh=0.75):
        """"
        match features between two frames
        """
        # brute-force matching
        if detector == 'sift':
            bf = cv2.BFMatcher(cv2.NORM_L2, crossCheck=True)
        elif detector == 'orb':
            bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

        # knn match
        matches = bf.knnMatch(desc1, desc2, k)

        # ratio test
        good_matches = []
        for m1,m2 in matches:
            if m1.distance < dist_thesh*m2.distance:
                good_matches.append([m1])
        
        return good_matches
    
    def motion_estimate(self, kp_t, kp_tnext, matches, depth_img):
        pass


    def visualize_trajectory(self, trajectory: List[np.ndarray]):
        """_summary_

        Args:
            trajectory (List[np.ndarray]): _description_
        """
        

    def load_config(self, path):
        """"
        load parameters from config file
        """
        with open(path, 'r') as file:
            data = yaml.safe_load(file)
        
        self.instrinsics = np.array(data['intrinsics'])
        self.detector = data['detector']


def main(args=None):

    rclpy.init(args=args)
    vo_node = VisualOdometry()
    rclpy.spin(vo_node)
    vo_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
