#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from typing import List

from message_filters import TimeSynchronizer, Subscriber
from sensor_msgs.msg import Image

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

        self.trajectory: List[np.ndarray] = [np.eye(4)]
        self.color_prev = None

        self.load_config("")

        tss = TimeSynchronizer(Subscriber("/camera/depth/image_rect_raw", Image),
                       Subscriber("/camera/color/image_raw", Image))
        tss.registerCallback(self.visual_odom_callback)


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


    def visualize_trajectory(self, trajectory: List[np.ndarray]):
        """_summary_

        Args:
            trajectory (List[np.ndarray]): _description_
        """
        

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
