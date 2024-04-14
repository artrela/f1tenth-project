#include <iostream>
#include <string>
#include <algorithm>
#include <cmath>
#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "message_filters/time_synchronizer.h"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Geometry>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/features2d.hpp"
#include "image_transport/image_transport.hpp"
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/image_encodings.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp> 

// #include "opencv2/core/cuda.hpp"
// #include "opencv2/cudalegacy.hpp"
// #include "opencv2/cudalegacy/NCV.hpp"
// #include "opencv2/cudalegacy/NPP_staging.hpp"
// #include "opencv2/cudalegacy/NCVPyramid.hpp"
// #include "opencv2/cudalegacy/NCVHaarObjectDetection.hpp"
// #include "opencv2/cudalegacy/NCVBroxOpticalFlow.hpp"
// #include "opencv2/video/background_segm.hpp"


using namespace std;


class VO : public rclcpp::Node {
    public:
        VO();
        virtual ~VO();

    private:
        // initializations
        std::unique_ptr<cv::Mat> color_prev_ptr;
        std::unique_ptr<cv::Mat> depth_prev_ptr;
        cv::Mat intrinsics;
        std::vector<cv::KeyPoint> kps_prev;
        cv::Mat desc_prev;
        cv::Mat global_tf;
        cv::Ptr<cv::Feature2D> feature_extractor;
        visualization_msgs::msg::MarkerArray pose_markers;
        nav_msgs::msg::Path traj_path;

        // publishers and subscribers
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr matches_publisher_;
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> ApproximatePolicy;
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> depth_img_sub;
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> color_img_sub;
        std::shared_ptr<message_filters::Synchronizer<ApproximatePolicy>> syncApproximate;    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr poses_viz_pub;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr traj_viz_pub;

        // callback
        void visual_odom_callback(const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,
            const sensor_msgs::msg::Image::ConstSharedPtr& color_msg);

        // methods
        cv::Mat convert_image(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg, bool depth);
        void draw_matches(const cv::Mat& img1, const cv::Mat& img2, const vector<cv::KeyPoint>& kps1, 
                    const vector<cv::KeyPoint>& kps2, vector<cv::DMatch>& matches);
        void get_matches(const cv::Mat& desc1, cv::Mat& desc2, std::vector<cv::DMatch>& good_matches);
        cv::Mat motion_estimate(const vector<cv::KeyPoint>& kp_tprev, const vector<cv::KeyPoint>& kp_t,
        const vector<cv::DMatch>& matches, const cv::Mat& depth_t_prev);
        void visualize_trajectory(cv::Mat tf);

};