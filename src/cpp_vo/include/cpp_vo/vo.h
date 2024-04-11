#include <iostream>
#include <string>
#include <algorithm>
#include <cmath>
#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "message_filters/time_synchronizer.h"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/features2d.hpp"
#include "image_transport/image_transport.h"
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/image_encodings.hpp"
#include <opencv2/features2d.hpp>

using namespace std;


class VO : public rclcpp::Node {
    public:
        VO();
        virtual ~VO();

    private:
    // initializations
    std::unique_ptr<cv::Mat> color_prev_ptr, depth_prev_ptr;  
    cv::Mat intrinsics;
    std::vector<cv::KeyPoint> kps_prev;
    std::vector<cv::Mat> desc_prev;

    cv::Mat global_tf;

    // publishers and subscribers
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr matches_publisher_;


    // callback
    void visual_odom_callback(const sensor_msgs::msg::Image::ConstSharedPtr depth_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr color_msg);

    // methods
    cv::Mat convert_image(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg, bool depth);
    void draw_matches(const cv::Mat& img1, const cv::Mat& img2, const vector<cv::KeyPoint>& kps1, 
        const vector<cv::KeyPoint>& kps2, const vector<cv::DMatch>& matches);
    void get_matches(const std::vector<cv::KeyPoint>& desc1, const std::vector<cv::KeyPoint>& desc2,
                        std::vector<cv::DMatch>& good_matches);
    cv::Mat motion_estimate(const vector<cv::KeyPoint>& kp_tprev, const vector<cv::KeyPoint>& kp_t,
        const vector<cv::DMatch>& matches, const cv::Mat& depth_t_prev);
    void visualize_trajectory();
    void load_config(string filepath);


};