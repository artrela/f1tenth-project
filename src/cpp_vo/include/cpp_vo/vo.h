#include <iostream>
#include <string>
#include <algorithm>
#include <cmath>
#include <vector>

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


using namespace std;


class VO : public rclcpp::Node {
    public:
        VO();
        virtual ~VO();

    private:
    // initializations
    cv::Mat color_prev, depth_prev;  
    cv::Mat intrinsics;

    // publishers and subscribers


    // callback
    void visual_odom_callback(const sensor_msgs::msg::Image::ConstSharedPtr depth_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr color_msg);

    // methods
    cv::Mat convert_image(const sensor_msgs::msg::Image& image_msg, bool depth);
    void draw_matches(const cv::Mat& img1, const cv::Mat& img2, const vector<cv::KeyPoint>& kps1, 
        const vector<cv::KeyPoint>& kps2, const vector<cv::DMatch>& matches);
    void get_features();
    void get_matches();
    cv::Mat motion_estimate(const vector<cv::KeyPoint>& kp_tprev, const vector<cv::KeyPoint>& kp_t,
        const vector<cv::DMatch>& matches, const cv::Mat& depth_t_prev);
    void visualize_trajectory();
    void load_config(string filepath);


};