#include "cpp_vo/vo.h"
using namespace std;
using sensor_msgs::msg::Image;
using namespace message_filters;
using SyncPolicy = sync_policies::ApproximateTime<Image, Image>;

// Destructor of the VO class
VO::~VO() {
    RCLCPP_INFO(rclcpp::get_logger("VO"), "%s\n", "VO shutting down");
}

// Constructor of the VO class
VO::VO(): rclcpp::Node("vo_node")
{
    // ROS publishers and subscribers
    message_filters::Subscriber<Image> depth_img_sub(this, "/camera/aligned_depth_to_color/image_raw");
    message_filters::Subscriber<Image> color_img_sub(this, "/camera/color/image_raw");
    auto sync = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), depth_img_sub, color_img_sub);
    sync->registerCallback(std::bind(&VO::visual_odom_callback, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(rclcpp::get_logger("VO"), "%s\n", "Created new VO Object.");
}

void VO::visual_odom_callback(const sensor_msgs::msg::Image::ConstSharedPtr depth_msg, const sensor_msgs::msg::Image::ConstSharedPtr color_msg){
    // convert image

    // get features

    // get matches

    // get transforms

    // visualize trajectory

    // update prev images
}

cv::Mat VO::convert_image(const sensor_msgs::msg::Image& image_msg, bool depth){

}

void VO::draw_matches(const cv::Mat& img1, const cv::Mat& img2, const vector<cv::KeyPoint>& kps1, const vector<cv::KeyPoint>& kps2, const vector<cv::DMatch>& matches){

}

void VO::get_features(){

}


void VO::get_matches(){

}


void VO::motion_estimate(){

}


void VO::visualize_trajectory(){

}


void VO::load_config(string filepath){

}


