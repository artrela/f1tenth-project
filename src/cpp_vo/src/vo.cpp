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

    // initialize tf values
    cv::Mat global_tf = cv::Mat::eye(4, 4, CV_64F);
    global_tf.col(3).setTo(cv::Scalar(1));

    // set intrinsics values
    intrinsics = (cv::Mat_<double>(3, 3) <<  606.328369140625, 0, 322.6350402832031,
                                            0, 605.257568359375, 239.6647491455078,
                                            0.0, 0.0, 1.0);

    RCLCPP_INFO(rclcpp::get_logger("VO"), "%s\n", "Created new VO Object.");
}

void VO::visual_odom_callback(const sensor_msgs::msg::Image::ConstSharedPtr depth_msg, const sensor_msgs::msg::Image::ConstSharedPtr color_msg){
    // convert image

    // get features

    // get matches

    // get transforms
    // relative_tf = motion_estimate(kps_prev, kps, matches, depth_prev)
    // global_tf = global_tf * relative_tf

    // visualize trajectory
    // visualize_trajectory(global_tf)

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


cv::Mat VO::motion_estimate(const vector<cv::KeyPoint>& kp_tprev, const vector<cv::KeyPoint>& kp_t,
    const vector<cv::DMatch>& matches, const cv::Mat& depth_t_prev)
{
    // cv::Mat tf = cv::Mat::eye(4, 4, CV_64F);

    // if (matches.size() < 4) {
    //     RCLCPP_INFO(rclcpp::get_logger("VO"), "Less than 4 matches! Cannot attempt PnP");
    //     return tf;
    // }

    // vector<cv::Point2f> kp_tprev_idx, kp_t_idx;
    // for (const auto& m : matches) {
    //     kp_tprev_idx.push_back(kp_tprev[m[0].queryIdx].pt);
    //     kp_t_idx.push_back(kp_t[m[0].trainIdx].pt);
    // }

    // vector<double> Z;
    // for (const auto& point: kp_tprev_idx){
    //     auto value = depth_t_prev.at<double>(point.x, point.y);
    //     z.push_back(value);
    // }

    // vector<Point3f> world_pts(Z.size());
    // vector<Point3f> img_pts(Z.size());

    // for (int i=0; i<Z.size(); ++i){

    // }

    // cv::Mat rvec, tvec;
    // cv::Mat dist_coeffs = cv::Mat::zeros(4, 1, CV_64F); // assuming no lens distortion
    // bool success = cv::solvePnPRansac(world_pts, img_pts, intrinsics, dist_coeffs, rvec, tvec);

    // if (success) {
        
    // } else {
    //     RCLCPP_INFO(rclcpp::get_logger("VO"), "PnP Pose Estimate Failed!");
    // }

    // return tf;

}


void VO::visualize_trajectory(){

}


void VO::load_config(string filepath){

}


