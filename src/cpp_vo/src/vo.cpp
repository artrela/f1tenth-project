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
    poses_viz_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/poses_viz", 10);
    traj_viz_pub = this->create_publisher<nav_msgs::msg::Path>("/traj_viz", 10);

    // initialize tf values
    cv::Mat global_tf = cv::Mat::eye(4, 4, CV_64F);

    // set intrinsics values
    intrinsics = (cv::Mat_<double>(3, 3) <<  606.328369140625, 0, 322.6350402832031,
                                            0, 605.257568359375, 239.6647491455078,
                                            0.0, 0.0, 1.0);

    // visualizing pose chain
    pose_markers = visualization_msgs::msg::MarkerArray();
    traj_path = nav_msgs::msg::Path();
    traj_path.header.frame_id = "origin";
    visualize_trajectory(global_tf);

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

cv::Mat VO::convert_image(const sensor_msgs::msg::Image::ConstSharedPtr image_msg, bool depth){
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
    cv::Mat tf = cv::Mat::eye(4, 4, CV_64F);

    // check for at least 4 matches
    if (matches.size() < 4) {
        RCLCPP_INFO(rclcpp::get_logger("VO"), "Less than 4 matches! Cannot attempt PnP");
        return tf;
    }

    std::vector<cv::Point2f> kp_tprev_idx, kp_t_idx;
    std::vector<cv::Point3f> world_pts;

    for (auto& match : matches) {

        // get x,y coordinates of matched keypoints
        cv::Point2f pt_prev = kp_tprev[match.queryIdx].pt;
        cv::Point2f pt_current = kp_t[match.trainIdx].pt;

        // get depth value at keypoint coords
        auto Z = depth_t_prev.at<double>(pt_prev.y, pt_prev.x);
        
        // get world coordinates from intrinsics
        float X = Z * (pt_prev.x - intrinsics.at<double>(0, 2)) / intrinsics.at<double>(0, 0);
        float Y = Z * (pt_prev.y - intrinsics.at<double>(1, 2)) / intrinsics.at<double>(1, 1);

        world_pts.push_back(cv::Point3f(X, Y, Z));
        kp_t_idx.push_back(pt_current);

    }

    // use solve PnP RANSAC to get rvec and tvec
    cv::Mat rvec, tvec;
    bool success = cv::solvePnPRansac(world_pts, kp_t_idx, intrinsics, cv::Mat(), rvec, tvec);

    // make tf matrix
    if (success) {
        cv::Mat R;
        cv::Rodrigues(rvec, R);
        R.copyTo(tf(cv::Rect(0, 0, 3, 3)));
        tvec.copyTo(tf(cv::Rect(3, 0, 1, 3)));
    } else {
        RCLCPP_INFO(rclcpp::get_logger("VO"), "PnP Pose Estimate Failed");
    }

    return tf;

}


void VO::visualize_trajectory(cv::Mat tf){
    Eigen::Matrix4d eigen_mat;
    cv::cv2eigen(tf, eigen_mat);
    Eigen::Isometry3d eigen_tf = Eigen::Isometry3d(eigen_mat);
    Eigen::Quaterniond eigen_quat(eigen_tf.rotation());

    auto pose_marker = visualization_msgs::msg::Marker();
    pose_marker.header.frame_id = "origin";
    pose_marker.id = pose_markers.markers.size()+1;
    pose_marker.type = 0;
    pose_marker.pose.position.x = eigen_tf.translation().x();
    pose_marker.pose.position.y = eigen_tf.translation().y();
    pose_marker.pose.position.z = eigen_tf.translation().z();
    pose_marker.pose.orientation.x = eigen_quat.x();
    pose_marker.pose.orientation.y = eigen_quat.y();
    pose_marker.pose.orientation.z = eigen_quat.z();
    pose_marker.pose.orientation.w = eigen_quat.w();
    pose_marker.scale.x = 0.3;
    pose_marker.scale.y = 0.025;
    pose_marker.scale.z = 0.025;
    pose_marker.color.a = 1.0;
    pose_marker.color.b = 1.0;
    pose_markers.markers.push_back(pose_marker);

    auto traj_marker = geometry_msgs::msg::PoseStamped();
    traj_marker.pose.position.x = eigen_tf.translation().x();
    traj_marker.pose.position.y = eigen_tf.translation().y();
    traj_marker.pose.position.z = eigen_tf.translation().z();
    traj_path.poses.push_back(traj_marker);

    poses_viz_pub->publish(pose_markers);
    traj_viz_pub->publish(traj_path);

}


void VO::load_config(string filepath){

}


