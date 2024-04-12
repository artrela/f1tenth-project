#include "cpp_vo/vo.h"

using namespace std;
// using sensor_msgs::msg::Image;
using namespace sensor_msgs::msg;
using namespace message_filters;
// using SyncPolicy = sync_policies::ApproximateTime<Image, Image>;

// Destructor of the VO class
VO::~VO() {
    RCLCPP_INFO(rclcpp::get_logger("VO"), "%s\n", "VO shutting down");
}

// Constructor of the VO class
VO::VO(): rclcpp::Node("vo_node")
{
    color_prev_ptr = std::make_unique<cv::Mat>();
    depth_prev_ptr = std::make_unique<cv::Mat>();

    // ROS publishers and subscribers
    matches_publisher_ = this->create_publisher<Image>("/matches", 10);

    depth_img_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "/camera/aligned_depth_to_color/image_raw");
    color_img_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "/camera/color/image_raw");

    syncApproximate = std::make_shared<message_filters::Synchronizer<ApproximatePolicy>>(ApproximatePolicy(10), *depth_img_sub, *color_img_sub);

    syncApproximate->setMaxIntervalDuration(rclcpp::Duration(0, 100000000));  // 0.1 seconds
    syncApproximate->registerCallback(std::bind(&VO::visual_odom_callback, this, std::placeholders::_1, std::placeholders::_2));
    poses_viz_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/poses_viz", 10);
    traj_viz_pub = this->create_publisher<nav_msgs::msg::Path>("/traj_viz", 10);

    // initialize tf values
    global_tf = cv::Mat::eye(4, 4, CV_64F);
    global_tf.col(3).setTo(cv::Scalar(1));

    // set intrinsics values
    intrinsics = (cv::Mat_<double>(3, 3) <<  606.328369140625, 0, 322.6350402832031,
                                            0, 605.257568359375, 239.6647491455078,
                                            0.0, 0.0, 1.0);
    
    // TODO: turn to rosparam
    std::string feature = "sift";

    if( feature == "sift" ){
        // feature_extractor = cv::SIFT::create(200, 3, 0.04, 10, 1.6, false);
        feature_extractor = cv::SIFT::create();
    }
    else{
        // feature_extractor = cv::ORB::create(500, 1.2f, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, 20);
        feature_extractor = cv::ORB::create();
    }

    // visualizing pose chain
    pose_markers = visualization_msgs::msg::MarkerArray();
    traj_path = nav_msgs::msg::Path();
    traj_path.header.frame_id = "origin";
    visualize_trajectory(global_tf);

    RCLCPP_INFO(rclcpp::get_logger("VO"), "%s %s\n", "OpenCV Version: ", CV_VERSION);
    RCLCPP_INFO(rclcpp::get_logger("VO"), "%s\n", "Created new VO Object.");
}

void VO::visual_odom_callback(const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg, const sensor_msgs::msg::Image::ConstSharedPtr& color_msg){
    
    RCLCPP_INFO(rclcpp::get_logger("VO"), "%s\n", "In Callback!");

    // convert image
    cv::Mat color_img = convert_image(color_msg, false);
    cv::Mat depth_img = convert_image(depth_msg, true);


    // first time initialization check
    if (color_prev_ptr->empty() && depth_prev_ptr->empty()) {
        *color_prev_ptr = color_img.clone();
        *depth_prev_ptr = depth_img.clone();
        // RCLCPP_INFO(rclcpp::get_logger("VO"), "Color image size: %d x %d", color_prev_ptr->rows, color_prev_ptr->cols);
        // RCLCPP_INFO(rclcpp::get_logger("VO"), "Depth image size: %d x %d", depth_prev_ptr->rows, depth_prev_ptr->cols);
        feature_extractor->detectAndCompute(*color_prev_ptr, cv::noArray(), kps_prev, desc_prev);
    }

    // get features
    std::vector<cv::KeyPoint> kps;
    cv::Mat desc;
    feature_extractor->detectAndCompute(color_img, cv::noArray(), kps, desc);
    RCLCPP_INFO(rclcpp::get_logger("VO"), "%s\n", "Got Features!");
    
    // get matches
    vector<cv::DMatch> good_matches;
    get_matches(desc_prev, desc, good_matches);
    draw_matches(*color_prev_ptr, color_img, kps_prev, kps, good_matches);
    cout << good_matches.size() << endl;
    RCLCPP_INFO(rclcpp::get_logger("VO"), "%s\n", "Got Matches!");

    // get transforms
    cout << good_matches.size() << endl;
    cv::Mat relative_tf = motion_estimate(kps_prev, kps, good_matches, *depth_prev_ptr);
    global_tf = global_tf * relative_tf;

    // visualize trajectory
    visualize_trajectory(global_tf);

    // update prev images
    *color_prev_ptr = color_img;
    *depth_prev_ptr = depth_img;
    kps_prev = kps;
    desc_prev = desc;
}

cv::Mat VO::convert_image(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg, bool depth){

    cv_bridge::CvImagePtr cv_ptr;

    if( depth ){
        cv_ptr = cv_bridge::toCvCopy(image_msg, "16UC1");
    }
    else{
        cv_ptr = cv_bridge::toCvCopy(image_msg, "bgr8");
    }

    return cv_ptr->image;

}

void VO::draw_matches(const cv::Mat& img1, const cv::Mat& img2, const vector<cv::KeyPoint>& kps1, const vector<cv::KeyPoint>& kps2, vector<cv::DMatch>& matches){

    cv::Mat img_matches;
    drawMatches( img1, kps1, img2, kps2, matches, img_matches, cv::Scalar::all(-1),
    cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

    sensor_msgs::msg::Image::SharedPtr msg =
            cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img_matches)
                .toImageMsg();

    matches_publisher_->publish(*msg);
}


void VO::get_matches(const cv::Mat& desc1, cv::Mat& desc2, std::vector<cv::DMatch>& good_matches ){

    // https://docs.opencv.org/3.4/d5/d6f/tutorial_feature_flann_matcher.html 

    RCLCPP_INFO(rclcpp::get_logger("VO"), "%s %i\n", "Number of Matches: ", good_matches.size());

    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
    std::vector<std::vector<cv::DMatch>> knn_matches;
    matcher->knnMatch( desc1, desc2, knn_matches, 2 );

    const float ratio_thresh = 0.7f;
    for (size_t i = 0; i < knn_matches.size(); i++){
    
        if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance){
            good_matches.push_back(knn_matches[i][0]);
        }
    }
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
        // auto Z = depth_values / 1000; // mm t0 m
        
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
        // cv::Mat tvec_scaled = tvec / 1000;
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



