#include "cpp_vo/vo.h"

using namespace std;
using namespace sensor_msgs::msg;
using namespace message_filters;

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
    tracking_publisher_ = this->create_publisher<Image>("/tracked_matches", 10);

    depth_img_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "/camera/camera/aligned_depth_to_color/image_raw");
    color_img_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "/camera/camera/color/image_raw");
    syncApproximate = std::make_shared<message_filters::Synchronizer<ApproximatePolicy>>(ApproximatePolicy(10), *depth_img_sub, *color_img_sub);

    syncApproximate->setMaxIntervalDuration(rclcpp::Duration(0, 100000000));  // 0.1 seconds
    syncApproximate->registerCallback(std::bind(&VO::visual_odom_callback, this, std::placeholders::_1, std::placeholders::_2));
    poses_viz_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/poses_viz", 10);
    traj_viz_pub = this->create_publisher<nav_msgs::msg::Path>("/traj_viz", 10);
    pose_pub = this->create_publisher<nav_msgs::msg::Odometry>("/visual_odometry/pose", 10);

    this->declare_parameter("cov_x", 0.4);
    this->declare_parameter("cov_y", 0.4);
    this->declare_parameter("cov_yaw", 0.8);

    cov_x = this->get_parameter("cov_x").as_double();
    cov_y = this->get_parameter("cov_y").as_double();
    cov_yaw = this->get_parameter("cov_yaw").as_double();

    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    curr_pose.header.frame_id = "odom";
    curr_pose.child_frame_id = "camera_imu_optical_frame";

    // initialize tf values
    global_tf = cv::Mat::eye(4, 4, CV_64F);

    // set intrinsics values
    // intrinsics = (cv::Mat_<double>(3, 3) <<  606.328369140625, 0, 322.6350402832031,
    //                                         0, 605.257568359375  , 239.6647491455078,
    //                                         0.0, 0.0, 1.0);
    intrinsics = (cv::Mat_<double>(3, 3) <<  905.43359375, 0, 639.9384765625,
                                            0, 905.3507080078125, 356.8575134277344,
                                            0.0, 0.0, 1.0);
    // intrinsics = (cv::Mat_<double>(3, 3) <<  426.876, 0, 426.593,
    //                                         0, 426.593, 238.275,
    //                                         0.0, 0.0, 1.0); 
    
    // TODO: turn to rosparam
    feature = "sift";

    if( feature == "sift" ){
        feature_extractor = cv::SIFT::create(2000, 3, 0.04, 10, 2.0);
    }
    else{
        feature_extractor = cv::ORB::create(1000, 1.3, 8, 31, 0, 4, cv::ORB::HARRIS_SCORE, 31, 20);
    }

    // visualizing pose chain
    pose_markers = visualization_msgs::msg::MarkerArray();
    traj_path = nav_msgs::msg::Path();
    traj_path.header.frame_id = "odom";
    publish_position(global_tf);

    RCLCPP_INFO(rclcpp::get_logger("VO"), "%s %s\n", "OpenCV Version: ", CV_VERSION);
    RCLCPP_INFO(rclcpp::get_logger("VO"), "%s\n", "Created new VO Object.");
}

void VO::visual_odom_callback(const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg, const sensor_msgs::msg::Image::ConstSharedPtr& color_msg){
    
    // convert image
    cv::Mat color_img = convert_image(color_msg, false);
    cv::Mat depth_img = convert_image(depth_msg, true);

    // first time initialization check
    if (color_prev_ptr->empty() && depth_prev_ptr->empty()) {
        *color_prev_ptr = color_img.clone();
        *depth_prev_ptr = depth_img.clone();
        feature_extractor->detectAndCompute(*color_prev_ptr, cv::noArray(), kps_prev, desc_prev);
    }

    // get features
    std::vector<cv::KeyPoint> kps;
    cv::Mat desc;
    feature_extractor->detectAndCompute(color_img, cv::noArray(), kps, desc);
    
    // Convert binary descriptors to float type
    if (feature == "orb") {
        desc.convertTo(desc, CV_32F);
        desc_prev.convertTo(desc_prev, CV_32F);
    }

    // get matches
    vector<cv::DMatch> good_matches;
    get_matches(desc_prev, desc, good_matches);

    // get transforms
    cv::Mat relative_tf = motion_estimate(kps_prev, kps, good_matches, *depth_prev_ptr, color_img);
    global_tf = global_tf * relative_tf;

    // visualize trajectory
    publish_position(global_tf);
    // publish_position(relative_tf);

    // update prev images
    *color_prev_ptr = color_img;
    *depth_prev_ptr = depth_img;
    kps_prev = kps;
    desc_prev = desc;
}

cv::Mat VO::convert_image(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg, bool depth){

    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat img;

    if( depth ){
        cv_ptr = cv_bridge::toCvCopy(image_msg, "16UC1");
    }
    else{
        cv_ptr = cv_bridge::toCvCopy(image_msg, "bgr8");
    }
    img = cv_ptr->image;

    return img;

}

void VO::draw_matches(const cv::Mat& img1, const cv::Mat& img2, const vector<cv::KeyPoint>& kps1, const vector<cv::KeyPoint>& kps2, vector<cv::DMatch>& matches){

    vector<cv::DMatch> top_matches;
    for(size_t i = 0; i < 15; i++){
        top_matches.push_back(matches[i]);
    }

    cv::Mat img_matches;
    drawMatches( img1, kps1, img2, kps2, top_matches, img_matches, cv::Scalar::all(-1),
    cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

    sensor_msgs::msg::Image::SharedPtr msg =
            cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img_matches)
                .toImageMsg();

    matches_publisher_->publish(*msg);
}


void VO::get_matches(const cv::Mat& desc1, cv::Mat& desc2, std::vector<cv::DMatch>& good_matches ){

    // https://docs.opencv.org/3.4/d5/d6f/tutorial_feature_flann_matcher.html 

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
    const vector<cv::DMatch>& matches, const cv::Mat& depth_t_prev, const cv::Mat& color_prev)
{
    cv::Mat tf = cv::Mat::eye(4, 4, CV_64F);       

    std::vector<cv::Point2f> kp_tprev_idx, kp_t_idx;
    std::vector<cv::Point3f> world_pts;

    for (auto& match : matches) {

        // get x,y coordinates of matched keypoints
        cv::Point2f pt_prev = kp_tprev[match.queryIdx].pt;
        cv::Point2f pt_current = kp_t[match.trainIdx].pt;

        // get depth value at keypoint coords
        // IMPORTANT: https://github.com/IntelRealSense/realsense-ros/issues/807#issuecomment-529909735 
        uint16_t Z = depth_t_prev.at<uint16_t>(pt_prev.y, pt_prev.x);

        // per orbslam2, if Z > 40 * baseline (50mm) cannot track, lets be liberal on that here
        // https://www.framos.com/en/products/depth-camera-d435i-bulk-22610 
        if( Z < 200 ){
            continue;
        }

        // https://github.com/IntelRealSense/realsense-ros/issues/2464
        // get value in meters
        Z /= 1000;

        // get world coordinates from intrinsics
        double X = Z * (pt_prev.x - intrinsics.at<double>(0, 2)) / intrinsics.at<double>(0, 0);
        double Y = Z * (pt_prev.y - intrinsics.at<double>(1, 2)) / intrinsics.at<double>(1, 1);

        world_pts.push_back(cv::Point3f(X, Y, Z));
        kp_t_idx.push_back(pt_current);
        kp_tprev_idx.push_back(pt_prev);

    }

    display_tracking(color_prev, kp_tprev_idx, kp_t_idx, 30);

    if( world_pts.size() < 4 ){
        RCLCPP_INFO(rclcpp::get_logger("VO"), "Less than 4 matches! Cannot attempt PnP");
        return tf;
    }

    // use solve PnP RANSAC to get rvec and tvec
    /*
    InputArray 	imagePoints,
    InputArray 	cameraMatrix,
    InputArray 	distCoeffs,
    OutputArray 	rvec,
    OutputArray 	tvec,
    bool 	useExtrinsicGuess = false,
    int 	iterationsCount = 100,
    float 	reprojectionError = 8.0,
    double 	confidence = 0.99,
    OutputArray 	inliers = noArray(),
    int 	flags = SOLVEPNP_ITERATIVE 
    */

    cv::Mat rvec, tvec;
    // bool success = cv::solvePnPRansac(world_pts, kp_t_idx, intrinsics, cv::noArray(), rvec, tvec);
    bool success = cv::solvePnPRansac(world_pts, kp_t_idx, intrinsics, cv::noArray(), rvec, tvec, false,
                                        350, 8.0, 0.99, cv::noArray(), cv::SOLVEPNP_ITERATIVE);

    double norm = cv::norm(tvec, cv::NORM_L2, cv::noArray());

    // make tf matrix
    if (success && norm > 0.01 && norm < 1.0) {

        cv::solvePnPRefineLM(world_pts, kp_t_idx, intrinsics, cv::noArray(), rvec, tvec);

        cv::Mat R;
        cv::Rodrigues(rvec, R);
        R.copyTo(tf(cv::Rect(0, 0, 3, 3)));
        tvec.copyTo(tf(cv::Rect(3, 0, 1, 3)));

    }
    else if(success && norm < 0.01){
        RCLCPP_INFO(rclcpp::get_logger("VO"), "Skipping PnP Pose Estimate, too Small!");
    }
    else if(success && norm > 1.0){
        RCLCPP_INFO(rclcpp::get_logger("VO"), "PnP Pose Estimate too Large!");
    }
    else {
        RCLCPP_INFO(rclcpp::get_logger("VO"), "PnP Pose Estimate Failed");
    }

    return tf;

}

void VO::display_tracking(const cv::Mat img_prev, 
                    std::vector<cv::Point2f>&  kp_prev,
                    std::vector<cv::Point2f>&  kp,
                    const size_t vis_count)
{
    int radius = 2;
    cv::Mat vis = img_prev.clone();
    size_t max = std::min(vis_count, kp_prev.size());
    
    for (size_t i = 0; i < max; i++)
    {
        cv::circle(vis, cv::Point(kp_prev[i].x, kp_prev[i].y), radius, CV_RGB(0,255,0));
    }

    for (size_t i = 0; i < max; i++)
    {
        cv::circle(vis, cv::Point(kp[i].x, kp[i].y), radius, CV_RGB(255,0,0));
    }
    
    for( size_t i = 0; i < max; i++)
    {  
        cv::Point start(kp_prev[i].x, kp_prev[i].y);
        cv::Point end(kp[i].x, kp[i].y);
        cv::line(vis, start, end, CV_RGB(0,255,0));
    }

    sensor_msgs::msg::Image::SharedPtr msg =
            cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", vis)
                .toImageMsg();

    tracking_publisher_->publish(*msg);
}

void VO::publish_position(cv::Mat tf){
    
    Eigen::Matrix4d eigen_mat;
    cv::cv2eigen(tf, eigen_mat);
    Eigen::Isometry3d eigen_tf = Eigen::Isometry3d(eigen_mat);
    Eigen::Quaterniond eigen_quat(eigen_tf.rotation());

    auto pose_marker = visualization_msgs::msg::Marker();
    pose_marker.header.frame_id = "odom";
    pose_marker.id = pose_markers.markers.size()+1;
    pose_marker.type = 0;
    // pose_marker.pose.position.x = eigen_tf.translation().x();
    pose_marker.pose.position.x = -eigen_tf.translation().z();
    pose_marker.pose.position.y =  eigen_tf.translation().x();
    // pose_marker.pose.position.z = eigen_tf.translation()z.();
    // pose_marker.pose.position.z = eigen_tf.translation().y();
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
    // traj_marker.pose.position.x = eigen_tf.translation().x();
    traj_marker.pose.position.x = -eigen_tf.translation().z();
    traj_marker.pose.position.y =  eigen_tf.translation().x();
    // traj_marker.pose.position.z = eigen_tf.translation().z();
    // traj_marker.pose.position.z = eigen_tf.translation().y();
    traj_path.poses.push_back(traj_marker);

    curr_pose.header.stamp = this->get_clock()->now();
    curr_pose.pose.pose.position.x = -eigen_tf.translation().z();
    curr_pose.pose.pose.position.y =  eigen_tf.translation().x();
    curr_pose.pose.pose.orientation.x = eigen_quat.x();
    curr_pose.pose.pose.orientation.y = eigen_quat.y();
    curr_pose.pose.pose.orientation.z = eigen_quat.z();
    curr_pose.pose.pose.orientation.w = eigen_quat.w();

    // Position uncertainty
    curr_pose.pose.covariance[0] = cov_x;   ///< x
    curr_pose.pose.covariance[7] = cov_y;   ///< y
    curr_pose.pose.covariance[35] = cov_yaw;  ///< yaw

    poses_viz_pub->publish(pose_markers);
    traj_viz_pub->publish(traj_path);
    pose_pub->publish(curr_pose);

    
}



