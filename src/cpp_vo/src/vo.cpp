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
    matches_publisher_ = this->create_publisher<Image>("/matches", 10);

    message_filters::Subscriber<Image> depth_img_sub(this, "/camera/aligned_depth_to_color/image_raw");
    message_filters::Subscriber<Image> color_img_sub(this, "/camera/color/image_raw");
    auto sync = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), depth_img_sub, color_img_sub);
    sync->registerCallback(std::bind(&VO::visual_odom_callback, this, std::placeholders::_1, std::placeholders::_2));

    // initialize tf values
    global_tf = cv::Mat::eye(4, 4, CV_64F);
    global_tf.col(3).setTo(cv::Scalar(1));

    // set intrinsics values
    intrinsics = (cv::Mat_<double>(3, 3) <<  606.328369140625, 0, 322.6350402832031,
                                            0, 605.257568359375, 239.6647491455078,
                                            0.0, 0.0, 1.0);
    

    // TODO turn to rosparam
    std::string feature = "sift";

    if( feature == "sift" ){
        static cv::Ptr<cv::Feature2D> feature_extractor = cv::SIFT::create(200, 3, 0.04, 10, 1.6, false);
    }
    else{
        static cv::Ptr<cv::ORB> feature_extractor = cv::ORB::create(500, 1.2f, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, 20);

    }

    RCLCPP_INFO(rclcpp::get_logger("VO"), "%s\n", "Created new VO Object.");
}

void VO::visual_odom_callback(const sensor_msgs::msg::Image::ConstSharedPtr depth_msg, const sensor_msgs::msg::Image::ConstSharedPtr color_msg){
    
    // convert image
    cv::Mat color_img = convert_image(color_msg, false);
    cv::Mat depth_img = convert_image(depth_msg, true);

    if( !color_prev_ptr && !depth_prev_ptr){
        color_prev_ptr = std::make_unique<cv::Mat>(color_img);
        depth_prev_ptr = std::make_unique<cv::Mat>(depth_img);

        feature_extractor->detectAndCompute(*color_prev_ptr, this->kps_prev);
    }

    // get features
    std::vector<cv::KeyPoint> kps;
    std::vector<cv::Mat> desc;
    feature_extractor->detectAndCompute(color_img, cv::noArray(), kps, desc);
    
    // get matche


    // get transforms
    // cv::Mat relative_tf = motion_estimate(this->kps_prev, kps, matches, *depth_prev_ptr);
    // global_tf = global_tf * relative_tf;

    // visualize trajectory
    visualize_trajectory(global_tf);

    // update prev images
    *color_prev_ptr = color_img;
    *depth_prev_ptr = depth_img;
    this->kps_prev = kps;
    this->desc_prev = desc;
}

cv::Mat VO::convert_image(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg, bool depth){

    cv_bridge::CvImagePtr cv_ptr;

    if( depth ){
        cv_ptr = cv_bridge::toCvCopy(image_msg, "passthrough");
    }
    else{
        cv_ptr = cv_bridge::toCvCopy(image_msg, "bgr8");
    }

}

void VO::draw_matches(const cv::Mat& img1, const cv::Mat& img2, const vector<cv::KeyPoint>& kps1, const vector<cv::KeyPoint>& kps2, const vector<cv::DMatch>& matches){

    cv::Mat img_matches;
    drawMatches( img1, kps1, img2, kps2, matches, img_matches, cv::Scalar::all(-1),
    cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr->image = img_matches;
    cv_ptr = cv_bridge::toCvCopy(cv_ptr, sensor_msgs::image_encodings::BGR8);
    matches_publisher_->publish(cv_ptr->toImageMsg());
}



void VO::get_matches(const std::vector<cv::KeyPoint>& desc1, const std::vector<cv::KeyPoint>& desc2,
                        std::vector<cv::DMatch>& good_matches ){

    // https://docs.opencv.org/3.4/d5/d6f/tutorial_feature_flann_matcher.html 

    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
    std::vector< std::vector<cv::DMatch> > knn_matches;
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


