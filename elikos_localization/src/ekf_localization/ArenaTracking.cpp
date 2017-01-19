//
// Created by Antonio Sanniravong on 21/12/15.
//

#include <ArenaTracking.h>

void OnTrackbar(int, void *) {
    // Trackbar position has changed
}

ArenaTracking::ArenaTracking(ros::NodeHandle* nh)
        : nh_(nh), it_(*nh), pose_pub_(nh) {
    // Set subscribers and publishers
    SetSubcribers();
    SetPublishers();

    // Init output_ matrix
    output_ = cv::Mat(480, 640, CV_8UC3);


    cv::Mat_<float> mat(3,3);

    // Init undistortion map
    camera_matrix_ = (cv::Mat_<float>(3,3) << 422.918640,    0.000000,    350.119451,
                                                0.000000,  423.121112,    236.380265,
                                                0.000000,    0.000000,      1.000000);

    camera_distortion_ = (cv::Mat_<float>(1,5) << -0.321590, 0.089597, 0.001090, -0.000489, 0.000000);

    camera_matrix_new_ = cv::getOptimalNewCameraMatrix(camera_matrix_, camera_distortion_, cv::Size(640, 480), 0);

    std::cout << camera_matrix_new_;

    cv::initUndistortRectifyMap(camera_matrix_, camera_distortion_, cv::Mat(), camera_matrix_new_,
                                cv::Size(640, 480), CV_32FC1, distortion_map1_, distortion_map2_);

    H_MIN = 77;
    H_MAX = 256;
    S_MIN = 10;
    S_MAX = 256;
    V_MIN = 0;
    V_MAX = 256;
    cv::namedWindow("HSVTrackbars", 0);
    cv::createTrackbar("H_MIN W", "HSVTrackbars", &H_MIN, 256, OnTrackbar);
    cv::createTrackbar("H_MAX W", "HSVTrackbars", &H_MAX, 256, OnTrackbar);
    cv::createTrackbar("S_MIN W", "HSVTrackbars", &S_MIN, 256, OnTrackbar);
    cv::createTrackbar("S_MAX W", "HSVTrackbars", &S_MAX, 256, OnTrackbar);
    cv::createTrackbar("V_MIN W", "HSVTrackbars", &V_MIN, 256, OnTrackbar);
    cv::createTrackbar("V_MAX W", "HSVTrackbars", &V_MAX, 256, OnTrackbar);
    cv::createTrackbar("Frame Select", "HSVTrackbars", &frame_select_, 7, OnTrackbar);
}

ArenaTracking::~ArenaTracking() {

}

void ArenaTracking::SetSubcribers() {
//    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, &ArenaTracking::CameraCallback, this);
    image_sub_ = it_.subscribe("/camera/image_raw", 1, &ArenaTracking::CameraCallback, this);
    pose_sub_ = nh_->subscribe("mavros/local_position/pose", 1, &ArenaTracking::PoseCallback, this);
    odometry_sub_ = nh_->subscribe("odometry/filtered", 1, &ArenaTracking::OdometryCallback, this);
}

void ArenaTracking::SetPublishers() {

}

void ArenaTracking::CameraCallback(const sensor_msgs::ImageConstPtr &msg) {
    input_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
    ProcessFrame(input_);
}

void ArenaTracking::PoseCallback(const geometry_msgs::PoseStampedConstPtr &msg) {
    pose_is_init_ = true;
    world_imu_pose_ = *msg;
}

void ArenaTracking::OdometryCallback(const nav_msgs::OdometryConstPtr &msg) {
    odometry_is_init_ = true;
    world_imu_odometry_ = *msg;
    grid_fitting_.SetWorldIMUPose(msg);
}

void ArenaTracking::ProcessFrame(const cv::Mat& src) {
    // Mark as initialized
    image_is_init_ = true;

    // Start process timer
    StartTimer();


    // Apply undistortion
#define UNDISTORT
#ifdef UNDISTORT
    Undistort(src, undistorted_);

    // Convert undistorted image to grayscale if necessary
    if (undistorted_.type() != CV_8UC1) {
        cv::cvtColor(undistorted_, working_copy_, CV_BGR2GRAY);
    } else {
        undistorted_.copyTo(working_copy_);
    }

    undistorted_.copyTo(display_copy_);
#else
    // Convert source image to grayscale if necessary
    if (src.type() != CV_8UC1) {
        cv::cvtColor(src, working_copy_, CV_BGR2GRAY);
    } else {
        src.copyTo(working_copy_);
    }

    src.copyTo(display_copy_);
#endif

    // Blur
    GaussianBlur(working_copy_, blurred_, cv::Size(7,7), 8, 8);

    WhiteThreshold(working_copy_, value_thresholded_);

    // Color threshold if image isn't grayscale
#ifdef UNDISTORT
    if (undistorted_.type() != CV_8UC1) {
        HSVFilter(undistorted_, hsv_thresholded_);
        SubtractThresholds(value_thresholded_, hsv_thresholded_, thresholded_);
    } else {
        thresholded_ = value_thresholded_;
    }
#else
    if (src.type() != CV_8UC1) {
        HSVFilter(src, hsv_thresholded_);
        SubtractThresholds(value_thresholded_, hsv_thresholded_, thresholded_);
    } else {
        thresholded_ = value_thresholded_;
    }
#endif

    // Apply sequence of processes for line detection
    FindEdges(thresholded_, edges_);
    FindLines(edges_, lines_);
    output_ = lines_;

    geometry_msgs::PoseWithCovarianceStampedPtr pose_covariance = grid_fitting_.GetEstimatedPose();
    if (pose_covariance != nullptr) {
        pose_pub_.PublishPose(*grid_fitting_.GetEstimatedPose());
    }

    pose_pub_.PublishPose(grid_fitting_.GetPose());
    pose_pub_.PublishPose2(grid_fitting_.GetPose2());
    pose_pub_.PublishPointCloud2(grid_fitting_.GetGlobalCloud());
    if (odometry_is_init_) {
        pose_pub_.PublishPointCloud(grid_fitting_.GetIntersectionCloud());
    }

    StopTimer();
    PrintProcessTime(output_);

}

void ArenaTracking::FindEdges(const cv::Mat &src, cv::Mat &dst) const {
    dilate(src, dst, element_, cv::Point(0), 8);
    GaussianBlur(src, dst, cv::Size(7,7), 1.5, 1.5);
    Canny(dst, dst, 1, 30, 3);
}

void ArenaTracking::FindLines(const cv::Mat &src, cv::Mat &dst) {
    std::vector<cv::Vec2f> raw_lines, pruned_lines_x, pruned_lines_y;
    std::vector<cv::Point> line_points_x, line_points_y;
    HoughLines(src, raw_lines, 1, CV_PI/180, 100, 0, 0 );
    dst = cv::Mat(src.size(), CV_8UC3, cv::Scalar(0, 0, 0));

    if (pose_is_init_) {
        pruned_lines_x = grid_fitting_.FitToGrid(raw_lines, world_imu_pose_, GridFitting::Orientation::X);
        pruned_lines_y = grid_fitting_.FitToGrid(raw_lines, world_imu_pose_, GridFitting::Orientation::Y);
        Eigen::Matrix2d center_points = grid_fitting_.GetPerspectivePoints();
        for (int i = 0; i < center_points.cols(); ++i) {
            cv::circle(display_copy_, cv::Point(center_points(i, 0), center_points(i, 1)), 20, cv::Scalar(255-105*i, 150, 150+105*i), 5, CV_AA);
            line(display_copy_, cv::Point(320, 240), cv::Point(center_points(i, 0), center_points(i, 1)), cv::Scalar(255-255*i, 0, 255*i), 3, CV_AA);
        }
    }

    // Show raw lines
    DrawRawLines(dst, raw_lines);

    // Show pruned lines
    DrawPrunedLines(dst, pruned_lines_x, line_points_x);
    DrawPrunedLines(dst, pruned_lines_y, line_points_y);

    // Cluster lines with density
    std::vector<int> line_clusters_x = DBSCAN::DBSCAN(line_points_x, 1000, 2);
    std::vector<int> line_clusters_y = DBSCAN::DBSCAN(line_points_y, 1000, 2);

    std::vector<cv::Point> line_centroids_x = findCentroids(line_clusters_x, line_points_x);
    std::vector<cv::Point> line_centroids_y = findCentroids(line_clusters_y, line_points_y);
    std::vector<cv::Point> intersections = findIntersections(line_clusters_x, line_clusters_y,
                                                             line_centroids_x, line_centroids_y);

    DrawLineClusters(dst, pruned_lines_x, line_clusters_x);
    DrawLineClusters(dst, pruned_lines_y, line_clusters_y);

    auto line_centroids_rho_theta_x = getRhoTheta(line_centroids_x);
    auto line_centroids_rho_theta_y = getRhoTheta(line_centroids_y);

    DrawLineCentroids(dst, line_centroids_x, line_centroids_rho_theta_x);
    DrawLineCentroids(dst, line_centroids_y, line_centroids_rho_theta_y);


    for (int i = 0; i < intersections.size(); ++i) {
        cv::circle(dst, intersections[i], 8, cv::Scalar(255, 255, 255), 2, CV_AA);
    }

    // Check if we've received an odometry message before computing depth
    if (!odometry_is_init_) return;

    auto intersection_depths = grid_fitting_.ComputeIntersectionDepths(intersections);
    auto rectified_intersections = grid_fitting_.RectifyIntersections(intersection_depths);
}

void ArenaTracking::DrawLineCentroids(cv::Mat &dst, const std::vector<cv::Point> &line_centroids,
                                      const std::vector<cv::Vec2f> &line_centroids_rho_theta) const {
    for (int i = 0; i < line_centroids.size(); ++i) {
        float rho = line_centroids_rho_theta[i][0], theta = line_centroids_rho_theta[i][1];
        cv::Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1.x = cvRound(x0 + 1000*(-b));
        pt1.y = cvRound(y0 + 1000*(a));
        pt2.x = cvRound(x0 - 1000*(-b));
        pt2.y = cvRound(y0 - 1000*(a));
        line(dst, pt1, pt2, cv::Scalar(0, 255, 255), 3, CV_AA);
    }
}

void ArenaTracking::DrawRawLines(cv::Mat &dst, const std::vector<cv::Vec2f> &raw_lines) const {
    for( size_t i = 0; i < raw_lines.size() && i < 100; i++ )
    {
        float rho = raw_lines[i][0], theta = raw_lines[i][1];
        cv::Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1.x = cvRound(x0 + 1000*(-b));
        pt1.y = cvRound(y0 + 1000*(a));
        pt2.x = cvRound(x0 - 1000*(-b));
        pt2.y = cvRound(y0 - 1000*(a));
        line(dst, pt1, pt2, cv::Scalar(100,100,100), 3, CV_AA);
    }
}

void ArenaTracking::DrawPrunedLines(cv::Mat &dst, const std::vector<cv::Vec2f> &pruned_lines,
                                    std::vector<cv::Point> &line_points) const {
    for( size_t i = 0; i < pruned_lines.size(); i++ )
    {
        float rho = pruned_lines[i][0], theta = pruned_lines[i][1];
        cv::Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1.x = cvRound(x0 + 1000*(-b));
        pt1.y = cvRound(y0 + 1000*(a));
        pt2.x = cvRound(x0 - 1000*(-b));
        pt2.y = cvRound(y0 - 1000*(a));
        line(dst, pt1, pt2, cv::Scalar(255,255,255), 3, CV_AA);

        line_points.push_back(cv::Point(x0, y0));
    }
}

void ArenaTracking::DrawLineClusters(cv::Mat &dst, const std::vector<cv::Vec2f> &pruned_lines,
                                     const std::vector<int> &line_clusters) const {
    for (int i = 0; i < line_clusters.size(); ++i) {
        if (line_clusters[i] < 0) continue;
        float rho = pruned_lines[i][0], theta = pruned_lines[i][1];
        cv::Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1.x = cvRound(x0 + 1000*(-b));
        pt1.y = cvRound(y0 + 1000*(a));
        pt2.x = cvRound(x0 - 1000*(-b));
        pt2.y = cvRound(y0 - 1000*(a));
        line(dst, pt1, pt2, cv::Scalar(255/(line_clusters[i]%7+1), 255/(line_clusters[i]%5+1), 0), 3, CV_AA);
    }
}

const cv::Mat& ArenaTracking::GetInputFrame(){
    switch(frame_select_) {
        case 0: return input_;
#ifdef UNDISTORT
        case 1: return undistorted_;
#endif
        case 2: return display_copy_;
        case 3: return value_thresholded_;
        case 4: return hsv_thresholded_;
        case 5: return thresholded_;
        case 6: return edges_;
        case 7: return lines_;
        default: return input_;
    }
}

const cv::Mat &ArenaTracking::GetProcessedFrame() {
    return output_;
}

void ArenaTracking::Undistort(const cv::Mat &src, cv::Mat &dst) {
    cv::remap(src, dst, distortion_map1_, distortion_map2_, CV_INTER_LINEAR);
}

void ArenaTracking::HSVFilter(const cv::Mat &src, cv::Mat &dst) {
    cv::cvtColor(src, hsv_, cv::COLOR_BGR2HSV);
    cv::inRange(hsv_, cv::Scalar(H_MIN, S_MIN, V_MIN), cv::Scalar(H_MAX, S_MAX, V_MAX), dst);
}

void ArenaTracking::SubtractThresholds(const cv::Mat &src1, const cv::Mat &src2, cv::Mat &dst) {
    cv::subtract(src1, src2, dst);
}

void ArenaTracking::WhiteThreshold(const cv::Mat &src, cv::Mat &dst) {
    cv::adaptiveThreshold(src, dst, 200, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY, 551, 2);
//    cv::threshold(src,dst, 0, 255, CV_THRESH_OTSU);
//    cv::threshold(src,dst, 125, 255, CV_THRESH_BINARY);
}

void ArenaTracking::StartTimer() {
    start_tick_ = cv::getTickCount();
}

void ArenaTracking::StopTimer() {
    time_difference_ = 1000 * (cv::getTickCount() - start_tick_) / cv::getTickFrequency();
}

void ArenaTracking::PrintProcessTime(cv::Mat &src) {
    sprintf(process_time_string_, "Processing time: %f ms", time_difference_);
    cv::putText(src, process_time_string_, cv::Point(10, 30), CV_FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255));
}

