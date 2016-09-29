//
// Created by Antonio Sanniravong on 21/12/15.
//

#ifndef ARENA_TRACKING_H
#define ARENA_TRACKING_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <DBSCAN.h>
#include <PosePublisher.h>
#include <LineIntersections.h>
#include <GridFitting.h>
#include "PositionEstimator.h"

class ArenaTracking {
public:
    ArenaTracking(ros::NodeHandle* nh);
    ~ArenaTracking();
    const cv::Mat& GetInputFrame();
    const cv::Mat& GetProcessedFrame();
    float GetProcessingTime() const { return time_difference_; }
    bool isInitialized() const { return image_is_init_; }
private:
    // ArenaTracking private processes
    void SetSubcribers();
    void SetPublishers();
    void ProcessFrame(const cv::Mat& src);
    void CameraCallback(const sensor_msgs::ImageConstPtr &msg);
    void PoseCallback(const geometry_msgs::PoseStampedConstPtr &msg);
    void OdometryCallback(const nav_msgs::OdometryConstPtr &msg);
    void Undistort(const cv::Mat &src, cv::Mat &dst);
    void HSVFilter(const cv::Mat &src, cv::Mat &dst);
    void WhiteThreshold(const cv::Mat &src, cv::Mat &dst);
    void SubtractThresholds(const cv::Mat &src1, const cv::Mat &src2, cv::Mat &dst);
    void FindLines(const cv::Mat &src, cv::Mat &dst);
    void FindEdges(const cv::Mat &src, cv::Mat &dst) const;
    void StartTimer();
    void StopTimer();
    void PrintProcessTime(cv::Mat &src);

    // ROS
    ros::NodeHandle* nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber odometry_sub_;
    PosePublisher pose_pub_;
    GridFitting grid_fitting_;
    PositionEstimator position_estimator_;

    // Attributes
    bool image_is_init_ = false;
    bool pose_is_init_ = false;
    bool odometry_is_init_ = false;

    // Poses
    geometry_msgs::PoseStamped world_imu_pose_;
    nav_msgs::Odometry world_imu_odometry_;

    // Matrix buffers
    cv::Mat input_, hsv_, hsv_thresholded_, blurred_, display_copy_, working_copy_, edges_, output_, value_thresholded_,
            thresholded_, undistorted_, lines_;
    cv::Mat element_ = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));

    // Camera calibration
    cv::Mat camera_matrix_, camera_matrix_new_, camera_distortion_, distortion_map1_, distortion_map2_;

    // Timer variable
    int64 start_tick_;
    float time_difference_;
    char process_time_string_[30];

    // Drawing methods
    void DrawLineClusters(cv::Mat &dst, const std::vector<cv::Vec2f> &pruned_lines,
                          const std::vector<int> &line_clusters) const;

    void DrawPrunedLines(cv::Mat &dst, const std::vector<cv::Vec2f> &pruned_lines, std::vector<cv::Point> &line_points) const;

    void DrawRawLines(cv::Mat &dst, const std::vector<cv::Vec2f> &raw_lines) const;

    void DrawLineCentroids(cv::Mat &dst, const std::vector<cv::Point> &line_centroids,
                           const std::vector<cv::Vec2f> &line_centroids_rho_theta) const;

    // HSV values
    int H_MIN, H_MAX, S_MIN, S_MAX, V_MIN, V_MAX;

    // Frame selector
    int frame_select_ = 0;
};


#endif //ARENA_TRACKING_H
