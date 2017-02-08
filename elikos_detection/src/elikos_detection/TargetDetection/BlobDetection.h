//
// Created by ta11e4rand on 19/11/15.
//

#ifndef DETECTION_AND_TRACKING_BLOBDETECTION_H
#define DETECTION_AND_TRACKING_BLOBDETECTION_H

#include <tf/transform_listener.h>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <string>
#include "GreenColor.h"
#include "RedColor.h"
#include "RobotDesc.h"
#include "WhiteColor.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;
using namespace cv;

// This class implements the detection of robots
class BlobDetection {
   public:
    // constructor
    BlobDetection();

    void detect(const cv::Mat& input, cv::Mat& output_w, cv::Mat& output_r,
                cv::Mat& output_g, cv::Mat& output,
                std::vector<RobotDesc>& robotsArray);

    // Trackbars --> TODO: remove..
    void createTrackbars();

    void saveCalibration(string filename);
    void loadCalibration(string filename);

    /* TODO: REMOVE */
    void removePerspective(const cv::Mat& input, cv::Mat& rectified);
    Eigen::Matrix4f getPerspectiveProjectionTransform(double focalLength,
                                                      double width,
                                                      double height);
    void preProcessImage(const cv::Mat& raw, cv::Mat& preProcessed);

    std::string getAllParams();

   private:
    // ATTRIBUTES
    // Data used in computation
    Mat currentImage;
    Mat trackbarsWhiteMat_;
    Mat trackbarsRedMat_;
    Mat trackbarsGreenMat_;
    vector<RobotDesc> foundCircles;
    vector<RobotDesc> whiteObjects;
    vector<RobotDesc> greenObjects;
    vector<RobotDesc> redObjects;
    vector<Vec3f> circles;
    vector<Mat> thresholds;

    WhiteColor whiteColor_;
    RedColor redColor_;
    GreenColor greenColor_;

    int maxID;
    /* TODO: Remove this....*/
    // Final data (can be fetched by other classes through the getters)
    vector<RobotDesc> foundRobots;
    vector<RobotDesc> foundObstacles;
    vector<RobotDesc> robotsNotConfirmed;
    vector<RobotDesc> blobObjects;

    tf::TransformListener tfListener_;

    // Color blobs detection algotrithm
    void detectColor(const cv::Mat& input, cv::Mat& output_w, cv::Mat& output_r,
                     cv::Mat& output_g, cv::Mat& output,
                     std::vector<RobotDesc>& robotsArray);
    // Circle blobs detection algotrithm
    void detectCircles(const cv::Mat& input, cv::Mat& output_w,
                       cv::Mat& output_r, cv::Mat& output_g, cv::Mat& output,
                       std::vector<RobotDesc>& robotsArray);

   public:  // Public constants, used in TrackingCircles
    // CLASS CONSTANTS
    WhiteColor* getWhite() { return &whiteColor_; }
    RedColor* getRed() { return &redColor_; }
    GreenColor* getGreen() { return &greenColor_; }
    const int FRAME_WIDTH = 640;
    const int FRAME_HEIGHT = 480;

   private:
};
#endif  // DETECTION_AND_TRACKING_BLOBDETECTION_H
