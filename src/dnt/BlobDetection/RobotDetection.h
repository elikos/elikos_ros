//
// Created by ta11e4rand on 19/11/15.
//

#ifndef DETECTION_AND_TRACKING_ROBOTDETECTION_H
#define DETECTION_AND_TRACKING_ROBOTDETECTION_H

#include <string>
#include <opencv2/opencv.hpp>
#include "RobotDesc.h"
#include "GreenColor.h"
#include "RedColor.h"
#include "WhiteColor.h"

using namespace std;
using namespace cv;

//This class implements the detection of robots
class RobotDetection {
public:
    //constructor
    RobotDetection();

    //Trackbars
    //void createTrackbars();

    //Robot detection algorithm
    //Color blobs detection algotrithm
    void detectColor(const cv::Mat &input, cv::Mat &output_w, cv::Mat &output_r, cv::Mat &output_g, cv::Mat &output);
    //Getters
    vector<RobotDesc> getBlobObjects();


private:
    //Methods used in detection algorithms
    void displayBlobMarker(const RobotDesc& object, cv::Mat &output);
    string intToString(int number);
    // ATTRIBUTES
    //Data used in computation
    Mat currentImage;
    vector<RobotDesc> foundCircles;
    vector<RobotDesc> whiteObjects;
    vector<RobotDesc> greenObjects;
    vector<RobotDesc> redObjects;
    vector<Vec3f> circles;
    vector<Mat> thresholds;
    vector<Color*> colors;

    //Final data (can be fetched by other classes through the getters)
    vector<RobotDesc> foundRobots;
    vector<RobotDesc> foundObstacles;
    vector<RobotDesc> robotsNotConfirmed;
    vector<RobotDesc> blobObjects;

public://Public constants, used in TrackingCircles
    // CLASS CONSTANTS
    int H_MIN_W = 0;
    int H_MAX_W = 52;
    int S_MIN_W = 0;
    int S_MAX_W = 149;
    int V_MIN_W = 140;
    int V_MAX_W = 256;
    int H_MIN_G = 39;
    int H_MAX_G = 88;
    int S_MIN_G = 54;
    int S_MAX_G = 256;
    int V_MIN_G = 30;
    int V_MAX_G = 256;
    int H_MIN_R = 0;
    int H_MAX_R = 48;
    int S_MIN_R = 93;
    int S_MAX_R = 256;
    int V_MIN_R = 141;
    int V_MAX_R = 256;
    const int FRAME_WIDTH = 640;
    const int FRAME_HEIGHT = 480;

private:
};


#endif //DETECTION_AND_TRACKING_ROBOTDETECTION_H
