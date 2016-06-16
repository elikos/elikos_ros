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
#include <fstream>

using namespace std;
using namespace cv;

//This class implements the detection of robots
class RobotDetection {
public:
    //constructor
    RobotDetection();

    //Trackbars
    void createTrackbars();

    //Robot detection algorithm
    //Color blobs detection algotrithm
    void detectColor(const cv::Mat &input, cv::Mat &output_w, cv::Mat &output_r, cv::Mat &output_g, cv::Mat &output);
    //Getters
    vector<RobotDesc> getBlobObjects();

    void saveCalibration(string filename);
    void loadCalibration(string filename);
private:
    // ATTRIBUTES
    //Data used in computation
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
    //Final data (can be fetched by other classes through the getters)
    vector<RobotDesc> foundRobots;
    vector<RobotDesc> foundObstacles;
    vector<RobotDesc> robotsNotConfirmed;
    vector<RobotDesc> blobObjects;

public://Public constants, used in TrackingCircles
    // CLASS CONSTANTS
    WhiteColor* getWhite(){return &whiteColor_;}
    RedColor* getRed(){return &redColor_;}
    GreenColor* getGreen(){return &greenColor_;}
    const int FRAME_WIDTH = 640;
    const int FRAME_HEIGHT = 480;
private:
};
#endif //DETECTION_AND_TRACKING_ROBOTDETECTION_H
