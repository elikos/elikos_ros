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
    void createTrackbars();

    //Robot detection algorithm
    void detectRobots(const cv::Mat &input, cv::Mat &output);
    //Color blobs detection algotrithm
    void detectColor(const cv::Mat &input, cv::Mat &output_w, cv::Mat &output_r, cv::Mat &output_g, cv::Mat &output);
    //Getters
    vector<RobotDesc>& getFoundRobots();
    vector<RobotDesc> getFoundObstacles();
    vector<RobotDesc> getRobotsNotConfirmed();
    vector<RobotDesc> getBlobObjects();


private:
    //Methods used in detection algorithms
    void displayCircle(const RobotDesc& circle, cv::Mat &output);
    void displayBlobMarker(const RobotDesc& object, cv::Mat &output);
    string intToString(int number);
    void saveCircles();
    void robotsVerification(cv::Mat &output);
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
    int H_MAX_W = 256;
    int S_MIN_W = 0;
    int S_MAX_W = 256;
    int V_MIN_W = 0;
    int V_MAX_W = 256;
    int H_MIN_G = 0;
    int H_MAX_G = 256;
    int S_MIN_G = 0;
    int S_MAX_G = 256;
    int V_MIN_G = 0;
    int V_MAX_G = 256;
    int H_MIN_R = 0;
    int H_MAX_R = 256;
    int S_MIN_R = 0;
    int S_MAX_R = 256;
    int V_MIN_R = 0;
    int V_MAX_R = 256;
    const int FRAME_WIDTH = 640;
    const int FRAME_HEIGHT = 480;

private:
    int PRE_EROSIONS_W = 0;
    int DILATIONS_W = 25;
    int POST_EROSIONS_W = 0;
    int PRE_EROSIONS_G = 0;
    int DILATIONS_G = 25;
    int POST_EROSIONS_G = 0;

    int PRE_BLUR = 2;
    int CANNY_THRESH1 = 1;
    int CANNY_THRESH2 = 1;
    int CANNY_APERTURE = 3;
    int POLY_AREA_MIN = 0;
    int POLY_AREA_MAX = 50000;
    int MORPH_OP = 0;
    int MORPH_ELEMENT = 0;
    int MORPH_SIZE = 0;
    int MAX_DIST = 100;

    const int MAX_NUM_OBJECTS = 50;

    const int MIN_OBJECT_AREA = 40 * 40;
    const int MAX_OBJECT_AREA = FRAME_HEIGHT * FRAME_WIDTH / 1.5;
    const string trackbarWindowName = "Trackbars";
    const string HSVTrackbars = "HSV Trackbars";
    const string shapeDetectTrackbars = "Shape detector";

    int maxID = 0;
};


#endif //DETECTION_AND_TRACKING_ROBOTDETECTION_H
