#ifndef DETECTION_AND_TRACKING_BLOBDETECTION_H
#define DETECTION_AND_TRACKING_BLOBDETECTION_H


/* TODO:se débarasser des defines ci-dessus */


#include <string>
#include <opencv2/opencv.hpp>
#include "RobotDesc.h"
#include "GreenColor.h"
#include "RedColor.h"
#include "WhiteColor.h"

using namespace std;
using namespace cv;

class BlobDetection {
public:

    void trackBlobs(const cv::Mat &input, cv::Mat &threshold_w, cv::Mat &threshold_r, cv::Mat &threshold_g);
    string intToString(int number);
    void createTrackbars();


private:

    // ATTRIBUTES
    Mat currentImage;
    vector<RobotDesc> foundRobots;

    // DEBUG ATTRIBUTES
    Mat threshold_w;
    Mat threshold_g;
    Mat threshold_r;
    Mat hsv_w;
    Mat hsv_c;
    Mat mask;
    Mat closeWhite;
    Mat cropped_hsv;
    Mat grayscale_image;
    Mat canny;
    Mat morph_ex;
    Mat contour_drawings;
    VideoCapture capture;

    // CLASS CONSTANTS
    int H_MIN_W = 0;
    int H_MAX_W = 52;
    int S_MIN_W = 0;
    int S_MAX_W = 149;
    int V_MIN_W = 140;
    int V_MAX_W = 256;
    int H_MIN_G = 30;
    int H_MAX_G = 100;
    int S_MIN_G = 0;
    int S_MAX_G = 256;
    int V_MIN_G = 0;
    int V_MAX_G = 256;
    int H_MIN_R = 0;
    int H_MAX_R = 48;
    int S_MIN_R = 0;
    int S_MAX_R = 256;
    int V_MIN_R = 0;
    int V_MAX_R = 256;
    
    int PRE_EROSIONS_W = 12;
    int DILATIONS_W = 25;
    int POST_EROSIONS_W = 5;
    int PRE_EROSIONS_G = 3;
    int DILATIONS_G = 20;
    int POST_EROSIONS_G = 0;
    
    int PRE_BLUR = 2;
    int BLUR_AMOUNT;
    int CANNY_THRESH1 = 1;
    int CANNY_THRESH2 = 1;
    int CANNY_APERTURE = 3;
    int POLY_AREA_MIN = 0;
    int POLY_AREA_MAX = 50000;
    int MORPH_OP = 0;
    int MORPH_ELEMENT = 0;
    int MORPH_SIZE = 0;
    int MAX_DIST = 100;

    const int FRAME_WIDTH = 640;
    const int FRAME_HEIGHT = 480;

    const int MAX_NUM_OBJECTS = 50;

    const int MIN_OBJECT_AREA = 40 * 40;
    const int MAX_OBJECT_AREA = FRAME_HEIGHT * FRAME_WIDTH / 1.5;

    const double CAMERA_FOV_H = 120 * 3.1415926 / 180.0;
    const double CAMERA_FOV_V = 66 * 3.1415926 / 180.0;

    const string windowName = "Original Image";
    const string windowName1 = "HSV Image";
    const string windowName2 = "Thresholded Image";
    const string windowName3 = "After Morphological Operations";
    const string trackbarWindowName = "Trackbars";
    const string HSVTrackbars = "HSV Trackbars";
    const string shapeDetectTrackbars = "Shape detector";

};

#endif //DETECTION_AND_TRACKING_EDGEDETECTION_H
