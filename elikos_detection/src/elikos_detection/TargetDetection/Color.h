#ifndef DETECTION_AND_TRACKING_COLOR_H
#define DETECTION_AND_TRACKING_COLOR_H

#include <opencv2/opencv.hpp>
#include "RobotDesc.h"
#include "CVWrapperInterface.h"

using namespace std;
using namespace cv;

class Color
{
public:
    Color();
    ~Color();

    Mat generateThreshold(const Mat& image);
    void trackFilteredObjects(Mat &cameraFeed);
    virtual void printFoundObjects();
    virtual void drawFoundObjects(Mat& image);
    vector<RobotDesc> getObjects();
	virtual void createTrackbars(string windowName){};

    Mat& getThreshold();

	//Calibration values
    int* H_MIN;
    int* H_MAX;
    int* S_MIN;
    int* S_MAX;
    int* V_MIN;
    int* V_MAX;
    int* PRE_EROSIONS;
    int* DILATIONS;
    int* POST_EROSIONS;
    
protected:
    Mat threshold;
    Mat hsv;
    vector<RobotDesc> foundObjects;
	CVWrapperInterface* cvWrapper;

    int PRE_BLUR = 2;
    int BLUR_AMOUNT;

    const int FRAME_WIDTH = 640;
    const int FRAME_HEIGHT = 480;

    const int MAX_NUM_OBJECTS = 50;

    const int MIN_OBJECT_AREA = 40 * 40;
    const int MAX_OBJECT_AREA = FRAME_HEIGHT * FRAME_WIDTH / 1.5;

private:
    string intToString(int number);
};


#endif //DETECTION_AND_TRACKING_COLOR_H