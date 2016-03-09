#include <opencv2/opencv.hpp>
#include <string>
#include "../BlobDetection/RobotDesc.h"
#ifndef DETECTION_AND_TRACKING_EDGEDETECTION_H
#define DETECTION_AND_TRACKING_EDGEDETECTION_H
using namespace std;
using namespace cv;

class EdgeDetection {
public:
    static void FindEdges(const cv::Mat &input, cv::Mat &output);
    void trackCircles(const cv::Mat &input, cv::Mat &output);
    EdgeDetection();
private:
    int nbRobotsTot =0;//nombre total de robots détectés
    //int ancPos[20][2];//array pour stocker les anciennes positions des robots. Maximum de 20 robots à la fois.
    vector<RobotDesc> robots;

    const int FRAME_WIDTH = 640;
    const int FRAME_HEIGHT = 480;
};


#endif //DETECTION_AND_TRACKING_EDGEDETECTION_H
