//
// Created by ta11e4rand on 10/02/16.
//

#ifndef DETECTION_AND_TRACKING_BLOBTRACKING_H
#define DETECTION_AND_TRACKING_BLOBTRACKING_H
#include "RobotDetection.h"
#include <list>
class BlobTracking {

public:
    //Robot tracking algorithm. Includes color and robot detection
    void track(const cv::Mat &input, cv::Mat &output_w, cv::Mat &output_r, cv::Mat &output_g, cv::Mat &output);
    //Method to detect blobs of color on frames
    void detectColor(const cv::Mat &input, cv::Mat &output_w, cv::Mat &output_r, cv::Mat &output_g, cv::Mat &output);
    //Getters for the output vector of the algorithm
    vector<RobotDesc> getBlobObjects();
    int getTrackCounter(){return trackCounter;}
    void setPrevPoints(vector<cv::Point2f> p){
        prevPoints=p;
    }
private:
    //Computation methods
    void displayDirection(RobotDesc robot, cv::Mat &output);
    void trackRobots(const cv::Mat &input, cv::Mat &output);

    //Attributes
    //Detection class
    RobotDetection detection;
    //Update rate of the direction (=1 means "at each frame" or " at frame rate")
    static const int TrackingRate = 1;
    //To avoid chaotic computed direction between two frame, the comparison is applied between the actual frame and the TrackListLength'th last frame.
    static const int TrackListLength = 3;
    //Counter of the number of call of the method track()
    int trackCounter = 0;
    vector<cv::Point2f> prevPoints;
    //List containing the past vectors of detected robots. Its length is equal to TrackListLength.
    list<vector<RobotDesc>> trackList;
};

#endif //DETECTION_AND_TRACKING_BLOBTRACKING_H
