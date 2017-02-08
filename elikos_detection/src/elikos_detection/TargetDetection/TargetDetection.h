//
// Created by ta11e4rand on 10/02/16.
//

#ifndef DETECTION_AND_TRACKING_TargetDetection_H
#define DETECTION_AND_TRACKING_TargetDetection_H
#include "BlobDetection.h"

#include <list>
class TargetDetection
{

  public:
    TargetDetection();
    //Robot tracking algorithm. Includes color and robot detection
    void detect(const cv::Mat &input, cv::Mat &output_w, cv::Mat &output_r, cv::Mat &output_g, cv::Mat &output);

    void displayID(RobotDesc robot, cv::Mat &output);

    void createTrackbars() { blobDetection_.createTrackbars(); }
    void saveCalibration(string filename) { blobDetection_.saveCalibration(filename); }
    void loadCalibration(string filename) { blobDetection_.loadCalibration(filename); }
    std::string getAllParams() { return blobDetection_.getAllParams(); }

    void updateHSV(int color, int h, int s, int v, int delta);
    void fetchRemoteParams(int color, int h_max, int h_min, int s_max, int s_min, int v_max, int v_min,int preErode, int dilate, int postErode);

    //Getters for the output vector of the algorithm
    vector<RobotDesc> getRobots();

  private:
    //Computation methods
    void emplaceNewRobots(cv::Mat &output);
    vector<RobotDesc> robots;
    //Attributes
    //Detection class
    BlobDetection blobDetection_;
};

#endif //DETECTION_AND_TRACKING_TargetDetection_H
