//
// Created by ta11e4rand on 10/02/16.
//

#ifndef DETECTION_AND_TRACKING_TargetDetection_H
#define DETECTION_AND_TRACKING_TargetDetection_H
#include "BlobDetection.h"

#include <list>
class TargetDetection {

public:
	TargetDetection();
    //Robot tracking algorithm. Includes color and robot detection
    void detect(const cv::Mat &input, cv::Mat &output_w, cv::Mat &output_r, cv::Mat &output_g, cv::Mat &output);
    //Method to detect blobs of color on frames
    void detectColor(const cv::Mat &input, cv::Mat &output_w, cv::Mat &output_r, cv::Mat &output_g, cv::Mat &output);

    void displayID(RobotDesc robot, cv::Mat &output);

	void createTrackbars(){detection.createTrackbars();}
	void saveCalibration(string filename){detection.saveCalibration(filename);}
	void loadCalibration(string filename){detection.loadCalibration(filename);}
	
    //Getters for the output vector of the algorithm
    vector<RobotDesc> getRobots();
private:
    //Computation methods
    void emplaceNewRobots(cv::Mat &output);
    vector<RobotDesc> robots;
    //Attributes
    //Detection class
    BlobDetection detection;
};

#endif //DETECTION_AND_TRACKING_TargetDetection_H
