//
// Created by ta11e4rand on 10/02/16.
//
#include "TargetDetection.h"

TargetDetection::TargetDetection(){
}
/*
 * This method is the first step of the tracking algorithm
 * params: input: frame from the camera, output_*: output matrix with color, output: general output matrix
 */
void TargetDetection::detect(const cv::Mat &input, cv::Mat &output_w, cv::Mat &output_r, cv::Mat &output_g, cv::Mat &output){

    //Color detection
    detectColor(input, output_w, output_r, output_g, output);
    
    emplaceNewRobots(output);
}

void TargetDetection::emplaceNewRobots(cv::Mat &output){
	//Reset robot container
    robots.erase(robots.begin(),robots.end());

	//Fill container
	for(auto object : detection.getBlobObjects()){
		displayID(object, output);
		robots.emplace_back(object);
	}
}

void TargetDetection::detectColor(const cv::Mat &input, cv::Mat &output_w, cv::Mat &output_r, cv::Mat &output_g, cv::Mat &output){
    detection.detectColor(input, output_w, output_r, output_g, output);
}

vector<RobotDesc> TargetDetection::getRobots(){
    return robots;
}
//Method to display the direction arrow (for tests usage only)
void TargetDetection::displayID(RobotDesc robot, cv::Mat &output){
    //center
    Point center(cvRound(robot.getXPos()), cvRound(robot.getYPos()));
    //circle
    if(robot.getColor() == ColorsIndex::RED){
		circle(output, center, 5, Scalar(0,0,255));
	}
	else if(robot.getColor() == ColorsIndex::GREEN){
		circle(output, center, 5, Scalar(0,255,0));
	}
    string text = to_string(robot.getID());
    int fontFace = FONT_HERSHEY_SCRIPT_SIMPLEX;
    double fontScale = 2;
    int thickness = 3;
    cv::Point pos(robot.getXPos(), robot.getYPos());
    cv::putText(output, text, pos, fontFace, fontScale, Scalar(70,70,70), thickness,8);
}
