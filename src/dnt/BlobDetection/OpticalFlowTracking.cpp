//
// Created by ta11e4rand on 28/12/15.
//

#include "OpticalFlowTracking.h"

void OpticalFlowTracking::track(const cv::Mat &input, cv::Mat &output_w, cv::Mat &output_r, cv::Mat &output_g, cv::Mat &output){
    //Color detection
    detectColor(input, output_w, output_r, output_g, output);
    /*
     * Tracking
    */
    //The tracking rate depends on TrackingRate and needs at least as many detections as TrackListLength before to start.
    if(trackCounter>TrackListLength && trackCounter%TrackingRate==0)
        trackRobots(input, output);
    //Update trackList
    if(trackCounter>=TrackListLength)
        trackList.pop_back();

    trackList.emplace_front(make_pair(input,getBlobObjects()));
    //Count track() invocations
    trackCounter++;

    //overflow protection
    if (trackCounter>100000)
        trackCounter = TrackListLength;

}

void OpticalFlowTracking::trackRobots(const cv::Mat &input, cv::Mat &output){

    //Research of each robot in the last vector of trackList on the current frame
    auto pastInfo = trackList.back();
    vector<cv::Point2f> nextPoints;
    vector<uchar> status;
    Mat errors;
    /*for(RobotDesc robot: pastInfo.second){
        prevPoints.emplace_back(Point2f(robot.getXPos(),robot.getYPos()));
    }*/
    if(prevPoints.size()>0)
        calcOpticalFlowPyrLK(pastInfo.first,input,prevPoints,nextPoints, status, errors);
    int i =0;
    //for(RobotDesc robot: pastInfo.second) {
    for(auto point : nextPoints){
        double direction =
                fastAtan2(nextPoints.at(i).y - prevPoints.at(i).y, nextPoints.at(i).x - prevPoints.at(i).x) / 360 * 2 * PI;
        RobotDesc robot;
        robot.setXPos(prevPoints.at(i).x);
        robot.setYPos(prevPoints.at(i).y);
        robot.setDirection(direction);
        displayDirection(robot, output);
        i++;
    }
}

void OpticalFlowTracking::detectColor(const cv::Mat &input, cv::Mat &output_w, cv::Mat &output_r, cv::Mat &output_g, cv::Mat &output){
    detection.detectColor(input, output_w, output_r, output_g, output);
}

vector<RobotDesc> OpticalFlowTracking::getBlobObjects(){
    return detection.getBlobObjects();
}
//Method to display the direction arrow (for tests usage only)
void OpticalFlowTracking::displayDirection(RobotDesc robot, cv::Mat &output){
    //center
    Point center(cvRound(robot.getXPos()), cvRound(robot.getYPos()));
    //second point
    Point second(cvRound(robot.getXPos()+10*cos(robot.getDirection())), cvRound(robot.getYPos())+10*sin(robot.getDirection()));
    //arrow
    line(output,center, second, Scalar(0,255,0),3,8,0);
}

