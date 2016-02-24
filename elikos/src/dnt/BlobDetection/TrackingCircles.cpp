//
// Created by ta11e4rand on 28/12/15.
//

#include "TrackingCircles.h"

void TrackingCircles::track(const cv::Mat &input, cv::Mat &output_w, cv::Mat &output_r, cv::Mat &output_g, cv::Mat &output){
    //Color detection
    detectColor(input, output_w, output_r, output_g, output);
    /*
     * Tracking
    */
    //The tracking rate depends on TrackingRate and needs at least as many detections as TrackListLength before to start.
    if(trackCounter>TrackListLength && trackCounter%TrackingRate==0)
        trackRobots(input, output);
    /*
     * Detection of the robots with the class RobotDetection
     * TODO: Merge CamShift tracking and detectRobots
     */
    detectRobots(input,output);
    //Update trackList
    if(trackCounter<TrackListLength)
        trackList.emplace_front(getFoundRobots());
    else {
        trackList.pop_back();
        trackList.emplace_front(getFoundRobots());
    }
    //Count track() invocations
    trackCounter++;

    //overflow protection
    if (trackCounter>10000)
        trackCounter = TrackListLength;

}

void TrackingCircles::trackRobots(const cv::Mat &input, cv::Mat &output){
    //Research of each robot in the last vector of trackList on the current frame
    for(auto it =trackList.back().begin(); it!=trackList.back().end(); it++) {
        Mat hsv, hue, mask, hist, histimg = Mat::zeros(input.rows, input.cols, CV_8UC3), backproj;
        //Conversion to HSV
        cvtColor(input, hsv, COLOR_BGR2HSV); //TODO: Is it already available through output?
        inRange(hsv, Scalar(0, MIN(MIN(detection.S_MIN_G, detection.S_MIN_R),detection.S_MIN_W), MIN(MIN(detection.V_MIN_G, detection.V_MIN_R),detection.V_MIN_W)),
                Scalar(180, 256, MAX(MAX(detection.V_MAX_G, detection.V_MAX_R),detection.V_MAX_W)), mask);
        //temporary data
        float hranges[] = {0,180};
        int hsize = 16;
        const float* phranges = hranges;
        int ch[] = {0, 0};
        hue.create(hsv.size(), hsv.depth());
        mixChannels(&hsv, 1, &hue, 1, ch, 1);
        //CAMshift  computation
        //Info here: http://docs.opencv.org/master/db/df8/tutorial_py_meanshift.html#gsc.tab=0
        try {
            Mat maskroi(mask, it->getWindow().boundingRect());
            Mat roi(hue, it->getWindow().boundingRect());
            calcHist(&roi, 1, 0, maskroi, hist, 1, &hsize, &phranges);
            normalize(hist, hist, 0, 255, NORM_MINMAX);
            calcBackProject(&hue, 1, 0, hist, backproj, &phranges);
            backproj &= mask;

            Rect trackRect = it->getWindow().boundingRect();
            //10 iterations maximum
            RotatedRect trackWindow = CamShift(backproj, trackRect,
                                           TermCriteria(TermCriteria::EPS | TermCriteria::COUNT, 10, 1));
            //Compute direction trackWindow
            double direction = fastAtan2(trackWindow.center.y-it->getYPos(),trackWindow.center.x-it->getXPos())/360*2*PI;
            it->setDirection(direction);
            displayDirection(*it, output);
        }
        catch(cv::Exception){
            cerr<<"exception lancée dans le tracking!";//TODO: à gérer
        }
    }
}
//Method to invoke the detections directly from RobotDetection.
void TrackingCircles::detectRobots(const cv::Mat &input,cv::Mat &output){
    detection.detectRobots(input,output);
}

void TrackingCircles::detectColor(const cv::Mat &input, cv::Mat &output_w, cv::Mat &output_r, cv::Mat &output_g, cv::Mat &output){
    detection.detectColor(input, output_w, output_r, output_g, output);
}
//Getters
vector<RobotDesc> TrackingCircles::getFoundRobots(){
    return detection.getFoundRobots();
}
vector<RobotDesc> TrackingCircles::getFoundObstacles(){
    return detection.getFoundObstacles();
}
//Method to display the direction arrow (for tests usage only)
void TrackingCircles::displayDirection(RobotDesc robot, cv::Mat &output){
    //center
    Point center(cvRound(robot.getXPos()), cvRound(robot.getYPos()));
    //second point
    Point second(cvRound(robot.getXPos()+60*cos(robot.getDirection())), cvRound(robot.getYPos())+60*sin(robot.getDirection()));
    //arrow
    line(output,center, second, Scalar(0,255,0),3,8,0);
}

