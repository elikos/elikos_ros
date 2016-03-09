//
// Created by ta11e4rand on 10/02/16.
//
#include "BlobTracking.h"

/*
 * This method is the first step of the tracking algorithm
 * params: input: frame from the camera, output_*: output matrix with color, output: general output matrix
 */
void BlobTracking::track(const cv::Mat &input, cv::Mat &output_w, cv::Mat &output_r, cv::Mat &output_g, cv::Mat &output){

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
    trackList.emplace_front(getBlobObjects());

    //Count track() invocations
    trackCounter++;

    //overflow protection
    if (trackCounter>100000)
        trackCounter = TrackListLength;

}

void BlobTracking::trackRobots(const cv::Mat &input, cv::Mat &output){
    //Research of each robot in the last vector of trackList on the current frame
    for(auto it =trackList.back().begin(); it!=trackList.back().end(); it++) {
        //CAMshift  computation with the previous TrackListLength'th frame.
        //Info here: http://docs.opencv.org/master/db/df8/tutorial_py_meanshift.html#gsc.tab=0
        Mat hsv, hue, mask, hist, histimg = Mat::zeros(input.rows, input.cols, CV_8UC3), backproj;
        //Conversion to HSV
        cvtColor(input, hsv, COLOR_BGR2HSV); //TODO: Is it already available through output?
        inRange(hsv, Scalar(0, MIN(MIN(detection.S_MIN_G, detection.S_MIN_R),detection.S_MIN_W), MIN(MIN(detection.V_MIN_G, detection.V_MIN_R),detection.V_MIN_W)),
                Scalar(180, 256, MAX(MAX(detection.V_MAX_G, detection.V_MAX_R),detection.V_MAX_W)), mask);
        //temporary data for CamShift
        float hranges[] = {0,180};
        int hsize = 16;
        const float* phranges = hranges;
        int ch[] = {0, 0};
        hue.create(hsv.size(), hsv.depth());
        mixChannels(&hsv, 1, &hue, 1, ch, 1);
        Rect trackRect = it->getWindow().boundingRect();
        trackRect.x = it->getWindow().boundingRect().x;
        trackRect.y = it->getWindow().boundingRect().y;
        trackRect.width = it->getWindow().boundingRect().width;
        trackRect.height = it->getWindow().boundingRect().height;
        trackRect &= Rect(0, 0, input.cols, input.rows);

        //CamShift parameters
        Mat maskroi(mask, trackRect);
        Mat roi(hue, trackRect);
        calcHist(&roi, 1, 0, maskroi, hist, 1, &hsize, &phranges);
        normalize(hist, hist, 0, 255, NORM_MINMAX);
        calcBackProject(&hue, 1, 0, hist, backproj, &phranges);
        backproj &= mask;


        //The new position of the position of the robots is in trackWindow.
        //Maximum 10 iterations of computation
        RotatedRect trackWindow = CamShift(backproj, trackRect,
                                           TermCriteria(TermCriteria::EPS | TermCriteria::COUNT, 10, 1));



        //Update IDs with blobs detected on the previous TrackListLength'th frame.
        /*int maxID = 0;
        for (auto object: getBlobObjects()){
            maxID = max(maxID, object.getID());
            cout<<"object ID: "<<object.getID()<<endl;
        }
        bool found = false;
        for(auto object: getBlobObjects()){
            if(object.getXPos()<trackWindow.center.x+trackWindow.size.width/2 && object.getXPos()>trackWindow.center.x-trackWindow.size.width/2
                    && object.getYPos()<trackWindow.center.y+trackWindow.size.height/2 &&object.getYPos()>trackWindow.center.y-trackWindow.size.height/2) {
                object.setID(it->getID());
                found = true;
            }
        }
        if(!found) it->setID(maxID++);*/

        //Compute and display the direction of the trackWindow
        double direction = fastAtan2(trackWindow.center.y-it->getYPos(),trackWindow.center.x-it->getXPos())/360*2*PI;
        it->setDirection(direction);
        displayDirection(*it, output);

    }
}

void BlobTracking::detectColor(const cv::Mat &input, cv::Mat &output_w, cv::Mat &output_r, cv::Mat &output_g, cv::Mat &output){
    detection.detectColor(input, output_w, output_r, output_g, output);
}

vector<RobotDesc> BlobTracking::getBlobObjects(){
    return detection.getBlobObjects();
}
//Method to display the direction arrow (for tests usage only)
void BlobTracking::displayDirection(RobotDesc robot, cv::Mat &output){
    //center
    Point center(cvRound(robot.getXPos()), cvRound(robot.getYPos()));
    //second point
    Point second(cvRound(robot.getXPos()+30*cos(robot.getDirection())), cvRound(robot.getYPos())+30*sin(robot.getDirection()));
    //arrow
    line(output,center, second, Scalar(0,255,0),3,8,0);
    string text = to_string(robot.getID());
    int fontFace = FONT_HERSHEY_SCRIPT_SIMPLEX;
    double fontScale = 2;
    int thickness = 3;
    cv::Point pos(robot.getXPos(), robot.getYPos());
    cv::putText(output, text, pos, fontFace, fontScale, Scalar::all(0), thickness,8);
}

