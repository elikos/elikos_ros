//
// Created by ta11e4rand on 19/11/15.
//

#include "RobotDetection.h"

RobotDetection::RobotDetection(){
    //foundCircles initialisation
    //Maximum of 20 circles detected at the same time
    //It allows to loose a circle for a few frames without loosing its information
    for (int j = 0; j < 20; j++) {
        foundCircles.emplace_back(RobotDesc(j, 0, 0));
    }
}

//Color detection algorithm
void RobotDetection::detectColor(const cv::Mat &input, cv::Mat &output_w, cv::Mat &output_r, cv::Mat &output_g, cv::Mat &output){
    //Check if the input is empty
    if(!input.data)
        cerr << "Input of detecRobots is empty";//TODO:throw an exception
    currentImage = input;

    //Blob detection with color thresholds
    colors = {new WhiteColor, new GreenColor, new RedColor};

    for(Color* color : colors) {
        thresholds.emplace_back(color->generateThreshold(currentImage));
        color->trackFilteredObjects(currentImage);
    }
    output_w = thresholds[WHITE];
    output_g = thresholds[GREEN];
    output_r = thresholds[RED];

    //extraction of white, red and green objects (robots)
    whiteObjects = colors.at(0)->getObjects();
    greenObjects = colors.at(1)->getObjects();
    redObjects = colors.at(2)->getObjects();
    int maxID = 0;
    for (auto object: blobObjects){
        maxID = max(maxID, object.getID());
    }
    auto oldObjects = blobObjects;
    blobObjects.erase(blobObjects.begin(),blobObjects.end());

    for(auto object : greenObjects){
        object.setColor(GREEN);
        object.setWindow(RotatedRect(Point2f(object.getXPos(),object.getYPos()),Size2d(sqrt(object.getArea()),sqrt(object.getArea())),0));
        displayBlobMarker(object, output);
        //set ID
        bool found = false;
        for(auto old : oldObjects) {
            double old_ray = sqrt(old.getArea() / PI);
            double distance = sqrt(abs(object.getXPos() - old.getXPos()) + abs(object.getYPos() - old.getYPos()));
            object.setDistance(distance);
            //if(distance < old_ray && !old.getAlreadyFound()){
            if(object.getXPos() < old.getXPos() + old_ray && object.getXPos() > old.getXPos() - old_ray
               && object.getYPos() < old.getYPos() + old_ray && object.getYPos() > old.getYPos() - old_ray
               && !old.getAlreadyFound() && old.getColor() == object.getColor()){
                found = true;
                object.setID(old.getID());
                old.setAlreadyFound(true);
            }
        }
        if(!found) {
            object.setID(++maxID);
        }
        blobObjects.emplace_back(RobotDesc(object));
    }
    for(auto object : redObjects){
        object.setColor(RED);
        object.setWindow(RotatedRect(Point2f(object.getXPos(),object.getYPos()),Size2d(sqrt(object.getArea()),sqrt(object.getArea())),0));
        displayBlobMarker(object, output);
        //set ID
        bool found = false;
        for(auto old : oldObjects) {
            double old_ray = sqrt(old.getArea() / PI);
            double distance = sqrt(abs(object.getXPos() - old.getXPos()) + abs(object.getYPos() - old.getYPos()));
            object.setDistance(distance);
            //if(distance < old_ray && !old.getAlreadyFound()){
            if(object.getXPos() < old.getXPos() + old_ray && object.getXPos() > old.getXPos() - old_ray
             && object.getYPos() < old.getYPos() + old_ray && object.getYPos() > old.getYPos() - old_ray
                    && !old.getAlreadyFound() && old.getColor() == object.getColor()){
                found = true;
                object.setID(old.getID());
                old.setAlreadyFound(true);
            }
        }
        if(!found) {
            object.setID(++maxID);
        }
        blobObjects.emplace_back(RobotDesc(object));
    }


    for (Color* color : colors)
        delete color;

    //conversion of the input
    cvtColor(input, output, CV_BGR2GRAY);
}

//Robot detection algorithm
void RobotDetection::detectRobots(const cv::Mat &input, cv::Mat &output) {



    //reduce the noise to avoid false circle detection
    GaussianBlur(output, output,Size(9,9),2,2);


    //Apply the hough transform to find the circles
    //1st param: output Mat
    //2nd param: vector of circles found
    //3rd param: method, currently this is the only one that works
    //4th param: resolution ratio, 1 keeps the highest resolution possible
    //5th param: less it is, less the detection is accurate (suggestion 50 to 500), default 100
    //6th and 7th param: less it is, more we get false circle detection (suggestion 50 to 500), default 200 and 100
    //8th and 9th params: min and max circle radius , default 0 (don't care)
    HoughCircles(output, circles, CV_HOUGH_GRADIENT, 0.5, output.rows/8,200,100, 50); //TODO: add trackbars to set the parameters

    //Update foundCircles
    saveCircles();

    //Verify if there is some color in each circle, to confirm the detection of robots.
    robotsVerification(output);


}
//3rd sted of the robot detection algorithm
//Save the circles detected in foundCircles
//It allows to loose a circle for a few frames without loosing its information
void RobotDetection::saveCircles(){

    for(size_t i=0; i<circles.size();i++)
    {
        //save positions
        foundCircles.at(i).setXPos(circles[i][0]);
        foundCircles.at(i).setYPos(circles[i][1]);
        foundCircles.at(i).setRadius(circles[i][2]);

    }
}
//4th step of the robot detection algorithm
void RobotDetection::robotsVerification(cv::Mat &output) {

    //réinitialisation
    foundRobots.erase(foundRobots.begin(), foundRobots.end());//TODO: get rid of the reinitialisation, check for duplicates and if some disappeared
    foundObstacles.erase(foundObstacles.begin(), foundObstacles.end());

    //Verification robots or obstacles
    //If a robot is found, it is erased from its initial vector (redObjects or greenObjects).
    for(auto aCircle: foundCircles){
        bool isRed = false, isGreen = false, isWhite = false;
        for (auto it = redObjects.begin(); it<redObjects.end();it++) {
            auto redObject = *it;
            if (sqrt(pow(aCircle.getXPos() - redObject.getXPos(),2) + pow(aCircle.getYPos() - redObject.getYPos(),2)) < aCircle.getRadius()) {
                isRed = true;
                redObjects.erase(it);
            }
        }
        for (auto it = greenObjects.begin(); it<greenObjects.end();it++) {
            auto greenObject = *it;
            if (sqrt(pow(aCircle.getXPos() - greenObject.getXPos(),2) + pow(aCircle.getYPos() - greenObject.getYPos(),2)) < aCircle.getRadius()) {
                isGreen = true;
                greenObjects.erase(it);
            }
        }
        /*for (auto whiteObject: whiteObjects) {
            if (sqrt(pow(aCircle.getXPos() - whiteObject.getXPos(),2) + pow(aCircle.getYPos() - whiteObject.getYPos(),2)) < aCircle.getRadius() &&
                    whiteObject.getArea()>0.6*aCircle.getArea()) {
                isWhite = true;
            }
        }*/
        //Emplace robot detected in foundRobots
        if (isRed) {
            aCircle.setColor(RED);
            aCircle.setWindow(RotatedRect(Point2f(aCircle.getXPos(),aCircle.getYPos()),Size2f(aCircle.getRadius()*2,aCircle.getRadius()*2),0));
            foundRobots.emplace_back(aCircle);
            displayCircle(aCircle, output);
        }
        else if (isGreen) {
            aCircle.setColor(GREEN);
            aCircle.setWindow(RotatedRect(Point2f(aCircle.getXPos(),aCircle.getYPos()),Size2f(aCircle.getRadius()*2,aCircle.getRadius()*2),0));
            foundRobots.emplace_back(aCircle);
            displayCircle(aCircle, output);
        }
        /*else if (isWhite) {
            aCircle.setColor(WHITE);
            aCircle.setWindow(RotatedRect(Point2f(aCircle.getXPos(),aCircle.getYPos()),Size2f(aCircle.getRadius()*2,aCircle.getRadius()*2),0));//TODO: Will it work since the shape change slowly while moving?
            foundObstacles.emplace_back(aCircle);
            displayCircle(aCircle, output);
        }*/
    }

    //Push the rejected colored blobs in the robotsNotConfirmed vector and display little circle on the output
    for (auto redObject: redObjects) {
        redObject.setColor(RED);
        robotsNotConfirmed.emplace_back(redObject);
        displayBlobMarker(redObject, output);
    }
    for (auto greenObject: greenObjects) {
        greenObject.setColor(GREEN);
        robotsNotConfirmed.emplace_back(greenObject);
        displayBlobMarker(greenObject, output);
    }
}

//Getters
vector<RobotDesc>& RobotDetection::getFoundRobots(){
    return foundRobots;
}
vector<RobotDesc> RobotDetection::getFoundObstacles(){
    return foundObstacles;
}
vector<RobotDesc> RobotDetection::getRobotsNotConfirmed(){
    return robotsNotConfirmed;
}

/*
 * Display methods (for tests usage only)
 */
void RobotDetection::displayCircle(const RobotDesc& aCircle, cv::Mat &output){
    //index display
    //0: white object, 1:red object, 2 green object
    char index[1];
    sprintf(index, "%d", aCircle.getColor());
    putText(output, index, cvPoint(aCircle.getXPos(),aCircle.getYPos()), FONT_HERSHEY_PLAIN,3, Scalar(0,0,0),4);

    //circle center
    Point center(cvRound(aCircle.getXPos()), cvRound(aCircle.getYPos()));
    //radius
    int radius = cvRound(aCircle.getRadius());
    //circle center
    circle(output, center, 3, Scalar(0,255,0), -1,8,0);
    //circle outline
    circle(output, center, radius, Scalar(0,0,255),3,8,0);
}
//Blob marker for robotsNotConfirmed
void RobotDetection::displayBlobMarker(const RobotDesc& object, cv::Mat &output){
    cv::circle(output, cv::Point(object.getXPos(), object.getYPos()), 10, cv::Scalar(0, 0, 255));
    cv::putText(output, intToString(object.getXPos()) + " , " + intToString(object.getYPos()),
                cv::Point(object.getXPos(), object.getYPos() + 20), 1, 1, Scalar(0, 255, 0));
}
string RobotDetection::intToString(int number) {
    std::stringstream ss;
    ss << number;
    return ss.str();
}

vector<RobotDesc> RobotDetection::getBlobObjects(){
    return blobObjects;
}
vector<RobotDesc> getGreenObjects();

//Trackbars methods
/*
void on_trackbar(int, void *) {//This function gets called whenever a
    // trackbar position is changed
}

void RobotDetection::createTrackbars() {
    //create window for trackbars
    namedWindow(trackbarWindowName, 0);
    namedWindow(HSVTrackbars, 0);
    namedWindow(shapeDetectTrackbars, 0);
    //create trackbars and insert them into window
    //3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
    //the max value the trackbar can move (eg. H_HIGH),
    //and the function that is called whenever the trackbar is moved(eg. on_trackbar)
    //                                  ---->    ---->     ---->
    createTrackbar("PRE BLUR", trackbarWindowName, &PRE_BLUR, 50, on_trackbar);
    createTrackbar("PRE EROSIONS W", trackbarWindowName, &PRE_EROSIONS_W, 256, on_trackbar);
    createTrackbar("DILATIONS_W W", trackbarWindowName, &DILATIONS_W, 256, on_trackbar);
    createTrackbar("POST EROSIONS W", trackbarWindowName, &POST_EROSIONS_W, 50, on_trackbar);
    createTrackbar("PRE EROSIONS G", trackbarWindowName, &PRE_EROSIONS_G, 256, on_trackbar);
    createTrackbar("DILATIONS_W G", trackbarWindowName, &DILATIONS_G, 256, on_trackbar);
    createTrackbar("POST EROSIONS G", trackbarWindowName, &POST_EROSIONS_G, 50, on_trackbar);
    createTrackbar("H_MIN W", HSVTrackbars, &H_MIN_W, 256, on_trackbar);
    createTrackbar("H_MAX W", HSVTrackbars, &H_MAX_W, 256, on_trackbar);
    createTrackbar("S_MIN W", HSVTrackbars, &S_MIN_W, 256, on_trackbar);
    createTrackbar("S_MAX W", HSVTrackbars, &S_MAX_W, 256, on_trackbar);
    createTrackbar("V_MIN W", HSVTrackbars, &V_MIN_W, 256, on_trackbar);
    createTrackbar("V_MAX W", HSVTrackbars, &V_MAX_W, 256, on_trackbar);
    createTrackbar("H_MIN G", HSVTrackbars, &H_MIN_G, 256, on_trackbar);
    createTrackbar("H_MAX G", HSVTrackbars, &H_MAX_G, 256, on_trackbar);
    createTrackbar("S_MIN G", HSVTrackbars, &S_MIN_G, 256, on_trackbar);
    createTrackbar("S_MAX G", HSVTrackbars, &S_MAX_G, 256, on_trackbar);
    createTrackbar("V_MIN G", HSVTrackbars, &V_MIN_G, 256, on_trackbar);
    createTrackbar("V_MAX G", HSVTrackbars, &V_MAX_G, 256, on_trackbar);
    createTrackbar("H_MIN R", HSVTrackbars, &H_MIN_R, 256, on_trackbar);
    createTrackbar("H_MAX R", HSVTrackbars, &H_MAX_R, 256, on_trackbar);
    createTrackbar("S_MIN R", HSVTrackbars, &S_MIN_R, 256, on_trackbar);
    createTrackbar("S_MAX R", HSVTrackbars, &S_MAX_R, 256, on_trackbar);
    createTrackbar("V_MIN R", HSVTrackbars, &V_MIN_R, 256, on_trackbar);
    createTrackbar("V_MAX R", HSVTrackbars, &V_MAX_R, 256, on_trackbar);

    // Create shape detector trackbars
    createTrackbar("Canny Thresh 1", shapeDetectTrackbars, &CANNY_THRESH1, 256, on_trackbar);
    createTrackbar("Canny Thresh 2", shapeDetectTrackbars, &CANNY_THRESH2, 256, on_trackbar);
    createTrackbar("Canny Aperture", shapeDetectTrackbars, &CANNY_APERTURE, 5, on_trackbar);
    createTrackbar("Poly Area Min", shapeDetectTrackbars, &POLY_AREA_MIN, 50000, on_trackbar);
    createTrackbar("Poly Area Max", shapeDetectTrackbars, &POLY_AREA_MAX, 50000, on_trackbar);
    createTrackbar("Operator:\n 0: Opening - 1: Closing \n 2: Gradient - 3: Top Hat \n 4: Black Hat",
                   shapeDetectTrackbars, &MORPH_OP, 4, on_trackbar);
    createTrackbar("Element:\n 0: Rect - 1: Cross - 2: Ellipse", shapeDetectTrackbars, &MORPH_ELEMENT, 2,
                   on_trackbar);
    createTrackbar("Kernel size:\n 2n +1", shapeDetectTrackbars, &MORPH_SIZE, 21, on_trackbar);
    createTrackbar("Max distance", shapeDetectTrackbars, &MAX_DIST, 50000, on_trackbar);

}*/