#include "Color.h"
#include "GpuCV.h"
#include "CpuCV.h"

Color::Color() {
    // int nDevices = gpu::getCudaEnabledDeviceCount();
    // cvWrapper = (nDevices > 0 ? (CVWrapperInterface*) new GpuCV() :
    //                             (CVWrapperInterface*) new CpuCV());

    cvWrapper = new CpuCV();
}

Color::~Color() {
    delete cvWrapper;
}

Mat& Color::getThreshold()
{
    return threshold;
}

string Color::intToString(int number) {
    std::stringstream ss;
    ss << number;
    return ss.str();
}

Mat Color::generateThreshold(const Mat& image)
{
    //cvWrapper->
    cvtColor(image, hsv, COLOR_BGR2HSV);
    BLUR_AMOUNT = PRE_BLUR + 1;
    blur(hsv, hsv, Size(BLUR_AMOUNT, BLUR_AMOUNT), Point(-1, -1));



    //TODO optimize
    if(*H_MIN > *H_MAX){
        Mat1b maxCol, minCol;
        inRange(hsv, Scalar(0, *S_MIN, *V_MIN), Scalar(*H_MAX, *S_MAX, *V_MAX), maxCol);
        inRange(hsv, Scalar(*H_MIN, *S_MIN, *V_MIN), Scalar(180, *S_MAX, *V_MAX), minCol);
        threshold = maxCol | minCol;
    }else{
        // No inRange implementation for gpu module.
        // This should not cause performance problems on the jetson.

        inRange(hsv, Scalar(*H_MIN, *S_MIN, *V_MIN), Scalar(*H_MAX, *S_MAX, *V_MAX), threshold);
    }
    // Consolidate the colored parts into one big blob to delimit the robot
    erode(threshold, threshold, getStructuringElement(MORPH_ELLIPSE, Size(3, 3)), Point(-1, -1),
          *PRE_EROSIONS);
    dilate(threshold, threshold, getStructuringElement(MORPH_ELLIPSE, Size(3, 3)), Point(-1, -1), *DILATIONS);
    erode(threshold, threshold, getStructuringElement(MORPH_ELLIPSE, Size(3, 3)), Point(-1, -1),
          *POST_EROSIONS);

    return threshold;
}

void Color::trackFilteredObjects(Mat &cameraFeed) {

    //int x,y;
    RobotDesc myRobot;
    vector<RobotDesc> vecRobot;
    Mat temp;
    threshold.copyTo(temp);
    //these two vectors needed for output of findContours
    vector<vector<Point> > contours;

    vector<Vec4i> hierarchy;
    //find contours of filtered image using openCV findContours function
    findContours(temp, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
    //use moments method to find our filtered object
    double refArea = 0;
    //Reset objects container
    foundObjects.clear();

    if (hierarchy.size() > 0) {
        int numObjects = hierarchy.size();
        //if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
        if (numObjects < MAX_NUM_OBJECTS) {


            for (int index = 0; index >= 0; index = hierarchy[index][0]) {

                Moments moment = moments((cv::Mat) contours[index]);
                double area = moment.m00;

                //if the area is less than 20 px by 20px then it is probably just noise
                //if the area is the same as the 3/2 of the image size, probably just a bad filter
                //we only want the object with the largest area so we safe a reference area each
                //iteration and compare it to the area in the next iteration.
                //if (area > MIN_OBJECT_AREA) {

                    myRobot.setXPos(moment.m10 / area);
                    myRobot.setYPos(moment.m01 / area);
                    myRobot.setArea(area);

                    foundObjects.emplace_back(myRobot);

                //}
            }
        }
        else putText(cameraFeed, "TOO MUCH NOISE! ADJUST FILTER", Point(0, 50), 1, 2, Scalar(0, 0, 255), 2);
    }
}

void Color::printFoundObjects()
{
    for (int i=0; i<foundObjects.size(); i++){
        printf("Found Color Object[%i]:\n", i);
        printf("xPos: %i\n", foundObjects[i].getXPos());
        printf("yPos: %i\n", foundObjects[i].getYPos());
    }
}

void Color::drawFoundObjects(Mat& image)
{
    for (int i = 0; i < foundObjects.size(); i++) {
        cv::circle(image, cv::Point(foundObjects.at(i).getHPos(), foundObjects.at(i).getVPos()), 10, cv::Scalar(0, 0, 255));
        cv::putText(image, intToString(foundObjects.at(i).getHPos()) + " , " + intToString(foundObjects.at(i).getVPos()),
                    cv::Point(foundObjects.at(i).getHPos(), foundObjects.at(i).getVPos() + 20), 1, 1, Scalar(0, 255, 0));
    }
}

vector<RobotDesc> Color::getObjects(){
    return foundObjects;
}