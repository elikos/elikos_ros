#include "BlobDetection.h"
#include <opencv2/core/core.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

BlobDetection::BlobDetection()
{
    //foundCircles initialisation
    //Maximum of 20 circles detected at the same time
    //It allows to loose a circle for a few frames without loosing its information
    for (int j = 0; j < 20; j++)
    {
        foundCircles.emplace_back(RobotDesc(j, 0, 0));
    }
    // Init image
    trackbarsWhiteMat_ = Mat3b(1, 300, Vec3b(0, 0, 0));
    trackbarsRedMat_ = Mat3b(1, 300, Vec3b(0, 0, 0));
    trackbarsGreenMat_ = Mat3b(1, 300, Vec3b(0, 0, 0));
    maxID = 0;
}

//Color detection algorithm
void BlobDetection::detectColor(const cv::Mat &input, cv::Mat &output_w, cv::Mat &output_r, cv::Mat &output_g, cv::Mat &output)
{
    //Check if the input is empty
    if (!input.data)
        cerr << "Input of detecRobots is empty"; //TODO:throw an exception

    //Initialisation
    auto oldObjects = blobObjects;
    blobObjects.erase(blobObjects.begin(), blobObjects.end());

    currentImage = input;

    //Blob detection with color thresholds
    output_w = whiteColor_.generateThreshold(currentImage);
    whiteColor_.trackFilteredObjects(currentImage);
    output_r = redColor_.generateThreshold(currentImage);
    redColor_.trackFilteredObjects(currentImage);
    output_g = greenColor_.generateThreshold(currentImage);
    greenColor_.trackFilteredObjects(currentImage);

    //extraction of white, red and green objects (robots)
    whiteObjects = whiteColor_.getObjects();
    greenObjects = greenColor_.getObjects();
    redObjects = redColor_.getObjects();

    for (auto object : greenObjects)
    {
        object.setColor(GREEN);
        object.setWindow(RotatedRect(Point2f(object.getXPos(), object.getYPos()), Size2f(sqrt(object.getArea()), sqrt(object.getArea())), 0));
        //set ID
        bool found = false;
        for (auto old : oldObjects)
        {
            double old_ray = sqrt(old.getArea() / PI);
            double distance = sqrt(abs(object.getXPos() - old.getXPos()) + abs(object.getYPos() - old.getYPos()));
            object.setDistance(distance);
            //if(distance < old_ray && !old.getAlreadyFound()){
            if (object.getXPos() < old.getXPos() + old_ray && object.getXPos() > old.getXPos() - old_ray && object.getYPos() < old.getYPos() + old_ray && object.getYPos() > old.getYPos() - old_ray && !old.getAlreadyFound() && old.getColor() == object.getColor())
            {
                found = true;
                object.setID(old.getID());
                old.setAlreadyFound(true);
            }
        }
        if (!found)
        {
            object.setID(++maxID);
        }
        blobObjects.emplace_back(RobotDesc(object));
    }
    for (auto object : redObjects)
    {
        object.setColor(RED);
        object.setWindow(RotatedRect(Point2f(object.getXPos(), object.getYPos()), cv::Size2f(sqrt(object.getArea()), sqrt(object.getArea())), 0));
        //set ID
        bool found = false;
        for (auto old : oldObjects)
        {
            double old_ray = sqrt(old.getArea() / PI);
            double distance = sqrt(abs(object.getXPos() - old.getXPos()) + abs(object.getYPos() - old.getYPos()));
            object.setDistance(distance);
            //if(distance < old_ray && !old.getAlreadyFound()){
            if (object.getXPos() < old.getXPos() + old_ray && object.getXPos() > old.getXPos() - old_ray && object.getYPos() < old.getYPos() + old_ray && object.getYPos() > old.getYPos() - old_ray && !old.getAlreadyFound() && old.getColor() == object.getColor())
            {
                found = true;
                object.setID(old.getID());
                old.setAlreadyFound(true);
            }
        }
        if (!found)
        {
            object.setID(++maxID);
        }
        blobObjects.emplace_back(RobotDesc(object));
    }

    //conversion of the input
    output = input;
}

//Circle detection algorithm
void BlobDetection::detectCircles(const cv::Mat &input, cv::Mat &output_w, cv::Mat &output_r, cv::Mat &output_g, cv::Mat &output)
{
    //Check if the input is empty
    if (!input.data)
        cerr << "Input of detecRobots is empty"; //TODO:throw an exception

    //Initialisation
    auto oldObjects = blobObjects;
    blobObjects.erase(blobObjects.begin(), blobObjects.end());

    cv::Mat src_gray;
    cv::cvtColor(output, src_gray, CV_BGR2GRAY);

    vector<Vec3f> circles;

    // /mavros/px4flow/ground_distance
    /// Apply the Hough Transform to find the circles
    cv::HoughCircles(src_gray, circles, CV_HOUGH_GRADIENT, 1, src_gray.rows / 8, 200, 70, 0, 0);

    /// Draw the circles detected
    for (size_t i = 0; i < circles.size(); i++)
    {
        cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        // circle center
        cv::circle(output, center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
        // circle outline
        cv::circle(output, center, radius, cv::Scalar(0, 0, 255), 3, 8, 0);
    }
}

vector<RobotDesc> BlobDetection::getBlobObjects()
{
    return blobObjects;
}

void BlobDetection::createTrackbars()
{
    //create windows for trackbars
    string WhiteTrackbars = "White Calibration Trackbars";
    string RedTrackbars = "Red Calibration Trackbars";
    string GreenTrackbars = "Green Calibration Trackbars";

    namedWindow(WhiteTrackbars, 0);
    namedWindow(RedTrackbars, 1);
    namedWindow(GreenTrackbars, 2);

    startWindowThread();

    whiteColor_.createTrackbars(WhiteTrackbars);
    redColor_.createTrackbars(RedTrackbars);
    greenColor_.createTrackbars(GreenTrackbars);

    imshow(WhiteTrackbars, trackbarsWhiteMat_);
    imshow(RedTrackbars, trackbarsRedMat_);
    imshow(GreenTrackbars, trackbarsGreenMat_);
}
void BlobDetection::saveCalibration(string filename)
{
    std::fstream fs;
    fs.open(filename, std::fstream::out | std::fstream::trunc);
    if (!fs.fail())
    {
        fs << *whiteColor_.H_MIN << endl;
        fs << *whiteColor_.H_MAX << endl;
        fs << *whiteColor_.S_MIN << endl;
        fs << *whiteColor_.S_MAX << endl;
        fs << *whiteColor_.V_MIN << endl;
        fs << *whiteColor_.V_MAX << endl;
        fs << *whiteColor_.PRE_EROSIONS << endl;
        fs << *whiteColor_.DILATIONS << endl;
        fs << *whiteColor_.POST_EROSIONS << endl;

        fs << *redColor_.H_MIN << endl;
        fs << *redColor_.H_MAX << endl;
        fs << *redColor_.S_MIN << endl;
        fs << *redColor_.S_MAX << endl;
        fs << *redColor_.V_MIN << endl;
        fs << *redColor_.V_MAX << endl;
        fs << *redColor_.PRE_EROSIONS << endl;
        fs << *redColor_.DILATIONS << endl;
        fs << *redColor_.POST_EROSIONS << endl;

        fs << *greenColor_.H_MIN << endl;
        fs << *greenColor_.H_MAX << endl;
        fs << *greenColor_.S_MIN << endl;
        fs << *greenColor_.S_MAX << endl;
        fs << *greenColor_.V_MIN << endl;
        fs << *greenColor_.V_MAX << endl;
        fs << *greenColor_.PRE_EROSIONS << endl;
        fs << *greenColor_.DILATIONS << endl;
        fs << *greenColor_.POST_EROSIONS << endl;

        fs.close();
    }
}
void BlobDetection::loadCalibration(string filename)
{
    std::fstream fs;
    fs.open(filename, std::fstream::in);

    if (!fs.fail())
    {
        fs >> *whiteColor_.H_MIN;
        fs >> *whiteColor_.H_MAX;
        fs >> *whiteColor_.S_MIN;
        fs >> *whiteColor_.S_MAX;
        fs >> *whiteColor_.V_MIN;
        fs >> *whiteColor_.V_MAX;
        fs >> *whiteColor_.PRE_EROSIONS;
        fs >> *whiteColor_.DILATIONS;
        fs >> *whiteColor_.POST_EROSIONS;

        fs >> *redColor_.H_MIN;
        fs >> *redColor_.H_MAX;
        fs >> *redColor_.S_MIN;
        fs >> *redColor_.S_MAX;
        fs >> *redColor_.V_MIN;
        fs >> *redColor_.V_MAX;
        fs >> *redColor_.PRE_EROSIONS;
        fs >> *redColor_.DILATIONS;
        fs >> *redColor_.POST_EROSIONS;

        fs >> *greenColor_.H_MIN;
        fs >> *greenColor_.H_MAX;
        fs >> *greenColor_.S_MIN;
        fs >> *greenColor_.S_MAX;
        fs >> *greenColor_.V_MIN;
        fs >> *greenColor_.V_MAX;
        fs >> *greenColor_.PRE_EROSIONS;
        fs >> *greenColor_.DILATIONS;
        fs >> *greenColor_.POST_EROSIONS;

        fs.close();
    }
}

std::string BlobDetection::getAllParams()
{
    std::stringstream ss;

    ss << *redColor_.H_MAX << "\t"
       << *redColor_.H_MIN << "\t"
       << *redColor_.S_MAX << "\t"
       << *redColor_.S_MIN << "\t"
       << *redColor_.V_MAX << "\t"
       << *redColor_.V_MIN << "\t"
       << *redColor_.PRE_EROSIONS << "\t"
       << *redColor_.DILATIONS << "\t"
       << *redColor_.POST_EROSIONS << "\t"

       << *greenColor_.H_MAX << "\t"
       << *greenColor_.H_MIN << "\t"
       << *greenColor_.S_MAX << "\t"
       << *greenColor_.S_MIN << "\t"
       << *greenColor_.V_MAX << "\t"
       << *greenColor_.V_MIN << "\t"
       << *greenColor_.PRE_EROSIONS << "\t"
       << *greenColor_.DILATIONS << "\t"
       << *greenColor_.POST_EROSIONS << "\t"

       << *whiteColor_.H_MAX << "\t"
       << *whiteColor_.H_MIN << "\t"
       << *whiteColor_.S_MAX << "\t"
       << *whiteColor_.S_MIN << "\t"
       << *whiteColor_.V_MAX << "\t"
       << *whiteColor_.V_MIN << "\t"
       << *whiteColor_.PRE_EROSIONS << "\t"
       << *whiteColor_.DILATIONS << "\t"
       << *whiteColor_.POST_EROSIONS;

    return ss.str();
}
