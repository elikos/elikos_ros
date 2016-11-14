//
// Created by ta11e4rand on 10/02/16.
//
#include "TargetDetection.h"

TargetDetection::TargetDetection()
{
}
/*
 * This method is the first step of the tracking algorithm
 * params: input: frame from the camera, output_*: output matrix with color, output: general output matrix
 */
void TargetDetection::detect(const cv::Mat &input, cv::Mat &output_w, cv::Mat &output_r, cv::Mat &output_g, cv::Mat &output)
{

    //Color detection
    detectColor(input, output_w, output_r, output_g, output);

    emplaceNewRobots(output);
}

void TargetDetection::updateHSV(int color, int h, int s, int v, int delta)
{
    switch (color)
    {
    case 0: //RED
        *(detection.getRed()->H_MIN) = (int)(h - 0.15 * delta);
        *(detection.getRed()->H_MAX) = (int)(h + 0.15 * delta);

        *(detection.getRed()->S_MIN) = (int)(s - 1.1 * delta);
        *(detection.getRed()->S_MAX) = (int)(s + 1.1 * delta);

        *(detection.getRed()->V_MIN) = v - 2 * delta;
        *(detection.getRed()->V_MAX) = v + 2 * delta;
        break;
    case 1: //GREEN
        *(detection.getGreen()->H_MIN) = (int)(h - 0.15 * delta);
        *(detection.getGreen()->H_MAX) = (int)(h + 0.15 * delta);

        *(detection.getGreen()->S_MIN) = (int)(s - 1.1 * delta);
        *(detection.getGreen()->S_MAX) = (int)(s + 1.1 * delta);

        *(detection.getGreen()->V_MIN) = v - 2 * delta;
        *(detection.getGreen()->V_MAX) = v + 2 * delta;
        break;
    case 2: //WHITE
        *(detection.getWhite()->H_MIN) = (int)(h - 0.15 * delta);
        *(detection.getWhite()->H_MAX) = (int)(h + 0.15 * delta);

        *(detection.getWhite()->S_MIN) = (int)(s - 1.1 * delta);
        *(detection.getWhite()->S_MAX) = (int)(s + 1.1 * delta);

        *(detection.getWhite()->V_MIN) = v - 2 * delta;
        *(detection.getWhite()->V_MAX) = v + 2 * delta;
        break;
    }
}

void TargetDetection::updateHSV(int color, int h_max, int h_min, int s_max, int s_min, int v_max, int v_min)
{
    switch (color)
    {
    case 0: //RED
        *(detection.getRed()->H_MIN) = h_min;
        *(detection.getRed()->H_MAX) = h_max;

        *(detection.getRed()->S_MIN) = s_min;
        *(detection.getRed()->S_MAX) = s_max;

        *(detection.getRed()->V_MIN) = v_min;
        *(detection.getRed()->V_MAX) = v_max;
        break;
    case 1: //GREEN
        *(detection.getGreen()->H_MIN) = h_min;
        *(detection.getGreen()->H_MAX) = h_max;
        *(detection.getGreen()->S_MIN) = s_min;
        *(detection.getGreen()->S_MAX) = s_max;
        *(detection.getGreen()->V_MIN) = v_min;
        *(detection.getGreen()->V_MAX) = v_max;
        break;
    case 2: //WHITE
        *(detection.getWhite()->H_MIN) = h_min;
        *(detection.getWhite()->H_MAX) = h_max;
        *(detection.getWhite()->S_MIN) = s_min;
        *(detection.getWhite()->S_MAX) = s_max;
        *(detection.getWhite()->V_MIN) = v_min;
        *(detection.getWhite()->V_MAX) = v_max;
        break;
    }
}

void TargetDetection::emplaceNewRobots(cv::Mat &output)
{
    //Reset robot container
    robots.erase(robots.begin(), robots.end());

    //Fill container
    for (auto object : detection.getBlobObjects())
    {
        displayID(object, output);
        robots.emplace_back(object);
    }
}

void TargetDetection::detectColor(const cv::Mat &input, cv::Mat &output_w, cv::Mat &output_r, cv::Mat &output_g, cv::Mat &output)
{
    detection.detectColor(input, output_w, output_r, output_g, output);
}

vector<RobotDesc> TargetDetection::getRobots()
{
    return robots;
}
//Method to display the direction arrow (for tests usage only)
void TargetDetection::displayID(RobotDesc robot, cv::Mat &output)
{
    //center
    Point center(cvRound(robot.getXPos()), cvRound(robot.getYPos()));
    //circle
    if (robot.getColor() == ColorsIndex::RED)
    {
        circle(output, center, 5, Scalar(0, 0, 255));
    }
    else if (robot.getColor() == ColorsIndex::GREEN)
    {
        circle(output, center, 5, Scalar(0, 255, 0));
    }
    string text = to_string(robot.getID());
    int fontFace = FONT_HERSHEY_SCRIPT_SIMPLEX;
    double fontScale = 2;
    int thickness = 3;
    cv::Point pos(robot.getXPos(), robot.getYPos());
    cv::putText(output, text, pos, fontFace, fontScale, Scalar(70, 70, 70), thickness, 8);
}
