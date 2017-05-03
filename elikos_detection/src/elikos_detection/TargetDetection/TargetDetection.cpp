//
// Created by ta11e4rand on 10/02/16.
//
#include "TargetDetection.h"
#include <functional>

TargetDetection::TargetDetection() {}
/*
 * This method is the first step of the tracking algorithm
 * params: input: frame from the camera, output_*: output matrix with color,
 * output: general output matrix
 */
void TargetDetection::detect(const cv::Mat& input, cv::Mat& output_w,
                             cv::Mat& output_r, cv::Mat& output_g,
                             cv::Mat& output,
                             std::vector<RobotDesc>& outputRobotsArray) {
    // Color detection
    blobDetection_.detect(input, output_w, output_r, output_g, output, outputRobotsArray);

    // Circle detection (TODO: Move to ShapeDetection)
   // blobDetection_.detectCircles(input, output_w, output_r, output_g, output, outputRobotsArray);

    filterDuplicated(outputRobotsArray);
    // Shape detection
    /* TODO */

    displayRobots(outputRobotsArray, output);
}

void TargetDetection::updateHSV(int color, int h, int s, int v, int delta) {
    switch (color) {
        case 0:  // RED
            *(blobDetection_.getRed()->H_MIN) = (int)(h - 0.15 * delta);
            *(blobDetection_.getRed()->H_MAX) = (int)(h + 0.15 * delta);

            *(blobDetection_.getRed()->S_MIN) = (int)(s - 1.1 * delta);
            *(blobDetection_.getRed()->S_MAX) = (int)(s + 1.1 * delta);

            *(blobDetection_.getRed()->V_MIN) = v - 2 * delta;
            *(blobDetection_.getRed()->V_MAX) = v + 2 * delta;
            break;
        case 1:  // GREEN
            *(blobDetection_.getGreen()->H_MIN) = (int)(h - 0.15 * delta);
            *(blobDetection_.getGreen()->H_MAX) = (int)(h + 0.15 * delta);

            *(blobDetection_.getGreen()->S_MIN) = (int)(s - 1.1 * delta);
            *(blobDetection_.getGreen()->S_MAX) = (int)(s + 1.1 * delta);

            *(blobDetection_.getGreen()->V_MIN) = v - 2 * delta;
            *(blobDetection_.getGreen()->V_MAX) = v + 2 * delta;
            break;
        case 2:  // WHITE
            *(blobDetection_.getWhite()->H_MIN) = (int)(h - 0.15 * delta);
            *(blobDetection_.getWhite()->H_MAX) = (int)(h + 0.15 * delta);

            *(blobDetection_.getWhite()->S_MIN) = (int)(s - 1.1 * delta);
            *(blobDetection_.getWhite()->S_MAX) = (int)(s + 1.1 * delta);

            *(blobDetection_.getWhite()->V_MIN) = v - 2 * delta;
            *(blobDetection_.getWhite()->V_MAX) = v + 2 * delta;
            break;
    }
}


void TargetDetection::getRemoteParams(int color, int& h_max, int& h_min, int& s_max, int& s_min, int& v_max, int& v_min,int& preErode, int& dilate, int& postErode)
{
    std::function<Color*()> fun;
    switch (color)
    {
        case 0: //RED
            fun = [this](){return this->blobDetection_.getRed();};
            break;
        case 1: //GREEN
            fun = [this](){return this->blobDetection_.getGreen();};
            break;
        case 2: //WHITE
            fun = [this](){return this->blobDetection_.getWhite();};
            break;
    }
    h_min = *(fun()->H_MIN);
    h_max = *(fun()->H_MAX);
    s_min = *(fun()->S_MIN);
    s_max = *(fun()->S_MAX);
    v_min = *(fun()->V_MIN);
    v_max = *(fun()->V_MAX);
    preErode = *(fun()->PRE_EROSIONS);
    dilate = *(fun()->DILATIONS);
    postErode = *(fun()->POST_EROSIONS);
}

void TargetDetection::fetchRemoteParams(int color, int h_max, int h_min,
                                        int s_max, int s_min, int v_max,
                                        int v_min, int preErode, int dilate,
                                        int postErode) {
    switch (color) {
        case 0:  // RED
            *(blobDetection_.getRed()->H_MIN) = h_min;
            *(blobDetection_.getRed()->H_MAX) = h_max;

            *(blobDetection_.getRed()->S_MIN) = s_min;
            *(blobDetection_.getRed()->S_MAX) = s_max;

            *(blobDetection_.getRed()->V_MIN) = v_min;
            *(blobDetection_.getRed()->V_MAX) = v_max;

            *(blobDetection_.getRed()->PRE_EROSIONS) = preErode;
            *(blobDetection_.getRed()->DILATIONS) = dilate;
            *(blobDetection_.getRed()->POST_EROSIONS) = postErode;
            break;
        case 1:  // GREEN
            *(blobDetection_.getGreen()->H_MIN) = h_min;
            *(blobDetection_.getGreen()->H_MAX) = h_max;

            *(blobDetection_.getGreen()->S_MIN) = s_min;
            *(blobDetection_.getGreen()->S_MAX) = s_max;

            *(blobDetection_.getGreen()->V_MIN) = v_min;
            *(blobDetection_.getGreen()->V_MAX) = v_max;

            *(blobDetection_.getGreen()->PRE_EROSIONS) = preErode;
            *(blobDetection_.getGreen()->DILATIONS) = dilate;
            *(blobDetection_.getGreen()->POST_EROSIONS) = postErode;
            break;
        case 2:  // WHITE
            *(blobDetection_.getWhite()->H_MIN) = h_min;
            *(blobDetection_.getWhite()->H_MAX) = h_max;

            *(blobDetection_.getWhite()->S_MIN) = s_min;
            *(blobDetection_.getWhite()->S_MAX) = s_max;

            *(blobDetection_.getWhite()->V_MIN) = v_min;
            *(blobDetection_.getWhite()->V_MAX) = v_max;

            *(blobDetection_.getWhite()->PRE_EROSIONS) = preErode;
            *(blobDetection_.getWhite()->DILATIONS) = dilate;
            *(blobDetection_.getWhite()->POST_EROSIONS) = postErode;
            break;
    }
}

void TargetDetection::displayRobots(const std::vector<RobotDesc>& robotsArray,
                                    cv::Mat& output) const {
    for (auto object : robotsArray) {
        displayID(object, output);
    }
}

// Method to display the direction arrow (for tests usage only)
void TargetDetection::displayID(const RobotDesc& robot, cv::Mat& output) const {
    // center
    Point center(cvRound(robot.getXPos()), cvRound(robot.getYPos()));
    // circle
    if (robot.getColor() == ColorsIndex::RED) {
        circle(output, center, 5, Scalar(0, 0, 255));
    } else if (robot.getColor() == ColorsIndex::GREEN) {
        circle(output, center, 5, Scalar(0, 255, 0));
    }
    string text = to_string(robot.getID());
    int fontFace = FONT_HERSHEY_SCRIPT_SIMPLEX;
    double fontScale = 2;
    int thickness = 3;
    cv::Point pos(robot.getXPos(), robot.getYPos());
    cv::putText(output, text, pos, fontFace, fontScale, Scalar(70, 70, 70),
                thickness, 8);
}

void TargetDetection::filterDuplicated(
    std::vector<RobotDesc>& arrayToFilter) const {
    arrayToFilter.erase(std::unique(arrayToFilter.begin(), arrayToFilter.end()),
                        arrayToFilter.end());
}
