//
// Created by ta11e4rand on 10/02/16.
//
#include "TargetDetection.h"
#include <functional>

void doDetect(const cv::Mat& input, const cv::Mat& targetimg, cv::Mat& output) {
    if (input.empty() || targetimg.empty()) {
        cout << "Can't read images... :(" << endl;
        return;
    }

    cv::Mat srcGray, targetGray;
    cv::cvtColor(input, srcGray, cv::COLOR_BGR2GRAY);
    cv::cvtColor(targetimg, targetGray, cv::COLOR_RGB2GRAY);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));
    cv::Mat srcThresh, targetThresh;
    cv::threshold(srcGray, srcThresh, 254, 255, CV_THRESH_BINARY);
    cv::threshold(targetGray, targetThresh, 254, 255, CV_THRESH_BINARY);

    //   vector<cv::Vec3f> circles;
    // cv::HoughCircles(srcThresh, circles, CV_HOUGH_GRADIENT, 1, 20, 200, 15,
    // 0, 0);

    // for (sizeDetection_;_t i = 0; i < circles.size(); ++i) {
    //   cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
    //   int radius = cvRound(circles[i][2]);
    //   // draw the circle center
    //   cv::circle(srcImage, center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
    //   // draw the circle outline
    //   cv::circle(srcImage, center, radius, cv::Scalar(0, 0, 255), 3, 8, 0);
    // }

    /////////////////////////////////////////////////////////////////////
    cv::Canny(srcGray, output, 254, 670, 3);  // 670 = CANNY_THRESH
}

double getDiff(double matA[7], double matB[7]) {
    double totalDiff = 0;
    double medDiff;
    double maxDiff = 0;

    for (size_t i = 0; i < 7; ++i) {
        double diff = std::abs(matA[i] - matB[i]);
        totalDiff += diff;

        if (diff > maxDiff) maxDiff = diff;
    }

    medDiff = totalDiff / 7;

    return maxDiff;
}

void doShapeReco(const cv::Mat& input, const cv::Mat& targetimg,
                 cv::Mat& output, const double& errorRange) {
    output = input;
    cv::Mat srcGray, targetGray;
    cv::cvtColor(input, srcGray, cv::COLOR_BGR2GRAY);
    cv::cvtColor(targetimg, targetGray, cv::COLOR_RGB2GRAY);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));
    cv::Mat srcThresh, targetThresh;
    cv::threshold(srcGray, srcThresh, 254, 255, CV_THRESH_BINARY);
    cv::threshold(targetGray, targetThresh, 254, 255, CV_THRESH_BINARY);

    // cv::Canny(srcGray, output, 254, 670, 3);//670 = CANNY_THRESH

    vector<cv::Vec3f> circles;
    cv::HoughCircles(srcThresh, circles, CV_HOUGH_GRADIENT, 1, 20, 200, 11, 0,
                     0);

    double targetHu[7];
    cv::HuMoments(cv::moments(targetThresh, true), targetHu);

    for (size_t i = 0; i < circles.size(); ++i) {
        int x = cvRound(circles[i][0]), y = cvRound(circles[i][1]),
            radius = cvRound(circles[i][2]);

        cv::Point center(x, y);

        // get the Rect containing the circle:
        cv::Rect r(center.x - radius, center.y - radius, radius * 2,
                   radius * 2);

        // obtain the image ROI:
        if (r.x >= 0 && r.y >= 0 && r.width + r.x < srcThresh.cols &&
            r.height + r.y < srcThresh.rows) {
            cv::Mat subImg(srcThresh, r);

            double h[7];
            cv::HuMoments(cv::moments(subImg, true), h);
            double diff = getDiff(targetHu, h);
            if (diff < errorRange) {
                cv::circle(output, center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
                cv::circle(output, center, radius, cv::Scalar(0, 0, 255), 3, 8,
                           0);
            }
        }
    }
}

TargetDetection::TargetDetection() {
}
/*
 * This method is the first step of the tracking algorithm
 * params: input: frame from the camera, output_*: output matrix with color,
 * output: general output matrix
 */
void TargetDetection::detect(const cv::Mat& input, cv::Mat& output_w,
                             cv::Mat& output_r, cv::Mat& output_g,
                             cv::Mat& output,
                             std::vector<RobotDesc>& outputRobotsArray) {
    // Shape detection
    //shapeDetection_.detect(input, output_w, output_r, output_g, output,
    //                      outputRobotsArray, blobDetection_);

    // Color detection
    blobDetection_.detect(input, output_w, output_r, output_g, output,
                          outputRobotsArray);



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


void TargetDetection::getRemoteParams(int color, int& h_max, int& h_min, int& s_max, int& s_min, int& v_max, int& v_min,int& preErode, int& dilate, int& postErode, int& blur)
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
    blur = (fun()->PRE_BLUR);
}

void TargetDetection::fetchRemoteParams(int color, int h_max, int h_min,
                                        int s_max, int s_min, int v_max,
                                        int v_min, int preErode, int dilate,
                                        int postErode, int blur) {
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

    *(fun()->H_MIN) = h_min;
    *(fun()->H_MAX) = h_max;
    *(fun()->S_MIN) = s_min;
    *(fun()->S_MAX) = s_max;
    *(fun()->V_MIN) = v_min;
    *(fun()->V_MAX) = v_max;
    *(fun()->PRE_EROSIONS) = preErode;
    *(fun()->DILATIONS) = dilate;
    *(fun()->POST_EROSIONS) = postErode;
    (fun()->PRE_BLUR) = blur;

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
