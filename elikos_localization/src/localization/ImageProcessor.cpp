//
// Created by olivier on 10/2/16.
//

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>

#include "LineGroup.h"

#include "ImageProcessor.h"

namespace localization {

ImageProcessor* ImageProcessor::instance_ = nullptr;

ImageProcessor* ImageProcessor::getInstance()
{
    if (instance_ == nullptr)
    {
        instance_ = new ImageProcessor();
    }
    return instance_;
}

void ImageProcessor::freeInstance()
{
    delete instance_;
    instance_ = nullptr;
}

ImageProcessor::ImageProcessor()
{
    // Init undistortion map
    cv::Mat distortedCamera = (cv::Mat_<float>(3,3) << 422.918640,    0.000000,    350.119451,
            0.000000,  423.121112,    236.380265,
            0.000000,    0.000000,      1.000000);

    cv::Mat cameraDistortion = (cv::Mat_<float>(1,5) << -0.321590, 0.089597, 0.001090, -0.000489, 0.000000);

    cv::Mat undistortedCamera = cv::getOptimalNewCameraMatrix(distortedCamera, cameraDistortion, cv::Size(640, 480), 0);

    cv::initUndistortRectifyMap(distortedCamera, cameraDistortion, cv::Mat(), undistortedCamera,
                                cv::Size(640, 480), CV_32FC1, distortionMap1_, distortionMap2_);
}

void undistort(const cv::Mat& src, cv::Mat& undistorted)
{
}

void ImageProcessor::preProcess(const cv::Mat& raw, cv::Mat& preProcessed)
{
    //undistort
    // Blur
    cv::Mat typeConverted;
    if (raw.type() != CV_8UC1) {
        cv::cvtColor(raw, typeConverted, CV_BGR2GRAY);
    } else {
        raw.copyTo(typeConverted);
    }

    cv::Mat undistorted;
    cv::remap(typeConverted, undistorted, distortionMap1_, distortionMap2_, CV_INTER_LINEAR);

    cv::Mat blured;
    cv::GaussianBlur(undistorted, blured, cv::Size(7,7), 8, 8);

    cv::Mat eroded;
    cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));
    erode(blured, eroded, element, cv::Point(0), 8);

    cv::Mat thresholded;
    cv::adaptiveThreshold(blured, thresholded, 200, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY, 551, 2);

    preProcessed = thresholded;
}

void ImageProcessor::processImage(cv::Mat input)
{
    image_ = input;
    cv::Mat preProcessed;
    preProcess(input, preProcessed);

    cv::Mat edges;
    findEdges(preProcessed, edges);

    cv::Mat lines = cv::Mat(input.size(), CV_8UC3, cv::Scalar(0, 0, 0));
    vLines_ = cv::Mat(input.size(), CV_8UC3, cv::Scalar(0, 0, 0));
    hLines_ = cv::Mat(input.size(), CV_8UC3, cv::Scalar(0, 0, 0));

    findLines(edges, lines);

    cv::imshow("vertical", vLines_);
    cv::imshow("horizontal", hLines_);
    cv::imshow("input", input);
    cv::imshow("lines", lines);
    cv::waitKey(30);
}
void ImageProcessor::findEdges(const cv::Mat& src, cv::Mat& edges)
{
    cv::Canny(src, edges, 1, 30, 3);
}

void ImageProcessor::drawRawLines(cv::Mat &dst, const std::vector<cv::Vec2f> &raw_lines) const {
    for( size_t i = 0; i < raw_lines.size() && i < 100; ++i )
    {
        float rho = raw_lines[i][0];
        float theta = raw_lines[i][1];

        cv::Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1.x = cvRound(x0 + 1000*(-b));
        pt1.y = cvRound(y0 + 1000*(a));
        pt2.x = cvRound(x0 - 1000*(-b));
        pt2.y = cvRound(y0 - 1000*(a));
        line(dst, pt1, pt2, cv::Scalar(100,100,100), 3, CV_AA);
    }
}

void ImageProcessor::drawLineGroup(cv::Mat& dst, const LineGroup& group)
{
    std::vector<Line*> lines = group.getLines();
    for( size_t i = 0; i < lines.size() && i < 100; ++i )
    {
        float rho = lines[i]->getRho();
        float theta = lines[i]->getTheta();

        cv::Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1.x = cvRound(x0 + 1000*(-b));
        pt1.y = cvRound(y0 + 1000*(a));
        pt2.x = cvRound(x0 - 1000*(-b));
        pt2.y = cvRound(y0 - 1000*(a));
        line(dst, pt1, pt2, cv::Scalar(100,100,100), 3, CV_AA);
    }
}

void ImageProcessor::findLines(const cv::Mat& edges, cv::Mat& lines)
{
    std::vector<cv::Vec2f> rawLines;
    cv::HoughLines(edges, rawLines, 1, CV_PI/180, 100, 0, 0 );
    drawRawLines(lines, rawLines);


    buildLineArray(rawLines);
    analyzeLineCluster();

}

void ImageProcessor::analyzeLineCluster()
{
    if (lineCluster_.size() == 0)
    {
        return;
    }

    const int RHO = 0;
    const int THETA = 1;

    Line uLine(lineCluster_[0]);
    LineGroup U(uLine);

    Line vLine = uLine;
    vLine.rotate(CV_PI / 2);
    LineGroup V(vLine);

    for (size_t i = 0; i < lineCluster_.size(); ++i)
    {
        cv::Vec2f w = lineCluster_[i].getOrientation();

        float udotw = std::abs(U.getAvgOrientation().dot(w));

        float vdotw = std::abs(V.getAvgOrientation().dot(w));

        if (udotw > vdotw ) {
            U.add(lineCluster_[i]);
        } else {
            V.add(lineCluster_[i]);
        }
    }

    drawLineGroup(vLines_, U);
    drawLineGroup(hLines_, V);
}

void ImageProcessor::buildLineArray(const std::vector<cv::Vec2f>& lineCluster)
{
    lineCluster_.clear();
    for (int i = 0; i < lineCluster.size(); ++i)
    {
        lineCluster_.push_back(Line(lineCluster[i][0], lineCluster[i][1]));
    }
}


}
