//
// Created by olivier on 06/07/16.
//

#ifndef CV_WRAPPER_INTERFACE
#define CV_WRAPPER_INTERFACE

#include <opencv2/opencv.hpp>

class CVWrapperInterface
{
public:
    CVWrapperInterface() = default;
    virtual ~CVWrapperInterface() = 0;

    virtual void upload(const cv::Mat& src) = 0;
    virtual void download(cv::Mat& dst) = 0;

    virtual void cvtColor(int code, int dstCn = 0) = 0;
    virtual void blur(cv::Size ksize, cv::Point anchor=cv::Point(-1, -1)) = 0;
    virtual void erode(cv::Mat kernel, cv::Point anchor = cv::Point(-1, -1), int iterations = 1) = 0;
    virtual void dilate(cv::Mat kernel, cv::Point anchor = cv::Point(-1, -1), int iterations=1) = 0;
};

inline CVWrapperInterface::~CVWrapperInterface() { }

#endif // CV_WRAPPER_INTERFACE
