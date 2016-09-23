#ifndef CV_WRAPPER_INTERFACE
#define CV_WRAPPER_INTERFACE

#include <opencv2/opencv.hpp>

class CVWrapperInterface
{
public:
    CVWrapperInterface() = default;
    virtual ~CVWrapperInterface() = 0;
    virtual void cvtColor(const cv::Mat &src, cv::Mat &dst, int code) = 0;
    virtual void blur(const cv::Mat &src, cv::Mat &dst, cv::Size ksize, cv::Point anchor) = 0;
    virtual void erode(const cv::Mat &src, cv::Mat &dst, cv::Mat kernel, cv::Point anchor, int iterations) = 0;
    virtual void dilate(const cv::Mat &src, cv::Mat &dst, cv::Mat kernel, cv::Point anchor, int iterations) = 0;
};

inline CVWrapperInterface::~CVWrapperInterface() { }

#endif // CV_WRAPPER_INTERFACE
