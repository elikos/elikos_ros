#ifndef CPU_CV_H
#define CPU_CV_H

#include "CVWrapperInterface.h"

class CpuCV : public CVWrapperInterface
{
public:
    CpuCV() = default;
    virtual ~CpuCV();

    virtual void cvtColor(const cv::Mat&, cv::Mat&, int code);
    virtual void blur(const cv::Mat&, cv::Mat&, cv::Size ksize, cv::Point anchor);
    virtual void erode(const cv::Mat&, cv::Mat&, cv::Mat kernel, cv::Point anchor, int iterations);
    virtual void dilate(const cv::Mat&, cv::Mat&, cv::Mat kernel, cv::Point anchor, int iterations);
};


#endif // CPU_CV_H
