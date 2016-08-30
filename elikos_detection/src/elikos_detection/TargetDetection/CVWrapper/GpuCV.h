#ifndef GPU_CV_H
#define GPU_CV_H

#include "CVWrapperInterface.h"
#include "opencv2/gpu/gpu.hpp"

class GpuCV : public CVWrapperInterface
{
public:
    GpuCV() = default;
    virtual ~GpuCV();

    virtual void cvtColor(const cv::Mat&, cv::Mat&, int code);
    virtual void blur(const cv::Mat&, cv::Mat&, cv::Size ksize, cv::Point anchor);
    virtual void erode(const cv::Mat&, cv::Mat&, cv::Mat kernel, cv::Point anchor, int iterations);
    virtual void dilate(const cv::Mat&, cv::Mat&, cv::Mat kernel, cv::Point anchor, int iterations);

};


#endif // GPU_CV_H
