//
// Created by olivier on 06/07/16.
//


#ifndef GPU_CV_H
#define GPU_CV_H

#include "CVWrapperInterface.h"
#include "opencv2/gpu/gpu.hpp"

class GpuCV : public CVWrapperInterface
{
public:
    GpuCV() = default;
    virtual ~GpuCV();

    virtual void upload(const cv::Mat& src);
    virtual void download(cv::Mat& dst);

    virtual void cvtColor(int code, int dstCn = 0);
    virtual void blur(cv::Size ksize, cv::Point anchor=cv::Point(-1, -1));
    virtual void erode(cv::Mat kernel, cv::Point anchor = cv::Point(-1, -1), int iterations = 1);
    virtual void dilate(cv::Mat kernel, cv::Point anchor = cv::Point(-1, -1), int iterations=1);

private:
    cv::gpu::GpuMat mat_;
};


#endif // GPU_CV_H
