//
// Created by olivier on 06/07/16.
//

#ifndef CPU_CV_H
#define CPU_CV_H

#include "CVWrapperInterface.h"

class CpuCV : public CVWrapperInterface
{
public:
    CpuCV() = default;
    virtual ~CpuCV();

    virtual void upload(const cv::Mat& src);
    virtual void download(cv::Mat& dst);

    virtual void cvtColor(int code, int dstCn = 0);
    virtual void blur(cv::Size ksize, cv::Point anchor=cv::Point(-1, -1));
    virtual void erode(cv::Mat kernel, cv::Point anchor = cv::Point(-1, -1), int iterations = 1);
    virtual void dilate(cv::Mat kernel, cv::Point anchor = cv::Point(-1, -1), int iterations=1);
private:
    cv::Mat mat_;
};


#endif // CPU_CV_H
