#include "CpuCV.h"
CpuCV::~CpuCV()
{
}

void CpuCV::cvtColor(const cv::Mat& src, cv::Mat& dst, int code)
{
    cv::cvtColor(src, dst, code);
}

void CpuCV::blur(const cv::Mat& src, cv::Mat& dst, cv::Size ksize, cv::Point anchor)
{
    cv::blur(src, dst, ksize, anchor);
}

void CpuCV::erode(const cv::Mat& src, cv::Mat& dst, cv::Mat kernel, cv::Point anchor, int iterations)
{
    cv::erode(src, dst, kernel, anchor, iterations);
}

void CpuCV::dilate(const cv::Mat& src, cv::Mat& dst, cv::Mat kernel, cv::Point anchor, int iterations)
{
    cv::erode(src, dst, kernel, anchor, iterations);
}


