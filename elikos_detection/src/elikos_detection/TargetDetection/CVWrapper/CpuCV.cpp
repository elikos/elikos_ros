//
// Created by olivier on 06/07/16.
//

#include "CpuCV.h"

CpuCV::~CpuCV()
{
}

void CpuCV::upload(const cv::Mat& src)
{
    mat_ = src;
}

void CpuCV::download(cv::Mat& dst)
{
    dst = mat_;
}

void CpuCV::cvtColor(int code, int dstCn)
{
    cv::cvtColor(mat_, mat_, code, dstCn);
}

void CpuCV::blur(cv::Size ksize, cv::Point anchor)
{
    cv::blur(mat_, mat_, ksize, anchor);
}

void CpuCV::erode(cv::Mat kernel, cv::Point anchor, int iterations)
{
    cv::erode(mat_, mat_, kernel, anchor, iterations);
}

void CpuCV::dilate(cv::Mat kernel, cv::Point anchor, int iterations)
{
    cv::erode(mat_, mat_, kernel, anchor, iterations);
}


