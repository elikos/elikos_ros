//
// Created by olivier on 06/07/16.
//

#include "GpuCV.h"

GpuCV::~GpuCV()
{
}

void GpuCV::upload(const cv::Mat& src)
{
    mat_.upload(src);
}

void GpuCV::download(cv::Mat& dst)
{
    mat_.download(dst);
}

void GpuCV::cvtColor(int code, int dstCn)
{
    cv::gpu::cvtColor(mat_, mat_, code, dstCn);
}

void GpuCV::blur(cv::Size ksize, cv::Point anchor)
{
    cv::gpu::blur(mat_, mat_, ksize, anchor);
}

void GpuCV::erode(cv::Mat kernel, cv::Point anchor, int iterations)
{
    cv::gpu::erode(mat_, mat_, kernel, anchor, iterations);
}

void GpuCV::dilate(cv::Mat kernel, cv::Point anchor, int iterations)
{
    cv::gpu::dilate(mat_, mat_, kernel, anchor, iterations);
}

