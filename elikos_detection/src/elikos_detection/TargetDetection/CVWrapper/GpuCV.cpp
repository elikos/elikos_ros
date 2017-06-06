// #include "GpuCV.h"

// GpuCV::~GpuCV()
// {
// }

// void GpuCV::cvtColor(const cv::Mat& src, cv::Mat& dst, int code)
// {
//     cv::cuda::GpuMat srcGPU, dstGPU;
//     srcGPU.upload(src);
//     dstGPU.upload(dst);

//     cv::cuda::cvtColor(srcGPU, dstGPU, code);

//     dstGPU.download(dst);
// }

// void GpuCV::blur(const cv::Mat& src, cv::Mat& dst, cv::Size ksize, cv::Point anchor)
// {
//     cv::gpu::GpuMat srcGPU, dstGPU;
//     srcGPU.upload(src);
//     dstGPU.upload(dst);

//     cv::gpu::blur(srcGPU, dstGPU, ksize, anchor);

//     dstGPU.download(dst);
// }

// void GpuCV::erode(const cv::Mat& src, cv::Mat& dst, cv::Mat kernel, cv::Point anchor, int iterations)
// {
//     cv::gpu::GpuMat srcGPU, dstGPU;
//     srcGPU.upload(src);
//     dstGPU.upload(dst);

//     cv::gpu::erode(srcGPU, dstGPU, kernel, anchor, iterations);

//     dstGPU.download(dst);
// }

// void GpuCV::dilate(const cv::Mat& src, cv::Mat& dst, cv::Mat kernel, cv::Point anchor, int iterations)
// {
//     cv::gpu::GpuMat srcGPU, dstGPU;
//     srcGPU.upload(src);
//     dstGPU.upload(dst);

//     cv::gpu::dilate(srcGPU, dstGPU, kernel, anchor, iterations);

//     dstGPU.download(dst);
// }

