#ifndef CAMERA_INFO_H
#define CAMERA_INFO_H

#include <string>

#include <opencv2/imgproc/imgproc.hpp>

class CameraInfo
{
public:
    std::string frame;
    std::string topic;

    double hfov;
    double vfov;
    double height;
    double width;
    double focalLength;

    CameraInfo() = default;
    ~CameraInfo() = default;

    bool load(const std::string& cameraName);
};

#endif // CAMERA_INFO_H
