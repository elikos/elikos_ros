#ifndef CAMERA_INFO_H
#define CAMERA_INFO_H

#include <string>

class CameraInfo
{
public:
    std::string frame;
    std::string topic;

    CameraInfo() = default;
    ~CameraInfo() = default;

    bool load(const std::string& cameraName);
};

#endif // CAMERA_INFO_H
