#include <ros/ros.h>

#include "CameraInfo.h"

bool CameraInfo::load(const std::string& cameraName) 
{
    bool success = true;
    success = success && ros::param::get(cameraName + "/topic", topic);
    success = success && ros::param::get(cameraName + "/frame", frame);
    return success;
}