#include <math.h>
#include <cmath>

#include <ros/ros.h>

#include "CameraInfo.h"

bool CameraInfo::load(const std::string& cameraName) 
{
    bool success = ros::param::get(cameraName + "/topic", topic)  &&
              ros::param::get(cameraName + "/cam_fov_h", hfov)    &&
              ros::param::get(cameraName + "/cam_fov_v", vfov)    &&
              ros::param::get(cameraName + "/cam_height", height) &&
              ros::param::get(cameraName + "/cam_width", width)   &&
              ros::param::get(cameraName + "/frame", frame)       &&
              ros::param::get(cameraName + "/bw_threshold", threshold);

    hfov *= M_PI / 180.0;
    vfov *= M_PI / 180.0;

    focalLength = width / (2 * std::tan(hfov / 2.0));
    name = cameraName;

    return success;
}