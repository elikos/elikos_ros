#include "bootstrap.h"
#include "ros/ros.h"
#include "camera1394/Camera1394Config.h"
#include "camera1394/GetCameraRegisters.h"
#include "camera1394/SetCameraRegisters.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "bootstrap");
    ROS_INFO("Hello eva!");

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<camera1394::GetCameraRegisters>("GetCameraRegisters");
    camera1394::GetCameraRegisters getCameraRegisters;

    //getCameraRegisters.request.
    if(client.call(getCameraRegisters)){

    }
    return 0;
}
