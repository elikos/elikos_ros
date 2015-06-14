//
// Created by tonio on 12/06/15.
//

#include <ros/ros.h>
#include <elikos_lib/RCReceiver.h>
#include "SetpointManager.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "elikos_navigator");
    ros::NodeHandle nh;
    SetpointManager spManager_(nh);
    return 0;
}