#include "TrackingHandler.h"
#include "ros/ros.h"


int main(int argc, char** argv) {

    ros::init(argc, argv, "tracking_node");

    TrackingHandler trackHandler();

    ros::spin();

    return 0;
}
