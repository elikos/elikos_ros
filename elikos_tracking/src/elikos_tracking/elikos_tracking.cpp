#include <geometry_msgs/Point.h>
#include <memory>
#include <unordered_map>
#include <vector>
#include "TrackingHandler.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"
#include "robot.h"
#include "ros/ros.h"
#include "std_msgs/Header.h"


int main(int argc, char** argv) {

    ros::init(argc, argv, "tracking_node");

    TrackingHandler trackHandler();

    ros::spin();

    return 0;
}
