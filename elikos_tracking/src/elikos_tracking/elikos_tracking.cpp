#include <elikos_ros/RobotRaw.h>
#include <elikos_ros/RobotRawArray.h>
#include <geometry_msgs/Point.h>
#include <stdlib.h>  //for using the function sleep
#include <memory>
#include <unordered_map>
#include <vector>
#include "TrackingHandler.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"
#include "robot.h"
#include "ros/ros.h"
#include "std_msgs/Header.h"

/*void incertitudeCallback(const ros::TimerEvent& e) {
    ros::Duration diffTime = e.current_real - e.last_real;

    for (int id = 0; id < NUM_ROBOTS_PER_COLOR; id++) {
        redRobots.at(id)->updateIncertitude(diffTime.nsec);
        greenRobots.at(id + NUM_ROBOTS_PER_COLOR)
            ->updateIncertitude(diffTime.nsec);
    }
}*/

int main(int argc, char** argv) {
    cv::Mat_<cv::Vec3b> img(400, 400, cv::Vec3b(0, 0, 255));
    cv::namedWindow("Tracking-results", CV_WINDOW_AUTOSIZE);
    cv::imshow("Tracking-results", img);
    TrackingHandler::getInstance();
    ros::init(argc, argv, "tracking_node");

    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/elikos_target_robot_array", 1000,
                                      TrackingHandler::subCallback);

    // Timer pour calcul de l'incertitude
    // ros::Timer timer = n.createTimer(ros::Duration(0.1),
    // incertitudeCallback);
    ros::spin();

    cv::waitKey(1);

    return 0;
}
