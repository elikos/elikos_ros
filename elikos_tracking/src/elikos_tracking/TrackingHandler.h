#include <vector>
#include "modele/robot.h"
#include <elikos_msgs/TargetRobot.h>
#include <elikos_msgs/TargetRobotArray.h>
#include <geometry_msgs/Point.h>
#include <stdlib.h>  //for using the function sleep
#include <unordered_map>
#include <memory>
#include <vector>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

const int NUM_ROBOTS_PER_COLOR = 1;


class TrackingHandler {
    public:

    TrackingHandler();

    void drawResultImage();

    void MatchRobots(std::vector<double>& ModelMsgDistances, const elikos_msgs::TargetRobotArray::ConstPtr& msg);
    void AssignRobots(const elikos_msgs::TargetRobotArray::ConstPtr& msg);

    void subCallback(const elikos_msgs::TargetRobotArray::ConstPtr& msg);
    void incertitudeCallback(const ros::TimerEvent& e);
    void publishTargets();

   private:

    ros::Publisher targetsPub_;
    ros::Subscriber targetsSub_;
ros::Timer timer;
    //Debug purposes
    visualization_msgs::Marker marker_;
    ros::Publisher debugPub_;

    std::vector<std::shared_ptr<Robot>> robotsVec_;

};
