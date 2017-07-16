#include <vector>
#include "robot.h"
#include <elikos_ros/TargetRobot.h>
#include <elikos_ros/TargetRobotArray.h>
#include <geometry_msgs/Point.h>
#include <stdlib.h>  //for using the function sleep
#include <unordered_map>
#include <memory>
#include <vector>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

const int NUM_ROBOTS_PER_COLOR = 5;


class TrackingHandler {
    public:

    static TrackingHandler* getInstance();

    std::shared_ptr<Robot> getRobotAtIndex(int index);
    void clearRobots();
    void drawResultImage();
    void MatchRobots(std::vector<double>& ModelMsgDistances, const elikos_ros::TargetRobotArray::ConstPtr& msg);
    void AssignRobots(const elikos_ros::TargetRobotArray::ConstPtr& msg);
    static void subCallback(const elikos_ros::TargetRobotArray::ConstPtr& msg);
    static void incertitudeCallback(const ros::TimerEvent& e);
    void publishTargets();

   private:

    TrackingHandler();
    static TrackingHandler* handlerInstance_;
    ros::Publisher targetsPub_;
    ros::Publisher debugPub_;
    visualization_msgs::Marker marker_;

    //std::vector<std::unique_ptr<Robot>> redRobots_;
    //std::vector<std::unique_ptr<Robot>> greenRobots_;
    std::vector<std::shared_ptr<Robot>> robotsVec_;

};
